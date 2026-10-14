"""
Remembers Auto Vehicle Hold (AVH) and Auto Start Stop (ASS) across restarts on 2020-22 Outback/Legacy.

The car turns AVH off and ASS back on at every start. This learns the driver's choice from real button presses
and, once the engine has been running for a few seconds, restores it by sending copies of the car's own button
messages on the alt bus, where the brake and engine computers listen. Decoded on a 2022 Outback:

  AVH  status   0x32b byte 5 bit 5 (1 = on)
       request  0x6bb byte 2: bit 1 = turn on, bit 0 = turn off. Explicit, so repeating it is harmless.
  ASS  state    0x174 byte 4 bits 7-6: 0 = enabled, 3 = turned off by the driver, 2 = enabled but engine off.
                The button also works with the engine off, toggling 2 <-> 3.
       button   0x390 (Dashlights) byte 6 bit 6. The engine computer toggles ASS on every 0 -> 1 edge, and the
                car sends its own Dashlights (bit = 0) every 50-100 ms, so exactly one pressed frame is sent,
                right after a real frame, and the car's next frame acts as the release.

All four messages use the standard Subaru checksum (byte 0) and counter (low nibble of byte 1). Copies keep
every other byte of the latest real frame. The remembered states live in MEMORY_PATH as JSON:
  {"enabled": true, "avh": true, "ass_off": true}
Set "enabled" to false to turn the feature off (takes effect at the next start).
"""
import json
import os

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.carlog import carlog
from opendbc.car.subaru.values import CAR, DBC, CanBus

from opendbc.sunnypilot.car.subaru.values_ext import SubaruFlagsSP

MEMORY_PATH = os.environ.get("SUBARU_FEATURE_MEMORY_PATH", "/data/media/0/subaru_feature_memory.json")
SUPPORTED_PLATFORMS = (CAR.SUBARU_OUTBACK, CAR.SUBARU_LEGACY)

AVH_STATUS, AVH_REQUEST = 0x32B, 0x6BB
ENGINE_STOP_START, DASHLIGHTS = 0x174, 0x390
WATCHED = (AVH_STATUS, AVH_REQUEST, ENGINE_STOP_START, DASHLIGHTS)

ASS_ENABLED, ASS_ENGINE_OFF, ASS_OFF = 0, 2, 3

# the raw frame tap sits in the CarState parser dict under a key Subaru doesn't otherwise use
TAP_KEY = Bus.body

# timings in control frames (100 Hz)
SETTLE_FRAMES = 300     # engine running this long before restoring
VERIFY_FRAMES = 100     # the state must change this soon after an action
RETRY_FRAMES = 100      # pause between attempts
MAX_ATTEMPTS = 3
LEARN_FRAMES = 100      # read the resulting state this long after a real button press
AVH_FRAME_GAP = 5       # 50 ms between AVH request frames, like the real burst
AVH_REQUEST_FRAMES = 4  # the real burst carries the request in its first 4 frames

# freshness limits in parser updates (100 Hz)
STATUS_MAX_AGE = 50     # 0x32b is 10 Hz, 0x174 is 50 Hz
AVH_BASE_MAX_AGE = 200  # 0x6bb is only 1 Hz


def is_supported(CP: structs.CarParams) -> bool:
  return CP.carFingerprint in SUPPORTED_PLATFORMS and not CP.openpilotLongitudinalControl


def load_memory() -> dict:
  try:
    with open(MEMORY_PATH) as f:
      memory = json.load(f)
    return memory if isinstance(memory, dict) else {}
  except (OSError, ValueError):
    return {}


def save_memory(memory: dict) -> None:
  try:
    tmp = MEMORY_PATH + ".tmp"
    with open(tmp, "w") as f:
      json.dump(memory, f)
    os.replace(tmp, MEMORY_PATH)
  except OSError as e:
    carlog.error(f"subaru feature memory: could not save {MEMORY_PATH}: {e}")


def is_enabled() -> bool:
  return load_memory().get("enabled", True) is not False


def subaru_checksum(addr: int, dat: bytes | bytearray) -> int:
  return ((addr & 0xFF) + ((addr >> 8) & 0xFF) + sum(dat[1:])) & 0xFF


def copy_frame(addr: int, dat: bytearray, counter: int) -> CanData:
  dat[1] = (dat[1] & 0xF0) | (counter & 0xF)
  dat[0] = subaru_checksum(addr, dat)
  return CanData(addr, bytes(dat), CanBus.alt)


class RawFrameTap(CANParser):
  """Keeps the raw payloads of a few messages. It registers no DBC messages, so it can't affect CAN validity."""
  def __init__(self, dbc_name: str, bus: int, addresses: tuple[int, ...]):
    super().__init__(dbc_name, [], bus)
    self.latest: dict[int, bytes] = {}
    self.age: dict[int, int] = {}  # parser updates since the latest frame
    self.new_frames: dict[int, list[bytes]] = {a: [] for a in addresses}

  def update(self, strings, sendcan: bool = False):
    if strings and not isinstance(strings[0], list | tuple):
      strings = [strings]

    for addr in self.age:
      self.age[addr] += 1
    for frames in self.new_frames.values():
      frames.clear()

    for entry in strings:
      for address, dat, src in entry[1]:
        if src == self.bus and address in self.new_frames:
          self.new_frames[address].append(bytes(dat))
          self.latest[address] = bytes(dat)
          self.age[address] = 0

    return super().update(strings, sendcan)

  def fresh(self, addr: int, max_age: int) -> bytes | None:
    return self.latest[addr] if self.age.get(addr, max_age + 1) <= max_age else None


class FeatureMemoryCarState:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.feature_memory_tap: RawFrameTap | None = None

  def update_feature_memory(self, can_parsers) -> None:
    self.feature_memory_tap = can_parsers.get(TAP_KEY)

  @staticmethod
  def add_feature_memory_parser(parsers: dict, CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> dict:
    if CP_SP.flags & SubaruFlagsSP.FEATURE_MEMORY:
      parsers[TAP_KEY] = RawFrameTap(DBC[CP.carFingerprint][Bus.pt], CanBus.alt, WATCHED)
    return parsers


class RememberedFeature:
  def __init__(self, name: str, key: str, memory: dict):
    self.name = name
    self.key = key
    desired = memory.get(key)
    self.desired: bool | None = desired if isinstance(desired, bool) else None
    self.done = self.desired is None  # nothing to restore until a choice has been learned
    self.attempts = 0
    self.next_action_frame = 0
    self.verify_until: int | None = None
    self.learn_at: int | None = None
    self.button_prev = False

  def finish(self, msg: str | None = None) -> None:
    self.done = True
    self.verify_until = None
    if msg:
      carlog.warning(f"subaru feature memory: {self.name} {msg}")

  def ready_to_act(self, current: bool | None, frame: int) -> bool:
    """Tracks an ongoing restore. Returns True when a (new) action should be sent now."""
    if self.done or self.desired is None or current is None or self.learn_at is not None:
      return False
    if self.verify_until is not None:
      if current == self.desired:
        self.finish(f"restored to {self.desired} after {self.attempts} attempt(s)")
        return False
      if frame < self.verify_until:
        return False
      self.verify_until = None
      if self.attempts >= MAX_ATTEMPTS:
        self.finish(f"gave up restoring to {self.desired} after {self.attempts} attempts")
        return False
    if current == self.desired:
      self.finish()
      return False
    return frame >= self.next_action_frame

  def acted(self, frame: int, duration: int = 0) -> None:
    self.attempts += 1
    self.verify_until = frame + duration + VERIFY_FRAMES
    self.next_action_frame = self.verify_until + RETRY_FRAMES

  def saw_button(self, pressed: bool, frame: int) -> None:
    # a real press: the driver takes over, so stop restoring and learn the result shortly after
    if pressed and not self.button_prev:
      self.learn_at = frame + LEARN_FRAMES
      if not self.done:
        self.finish("restore cancelled by a real button press")
    self.button_prev = pressed

  def learn(self, current: bool | None, frame: int, memory: dict) -> None:
    if self.learn_at is None or frame < self.learn_at:
      return
    self.learn_at = None
    if current is not None and current != self.desired:
      self.desired = current
      memory[self.key] = current
      save_memory(memory)
      carlog.warning(f"subaru feature memory: remembered {self.name} = {current}")


class FeatureMemoryCarController:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.feature_memory_enabled = bool(CP_SP.flags & SubaruFlagsSP.FEATURE_MEMORY)
    self.memory = load_memory() if self.feature_memory_enabled else {}
    self.avh = RememberedFeature("AVH", "avh", self.memory)
    self.ass_off = RememberedFeature("ASS off", "ass_off", self.memory)
    self.engine_on_frames = 0
    self.avh_burst: list[tuple[int, CanData]] = []

  def create_feature_memory(self, CS, frame: int) -> list[CanData]:
    tap = CS.feature_memory_tap
    if not self.feature_memory_enabled or tap is None:
      return []

    status = tap.fresh(AVH_STATUS, STATUS_MAX_AGE)
    avh_on = bool(status[5] & 0x20) if status is not None else None
    state = tap.fresh(ENGINE_STOP_START, STATUS_MAX_AGE)
    ass_state = state[4] >> 6 if state is not None else None
    ass_off = {ASS_ENABLED: False, ASS_ENGINE_OFF: False, ASS_OFF: True}.get(ass_state) if ass_state is not None else None

    # restore only with the engine running: it's unknown whether a choice made with the engine off survives a start
    engine_running = ass_state in (ASS_ENABLED, ASS_OFF)
    self.engine_on_frames = self.engine_on_frames + 1 if engine_running else 0

    # real presses only: our own copies are transmitted, never received, so they don't show up here
    for dat in tap.new_frames[AVH_REQUEST]:
      self.avh.saw_button(bool(dat[2] & 0x03), frame)
    for dat in tap.new_frames[DASHLIGHTS]:
      self.ass_off.saw_button(bool(dat[6] & 0x40), frame)
    self.avh.learn(avh_on, frame, self.memory)
    self.ass_off.learn(ass_off, frame, self.memory)

    can_sends = []
    if self.engine_on_frames >= SETTLE_FRAMES:
      if not self.avh_burst and self.avh.ready_to_act(avh_on, frame):
        self.queue_avh_request(tap, frame)
      if self.ass_off.ready_to_act(ass_off, frame):
        can_sends += self.press_ass(tap, frame)

    while self.avh_burst and self.avh_burst[0][0] <= frame:
      can_sends.append(self.avh_burst.pop(0)[1])
    return can_sends

  def queue_avh_request(self, tap: RawFrameTap, frame: int) -> None:
    base = tap.fresh(AVH_REQUEST, AVH_BASE_MAX_AGE)
    if base is None:
      return
    request = 0x02 if self.avh.desired else 0x01
    counter = base[1] & 0xF
    for i in range(AVH_REQUEST_FRAMES):
      dat = bytearray(base)
      dat[2] = (dat[2] & ~0x03) | request
      self.avh_burst.append((frame + i * AVH_FRAME_GAP, copy_frame(AVH_REQUEST, dat, counter + 1 + i)))
    self.avh.acted(frame, AVH_REQUEST_FRAMES * AVH_FRAME_GAP)
    carlog.warning(f"subaru feature memory: requesting AVH {'on' if self.avh.desired else 'off'} (attempt {self.avh.attempts})")

  def press_ass(self, tap: RawFrameTap, frame: int) -> list[CanData]:
    # only right after a real Dashlights frame, so the car's next frame releases the button
    if tap.age.get(DASHLIGHTS) != 0:
      return []
    dat = bytearray(tap.latest[DASHLIGHTS])
    dat[6] |= 0x40
    self.ass_off.acted(frame)
    carlog.warning(f"subaru feature memory: pressing ASS to turn it {'off' if self.ass_off.desired else 'on'} (attempt {self.ass_off.attempts})")
    return [copy_frame(DASHLIGHTS, dat, (dat[1] & 0xF) + 1)]
