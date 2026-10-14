#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.sunnypilot.car.subaru.values_ext import SubaruSafetyFlagsSP
import opendbc.safety.tests.common as common
import opendbc.safety.tests.test_subaru as test_subaru  # module import, so pytest doesn't collect its tests here too
from opendbc.safety.tests.test_subaru import SUBARU_ALT_BUS, SUBARU_CAM_BUS, SUBARU_MAIN_BUS, lkas_tx_msgs

AVH_REQUEST = 0x6bb
DASHLIGHTS = 0x390

# idle frames recorded on a 2022 Outback
AVH_REQUEST_IDLE = bytes.fromhex("cd 05 00 01 00 00 06 00")
DASHLIGHTS_IDLE = bytes.fromhex("cb 05 14 47 7e 5a 00 00")


def with_byte(dat: bytes, index: int, value: int) -> bytes:
  out = bytearray(dat)
  out[index] = value
  return bytes(out)


class TestSubaruGen2FeatureMemorySafety(test_subaru.TestSubaruGen2TorqueStockLongitudinalSafety):
  TX_MSGS = lkas_tx_msgs(SUBARU_ALT_BUS) + [[DASHLIGHTS, SUBARU_ALT_BUS], [AVH_REQUEST, SUBARU_ALT_BUS]]
  SAFETY_PARAM_SP = SubaruSafetyFlagsSP.FEATURE_MEMORY

  def setUp(self):
    super().setUp()
    self.safety.set_current_safety_param_sp(self.SAFETY_PARAM_SP)
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, self.FLAGS)
    self.safety.init_tests()

  def tearDown(self):
    self.safety.set_current_safety_param_sp(0)

  def _tx_raw(self, addr: int, bus: int, dat: bytes) -> bool:
    return bool(self.safety.safety_tx_hook(common.make_msg(bus, addr, 8, dat)))

  def _states(self):
    # the button copies don't depend on engagement or speed
    for controls_allowed in (False, True):
      for speed in (0, 30):
        self.safety.set_controls_allowed(controls_allowed)
        self.assertTrue(self._rx(self._speed_msg(speed)))
        yield

  def test_avh_request(self):
    for _ in self._states():
      for request, allowed in ((0b00, False), (0b01, True), (0b10, True), (0b11, False)):
        dat = with_byte(AVH_REQUEST_IDLE, 2, request)
        self.assertEqual(allowed, self._tx_raw(AVH_REQUEST, SUBARU_ALT_BUS, dat), f"request {request:#04b}")

  def test_ass_button(self):
    for _ in self._states():
      for byte6, allowed in ((0x00, False), (0x01, False), (0x40, True), (0x41, True)):
        dat = with_byte(DASHLIGHTS_IDLE, 6, byte6)
        self.assertEqual(allowed, self._tx_raw(DASHLIGHTS, SUBARU_ALT_BUS, dat), f"byte 6 = {byte6:#04x}")

  def test_alt_bus_only(self):
    for bus in (SUBARU_MAIN_BUS, SUBARU_CAM_BUS):
      self.assertFalse(self._tx_raw(AVH_REQUEST, bus, with_byte(AVH_REQUEST_IDLE, 2, 0b10)))
      self.assertFalse(self._tx_raw(DASHLIGHTS, bus, with_byte(DASHLIGHTS_IDLE, 6, 0x40)))


class TestSubaruGen2WithoutFeatureMemorySafety(test_subaru.TestSubaruGen2TorqueStockLongitudinalSafety):
  def test_button_copies_blocked(self):
    for bus in (SUBARU_MAIN_BUS, SUBARU_ALT_BUS, SUBARU_CAM_BUS):
      self.assertFalse(self.safety.safety_tx_hook(common.make_msg(bus, AVH_REQUEST, 8, with_byte(AVH_REQUEST_IDLE, 2, 0b10))))
      self.assertFalse(self.safety.safety_tx_hook(common.make_msg(bus, DASHLIGHTS, 8, with_byte(DASHLIGHTS_IDLE, 6, 0x40))))


if __name__ == "__main__":
  unittest.main()
