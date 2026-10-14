# sunnypilot feature memory for Subaru

Remembers **Auto Vehicle Hold (AVH)** and **Auto Start Stop (ASS)** across restarts on a 2020-22 Outback/Legacy.
The car resets both at every start. This learns your choice from real button presses and restores it a few
seconds after the engine starts, by resending copies of the car's own button messages.

The branch to install is [`release-tizi-feature-memory`](../../tree/release-tizi-feature-memory): sunnypilot's
prebuilt `release-tizi` (comma 3X) plus [`feature-memory.patch`](feature-memory.patch), with the panda firmware
rebuilt to allow those two button messages (alt bus only, button presses only).

[`update.yml`](.github/workflows/update.yml) rebuilds it every 6 hours if sunnypilot has released a new
`release-tizi`. It refuses to build unless the toolchain reproduces sunnypilot's shipped firmware byte for byte,
and it pushes nothing if the patch no longer applies.

The remembered states are in `/data/media/0/subaru_feature_memory.json` on the device. Set `"enabled": false`
there to turn the feature off (takes effect at the next start).
