<!--
Copyright (c) Meta Platforms, Inc. and affiliates.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
-->

# Muse · Lichuang SZP ESP32-S3 Port

[中文](README.md) | English

A port of Muse for the **Lichuang SZP ESP32-S3 N16R8**, focused on the **2-inch, 320×240 landscape** display: UI layout, Chinese/English display, and screen and provisioning fixes.

Target configuration: **16 MB Flash / 8 MB PSRAM, ST7789 display, FT6336 touch**.

## What this repository changes

- **Landscape UI**: adjusted settings lists, titles, input fields, and the four-row keyboard; fixed input overflow and layout misalignment.
- **Chinese/English switching**: Chinese by default, switchable live in Settings (choice persists across reboots); completed the Chinese font coverage.
- **Display and touch**: fixed backlight control, LCD initialization and chip-select timing, and touch coordinate mapping.
- **Provisioning memory**: shrank the display refresh buffer, fixing a provisioning task that could not start for lack of contiguous memory.
- **Audio**: wired up the on-board ES8311 speaker and ES7210 dual microphones.

## New in this fork: Camera Mode (2026-11)

A feature of this fork only — neither the official firmware nor the upstream port has it:

- **Button camera mode**: double-tap the voice button to enter; live viewfinder (VGA frames downsampled onto the panel); short-press to shoot and save to the SD card with an on-screen freeze confirmation; long-press to exit and resume voice. The voice session is suspended while in camera mode.
- **Photos back to the chat**: on exit the board reports the shot count to the chat, which fetches them with `camera.saved_list` / `camera.fetch`; every original carries a 320×240 delivery copy (`IMG_NNNNS.jpg`) that a chat can restore and view directly.
- **Remote capture with thumbnail**: `camera.capture` returns the full image plus a 320×240 copy (`thumb_base64`).
- **Software mirror fix**: the GC0308 hmirror register proved to be a no-op on this board, so frames are flipped in software (YUV422, row by row) — capture, viewfinder, and remote shots all agree.
- **Camera memory root fix**: `CONFIG_CAMERA_DMA_BUFFER_SIZE_MAX=16384` ends the intermittent camera init failures caused by the DMA allocation (30,720 B) exceeding the firmware's largest steady-state contiguous heap block (26,624 B).

Note: `components/camera/camera_gc0308.c` is an early reference implementation and is not built; the active backend is `camera_dvp.c`. This release does not include the local-TTS experiment code. Build: `tools/muse/board.sh build szp`.

**Guide and troubleshooting**: [Camera Mode Guide](docs/lichuang/camera-mode.en.md) — working notes for AI assistants, build/flash steps, symptom-based troubleshooting, and rollback ([中文](docs/lichuang/camera-mode.md)).

## Screenshots

| Chinese input page | English input page |
|---|---|
| ![Chinese landscape UI](docs/lichuang/images/input-zh.png) | ![English landscape UI](docs/lichuang/images/input-en.png) |

## Usage

Take the **`main`** branch and follow the [setup and flashing guide](docs/lichuang/getting-started.md) (Chinese): select the SZP preset, fill in your own SDK token, build, and flash. The port code is all in the repository; no UI files need copying one by one.

[UI usage and modification](docs/lichuang/ui.md) · [Troubleshooting](docs/lichuang/troubleshooting.md) · [Board pins](esp32/devices/lichuang-szp.md) · [Validation record](docs/lichuang/validation.md) (all Chinese)

On-hardware records confirm display, touch, audio, pairing, and Wi-Fi; 224 host tests pass on the merged main branch. **Cloud conversation is not yet verified.** The Chinese UI has no pinyin input method yet. Hardware validation covers only the board listed above.

Based on the [Muse Gadget SDK](https://github.com/facebookincubator/muse-gadget-sdk); see [LICENSE](LICENSE) and [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
