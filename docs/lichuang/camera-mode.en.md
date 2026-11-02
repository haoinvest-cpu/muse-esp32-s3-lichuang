# Camera Mode — User and Troubleshooting Guide

An original feature of this fork: a complete on-board camera (GC0308) for the Muse voice firmware — button entry, live viewfinder, photos saved to the SD card, photos sent back to the chat for recognition, and remote capture with a thumbnail. Neither the official firmware nor the upstream port has any of this.

Chinese version: [camera-mode.md](camera-mode.md) · Back to the [repository home](../../README.en.md)

---

## Notes for AI assistants (read this section first)

If you are an AI assistant (agent) asked to help build, flash, or troubleshoot this firmware, read this section first, then start at step 0.

### Goal

Get the board running the camera-mode firmware and walk the user through acceptance: double-tap into the camera, take a photo, exit, and have the photo come back to the chat and be described.

### Working conventions

- **Build and flash are two separate actions**: run `tools/muse/board.sh build szp` and `board.sh flash szp` separately; `flash` only writes whatever binaries already sit in the build directory — it never compiles. After changing code, `build` first and watch it finish successfully, then `flash`. Flashing directly burns the *old* firmware.
- Watch build progress with exactly one command: `tail -f /tmp/muse_build_szp.log`. Attach it to every build command you give the user.
- Commands given to the user must be final, runnable values — no placeholders for the user to fill in.
- Work one step at a time; wait for the user to paste the output before giving the next step.
- Verify the flash package's md5 before flashing (build machine, transfer hop, and local copy must agree).

### Operations that need the user's explicit consent first

- Flashing firmware to the board (any version, including a rollback).
- Removing the SD card to read photos (the card holds the only copy of the originals; make sure nothing is writing first).
- Writing any credential (SDK token, WiFi password) into a repository file or a chat transcript.

### Operations only the user can do

- Plugging/unplugging USB, pressing the voice button (double-tap / short / long press), holding up a subject for acceptance shots.
- Logging in to their own accounts, entering passwords and tokens, merging GitHub PRs.

### Never do these on your own initiative

- Pulling the plug when the camera does not respond — gather evidence first (section 3), power-cycle last.
- Flashing a stale build while presenting it as new; retrying a failed build verbatim without reading the log.
- Committing real credentials to a public repository — token-like settings in `devices/sdkconfig.muse-lichuang-szp` must be emptied before any commit.

---

## 0. Preparation

- **Board**: Lichuang SZP ESP32-S3 N16R8, GC0308 camera on the J1 header; FAT32 SD card, photos in `/DCIM`.
- **Firmware**: this repository's `main` (camera mode included since the cammode series), built with the `szp` preset.
- **Computer**: ESP-IDF v6.0.1 environment; serial port `/dev/ttyUSB0` (CH340). Attaching the serial monitor resets the board once via EN — that is normal, not a crash.

## 1. Build and flash

### 1.1 Build

From the `esp32/` directory:

```bash
tools/muse/board.sh build szp
tail -f /tmp/muse_build_szp.log
```

A successful build ends with `Project build complete` in the log.

### 1.2 Package a flash pack (for transfer and rollback)

From `build-muse-lichuang-szp/`, take six files in this layout and record the tarball's md5:

```text
bootloader/bootloader.bin
partition_table/partition-table.bin
ota_data_initial.bin
srmodels/srmodels.bin
muse-gadget.bin
flash_args
```

### 1.3 Flash (exit the serial monitor first; keep a rollback pack before you flash)

The serial monitor holds `/dev/ttyUSB0`; exit it first (`Ctrl+]` in the monitor). Inside the unpacked flash-pack directory, with the IDF environment sourced:

```bash
python -m esptool --chip esp32s3 -p /dev/ttyUSB0 -b 460800 --before default-reset --after hard-reset write-flash @flash_args
```

A rollback pack is simply the previous final version's flash pack, flashed the same way. **Keep the rollback pack before flashing a new version.**

### 1.4 First-boot check

The screen lights up and voice wake works; a chat-side `camera.capture` returns both the full image and a thumbnail. That is a healthy boot.

## 2. Using camera mode

### 2.1 Enter: double-tap the voice button (both taps within 600 ms)

The screen switches to a live viewfinder (VGA frames downsampled onto the panel) with a status line. The voice session is suspended meanwhile; if the first tap caught a spoken fragment, it is held and sent after you exit. Known trade-off: the first tap can very rarely swallow an ultra-short (≤350 ms) voice note.

### 2.2 Shoot: short-press in the viewfinder (release within 500 ms)

The frame freezes for about 1.5 s showing `SAVED IMG_NNNN.jpg`. Two files land in `/DCIM`:

- `IMG_NNNN.jpg` — the original, 640×480, JPEG quality 60;
- `IMG_NNNNS.jpg` — a delivery copy, 320×240, quality 50, made from the same frame for chat viewing.

### 2.3 Exit: long-press (800 ms or more) — photos return to the chat

Back in the voice UI, about 3 s later the board posts a text message to the main chat reporting how many photos were taken; the chat then lists them, fetches the delivery copies, posts them, and describes each one. Presses between 500 and 800 ms do nothing (debounce).

### 2.4 Remote capture (without entering camera mode)

From the chat, `camera.capture` returns the full image (`data_base64`) plus the 320×240 copy (`thumb_base64`) in the same result. While camera mode owns the sensor it answers `camera busy`.

## 3. Troubleshooting (by symptom)

### Double-tapped, but camera mode did not engage (the screen says "hold to talk")

The two taps landed outside the 600 ms window and each was taken as a voice tap. Slow down slightly and tap twice in quick succession; a first tap being treated as a short voice note is a known trade-off — just retry.

### In camera mode, but the viewfinder is black or frozen

Long-press to exit, then re-enter. If it keeps happening, open the serial monitor and check the `lichuang_cam` log for DMA allocation errors — that memory failure was fixed by `CONFIG_CAMERA_DMA_BUFFER_SIZE_MAX=16384`; seeing it means the board is not running the current firmware.

### Took photos, but nothing arrived in the chat

Wait 3–10 s after exiting. Check the card with `camera.saved_list` (delivery copies are intentionally not listed). If new photos are listed but the chat stayed silent, that round's text turn was probably dropped — the photos are still on the card; ask the chat to fetch the delivery copies by hand.

### Text in photos reads backwards

The mirror bug of older firmware (cammode8 and earlier): the sensor's hmirror register is a no-op on this board. cammode9 and later flip in software — upgrade. (Photos saved by the standalone camera app v4.2 have the same trait; that app is out of this repository's scope.)

### Remote capture answers "did not respond"

**Gather evidence before unplugging**: ① open the serial monitor and capture the log to see which step it sticks on; ② read the SHT31 temperature/humidity at the same time — if that still answers, the bus is alive and the fault is on the sensor's power side; ③ only then unplug USB, wait 10 s, and replug. Do not lead with the power cycle; it destroys the evidence.

### Commands answer "camera busy"

Camera mode or the preview owns the sensor. Exit camera mode first, then call again.

## 4. Backup and rollback

- Keep every final version as `flashpack-<version>-<date>.tar.gz` with its md5 recorded; a rollback is the same esptool command on the older pack.
- The SD card holds the only copy of the original photos — back up `/DCIM` from the card regularly.

## 5. Updates

Building `main` is always the latest. Version history: [RELEASE_NOTES_ZHH.md](../../RELEASE_NOTES_ZHH.md) (Chinese).

## Appendix: chat-side commands

| Command | Parameters | Result |
| --- | --- | --- |
| `camera.capture` | — | `payload.data_base64` (full JPEG) + `payload.thumb_base64` (320×240); `camera busy` while camera mode is active |
| `camera.saved_list` | — | `payload.files[]` with `name` / `bytes` for the originals in `/DCIM`, ascending; delivery copies are not listed |
| `camera.fetch` | `name` (e.g. `IMG_0041S.jpg`) | `payload.data_base64`; names must be `IMG_*.jpg` basenames, ≤512 KB |
