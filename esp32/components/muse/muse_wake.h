/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/*
 * Voice wake (Lichuang SZP only): hands-free push-to-talk. While the
 * "Voice wake" setting is on, the voice pipeline's idle audio also runs
 * through Espressif's AFE with WakeNet (esp-sr, models in the "model"
 * flash partition); a hit posts the same PTT events the talk button
 * posts, so the rest of the voice pipeline cannot tell the difference.
 * Audio heard before a hit never leaves the board.
 *
 * There is no "release" in hands-free mode, so the wake task ends the
 * turn from the audio energy instead (the AFE's own VAD is not built:
 * it costs internal RAM this board does not have): once speech has
 * been heard, silence longer than MUSE_WAKE_SILENCE_MS posts the
 * PTT_UP. MUSE_WAKE_TURN_MAX_MS caps a turn that never quiets down
 * (background noise), and MUSE_WAKE_NOSPEECH_MS ends a turn where
 * nothing was said at all.
 * Off by default (muse_settings "wake_on"); while off, nothing here
 * runs and the microphone is only the voice pipeline's.
 */

/* Endpointing tunables (milliseconds). */
#define MUSE_WAKE_SILENCE_MS 1200    /* pause after speech that ends the turn */
#define MUSE_WAKE_TURN_MAX_MS 30000  /* hard cap for one wake-started turn */
#define MUSE_WAKE_NOSPEECH_MS 4000   /* give up if no speech follows the wake word */

/* Starts the (dormant) wake task. The AFE is built lazily the first
 * time the setting is switched on, so a board that never enables it
 * pays only the parked task. ESP_ERR_NOT_SUPPORTED on boards without
 * the wake build (everything but the Lichuang SZP). */
esp_err_t muse_wake_start(void);

/* Audio taps, called from the voice task only (single producer).
 * feed_idle takes the chunks muse_voice keeps as pre-roll while it
 * waits for a press; feed_record takes the chunks of the note being
 * recorded. Each is dropped unless the wake state machine is in the
 * matching phase, so a button-started turn is never wake-processed.
 * Cheap no-ops while the setting is off. */
void muse_wake_feed_idle(const int16_t *pcm, size_t frames);
void muse_wake_feed_record(const int16_t *pcm, size_t frames);
