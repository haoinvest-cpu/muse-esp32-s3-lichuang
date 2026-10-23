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

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

/*
 * SHT31 temperature/humidity sensor on the board's I2C bus, wired out to the
 * expansion header (J1 on the Lichuang SZP). Probed at the default address
 * 0x44, falling back to 0x45. A sampler task takes a single-shot reading
 * every 30 s and keeps the latest good one here for the UI; a board without
 * the sensor just never gets a reading, and nothing else changes.
 */

/* Starts the sampler on an already-created bus. ESP_ERR_NOT_FOUND when no
 * sensor answers; the caller treats any error as "no sensor" and boots on. */
esp_err_t muse_sht31_start(i2c_master_bus_handle_t bus);

/* True when a sensor answered at start. Stays true even if later samples
 * fail (a sensor unplugged mid-run shows up in muse_sht31_get instead). */
bool muse_sht31_present(void);

/* Copies the latest valid reading into *temp_c (degrees C) and *rh (percent,
 * 0-100) and returns true. False until the first good sample, and again after
 * repeated failures, when the cached reading is too stale to show. Either
 * pointer may be NULL. */
bool muse_sht31_get(float *temp_c, float *rh);

/* muse_sht31_get, plus the reading's age in seconds in *age_s (time since
 * the sample was taken). For the Link sensors.read command, which reports
 * each reading's age. Any pointer may be NULL. */
bool muse_sht31_get_ex(float *temp_c, float *rh, uint32_t *age_s);

/*
 * History: the sampler also folds every 10 samples (5 minutes) into one
 * averaged point and keeps a day of them in a ring. Points come out oldest
 * first. Each point carries a timestamp: *ts_is_wall says which kind --
 * true: seconds since the Unix epoch (a wall clock was available), false:
 * seconds since boot (this firmware has no clock source of its own, so
 * boot seconds are the normal case until something sets the time).
 */
#define MUSE_SHT31_HISTORY_POINTS 288

/* Number of history points currently held, 0..MUSE_SHT31_HISTORY_POINTS. */
int muse_sht31_history_count(void);

/* Point idx (0 = oldest). False when idx is out of range. Any output
 * pointer may be NULL. */
bool muse_sht31_history_point(int idx, float *temp_c, float *rh,
                              int64_t *ts, bool *ts_is_wall);

/* Temperature extremes over the points held (up to the last 24 h), with
 * the timestamp of each extreme. *ts_is_wall is true only when both
 * extremes carry wall-clock stamps. False when the ring is empty. Any
 * output pointer may be NULL. */
bool muse_sht31_history_extremes(float *min_c, float *max_c,
                                 int64_t *min_ts, int64_t *max_ts,
                                 bool *ts_is_wall);

/* Replace an empty ring with points restored from off-board storage
 * (the Pi hub's durable log, via muse_weather): arrays of n points,
 * oldest first, all stamped with wall-clock epoch seconds. Returns how
 * many points were taken in. A ring that already holds live points is
 * left untouched and 0 is returned: restore is a boot-time repair, not
 * a merge. */
int muse_sht31_history_import(const int64_t *ts, const float *temps,
                              const float *rhs, int n);

/*
 * Speak one line of text aloud through the Pi's local TTS service
 * (edge-tts on the hotspot gateway; see muse_sht31.c). Never blocks
 * the caller: the fetch + decode + playback runs on its own task.
 * One utterance at a time -- returns false when one is already
 * playing (the line is dropped, not queued) or on bad input.
 */
bool muse_sht31_speak(const char *text);
