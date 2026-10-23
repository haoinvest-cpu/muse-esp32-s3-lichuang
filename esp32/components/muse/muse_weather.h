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

#include "esp_err.h"

/*
 * Weather from the Pi hub (sht31-hub, see the Lichuang SZP port notes).
 * The board itself never talks to the internet for this: the Pi fetches
 * the forecast, formats the Chinese strings and holds the readings log.
 * The board polls the hub over the hotspot it already joins:
 *
 *   GET  /weather   forecast + the hub's wall-clock epoch (the board has
 *                   no clock source; a successful fetch sets its clock,
 *                   which also gives the SHT31 SD log real timestamps)
 *   POST /reading   each new 5-minute SHT31 point, so the Pi's CSV --
 *                   not the board's RAM -- is the durable record. (The
 *                   hub also serves it back at GET /history; the board
 *                   itself no longer displays a chart, so it does not
 *                   fetch it -- the log is for the record, and for
 *                   whatever wants the data later.)
 *
 * Only the Lichuang SZP build starts this (board_lichuang_szp.c): the
 * hub address is that home's Pi. Every other board simply never calls
 * muse_weather_start(), muse_weather_get() stays false and the UI
 * leaves the weather tile out.
 */

typedef struct {
    char name[12];   /* 今天 / 明天 / 周六 (hub-formatted) */
    char cond[16];   /* 多云 (hub-formatted) */
    int lo, hi;      /* degrees C */
    int precip;      /* percent, -1 when the source gives none */
} muse_weather_day_t;

typedef struct {
    bool valid;
    uint32_t version;   /* bumps on every successful fetch */
    int64_t epoch;      /* hub wall time when the fetch landed */
    char cond[16];      /* current condition, hub-formatted Chinese */
    float temp;         /* current outdoor temperature, degrees C */
    int hi, lo;         /* today's range, degrees C */
    int precip;         /* today's precipitation probability, -1 = none */
    muse_weather_day_t days[3];
    char advice[96];    /* one-line outing advice, hub-formatted */
    char speak[160];    /* the outdoor half of the spoken summary */
} muse_weather_t;

/* Starts the hub task. ESP_OK when already running. */
esp_err_t muse_weather_start(void);

/* True once muse_weather_start() has been called: the UI builds the
 * weather tile only then, so other boards' tiles are unchanged. */
bool muse_weather_enabled(void);

/* Copies the latest fetched weather into *out and returns true. False
 * until the first successful fetch (and forever when never started). */
bool muse_weather_get(muse_weather_t *out);
