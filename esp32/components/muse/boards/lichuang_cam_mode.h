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

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Camera mode for the Lichuang SZP (muse_input.c's double-click on the talk
 * key): the gc0308 as a live viewfinder on the LCD, a short press to take a
 * photo to the TF card, a long press to leave. The voice session is suspended
 * for as long as it lasts, and the camera is powered only in here.
 *
 * Everything expensive (frames, task stack) is in PSRAM; enter() only starts
 * the task, so the input task never waits on the sensor.
 */

/* Gives the module the board's sensor power switch (PCA9557 bit 2). Call once
 * from board init, before the first enter(). */
esp_err_t lichuang_cam_mode_prepare(esp_err_t (*power)(bool on));

/* True from enter() until the mode has torn down: the talk key belongs to the
 * camera meanwhile, so the input layer sends every edge to cam_mode_key(). */
bool lichuang_cam_mode_active(void);

/* Starts the mode (suspends voice, powers the sensor, shows the viewfinder).
 * False if the task couldn't be created: nothing changed. */
bool lichuang_cam_mode_enter(void);

/* Asks for one photo. Ignored while one is being taken or the view isn't up
 * yet. */
void lichuang_cam_mode_shutter(void);

/* Asks to leave: powers the sensor down, frees everything, resumes voice. */
void lichuang_cam_mode_leave(void);

/* Talk key edges while active: a press under the shutter limit takes a photo
 * on release, a hold past the exit limit leaves. */
void lichuang_cam_mode_key(bool pressed);

/* Polled by the input task while active, so a hold leaves without waiting for
 * the release. */
void lichuang_cam_mode_tick(void);

#ifdef __cplusplus
}
#endif
