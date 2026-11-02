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

#include "camera.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * A DVP sensor (gc0308 on the Lichuang SZP) behind esp32-camera. One frame at
 * a time: each capture powers the sensor, initializes the driver, grabs one
 * frame, encodes it to JPEG in software, and tears everything down again, so
 * no camera RAM or task stays resident between photos.
 *
 * `power` switches the sensor's rail; the board owns it (PCA9557 bit 2 here).
 */
typedef esp_err_t (*camera_dvp_power_fn)(bool on);

typedef struct {
    camera_dvp_power_fn power;
} camera_dvp_config_t;

/* A still-only driver: no preview stream. */
const camera_driver_t *camera_dvp(const camera_dvp_config_t *config);

#ifdef __cplusplus
}
#endif
