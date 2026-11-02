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
 * A gc0308 camera (640x480, JPEG) on the ESP32-S3's DVP peripheral, driven
 * by espressif/esp32-camera. The sensor is powered only for a capture or a
 * stream: power on, init the driver, capture/stream, deinit, power off.
 * Between them nothing runs: no task, no polling.
 *
 * Wiring (Lichuang SZP ESP32-S3):
 *   XCLK=IO5, PCLK=IO7, VSYNC=IO3, HREF=IO46,
 *   SDA=IO1, SCL=IO2 (SCCB, shared with the board's I2C bus),
 *   D0=IO16, D1=IO18, D2=IO8, D3=IO17, D4=IO15, D5=IO6, D6=IO4, D7=IO9.
 * PWDN and RESET are on the PCA9557 I/O expander (0x19), not GPIOs: the
 * board supplies power() to drive them. SCCB uses I2C_NUM_1 (the board
 * owns I2C_NUM_0); both share the IO1/IO2 pins.
 */

typedef struct {
    const char *name;               /* camera_driver_t.name */
    esp_err_t (*power)(bool on);    /* board's PCA9557 PWDN control; NULL: always on */
} camera_gc0308_config_t;

/* The backend for `config` (copied), to pass to camera_register(). One per device. */
const camera_driver_t *camera_gc0308(const camera_gc0308_config_t *config);

#ifdef __cplusplus
}
#endif
