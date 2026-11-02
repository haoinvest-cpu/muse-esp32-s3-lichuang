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
#include "cJSON.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Toggle the live preview; call from a UI gesture or button. */
void lichuang_camera_preview_toggle(void);
bool lichuang_camera_preview_active(void);
/* Captures one frame for the Home Link camera.capture command. */
bool lichuang_camera_capture(char **jpeg_base64, char **thumb_base64, const char **error);
esp_err_t lichuang_camera_prepare(void);

/*
 * The photos on the SD card, for the chat: camera.saved_list and camera.fetch.
 * Both refuse while camera mode owns the sensor ("camera busy"), and neither
 * changes a file on the card.
 */

/* The result of camera.saved_list: {"ok":true,"payload":{"count":N,"files":
 * [{"name":"IMG_0001.jpg","bytes":12345},...]}} in name order, at most 50;
 * an empty list when there is no card or no /DCIM. "camera busy" comes back
 * as an error result. NULL only when out of memory. */
cJSON *lichuang_camera_saved_list(void);

/* camera.fetch: the named photo as base64 JPEG in *jpeg_base64 (PSRAM, caller
 * frees), the same payload shape as camera.capture. False with *error set for
 * a name outside /DCIM's IMG_*.jpg set, a missing file, or one over 512 KB. */
bool lichuang_camera_fetch(const char *name, char **jpeg_base64, const char **error);

#ifdef __cplusplus
}
#endif
