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
 * The board's TF slot (Lichuang SZP: 1-bit SD on GPIO 47/48/21), mounted at
 * /sdcard. Two things write there now -- the SHT31 CSV log and the camera's
 * photos -- so the mount lives here instead of in either of them: whoever
 * needs the card first mounts it, and the other reuses that mount.
 *
 * Muse_sd never unmounts on its own; a card that fails mid-write is reported
 * by the caller, and the next caller's mount attempt (or the SHT31 log's
 * retry) picks it up again.
 */

#define MUSE_SD_MOUNT_POINT "/sdcard"

/* Mounts the card if it isn't mounted yet. ESP_OK when it is usable (by this
 * call or an earlier one), anything else when there's no card. Slow on the
 * first call and on a missing card: it runs the SD init timeouts. */
esp_err_t muse_sd_mount(void);

/* True once a mount has succeeded and nothing has unmounted it since. */
bool muse_sd_mounted(void);

/* Unmounts, if mounted. Later writers must call muse_sd_mount() again. */
void muse_sd_unmount(void);

#ifdef __cplusplus
}
#endif
