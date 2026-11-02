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

#include "muse_sd.h"

#include "driver/sdmmc_host.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

static const char *TAG = "muse_sd";

#define SD_CLK_GPIO GPIO_NUM_47
#define SD_CMD_GPIO GPIO_NUM_48
#define SD_D0_GPIO  GPIO_NUM_21

static sdmmc_card_t *s_card;
static bool s_mounted;

esp_err_t muse_sd_mount(void)
{
    if (s_mounted) {
        return ESP_OK;
    }
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot = SDMMC_HOST_SLOT_1;
    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.width = 1;
    slot.clk = SD_CLK_GPIO;
    slot.cmd = SD_CMD_GPIO;
    slot.d0 = SD_D0_GPIO;
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files = 4,   /* the log and a photo at once, with room to spare */
        .allocation_unit_size = 16 * 1024,
    };
    sdmmc_card_t *card = NULL;
    esp_err_t err = esp_vfs_fat_sdmmc_mount(MUSE_SD_MOUNT_POINT, &host, &slot,
                                            &mount_cfg, &card);
    if (err == ESP_ERR_INVALID_STATE) {
        /* Another module mounted it first: the path is ours to use. */
        err = ESP_OK;
        card = NULL;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "no card: %s", esp_err_to_name(err));
        return err;
    }
    s_card = card;
    s_mounted = true;
    ESP_LOGI(TAG, "TF card mounted at %s", MUSE_SD_MOUNT_POINT);
    return ESP_OK;
}

bool muse_sd_mounted(void)
{
    return s_mounted;
}

void muse_sd_unmount(void)
{
    if (!s_mounted) {
        return;
    }
    if (s_card) {
        esp_vfs_fat_sdcard_unmount(MUSE_SD_MOUNT_POINT, s_card);
    }
    s_card = NULL;
    s_mounted = false;
}
