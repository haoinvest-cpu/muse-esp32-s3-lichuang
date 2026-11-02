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

/*
 * GC0308 on the Lichuang SZP's DVP header, through esp32-camera. The sensor
 * has no JPEG encoder, so each capture takes YUV422 and runs frame2jpg()
 * (jpge, software) on it. VGA (the sensor's full frame, double-buffered)
 * mirrors the board's original gc0308 backend, whose capture config was
 * proven on this tree; the whole driver is torn down after every frame so
 * the voice pipeline's internal RAM is untouched while nobody is taking
 * photos.
 *
 * Pin table: the vendor 07-lcd_camera example, verified on this board. It
 * shares I2C0 (GPIO 1/2) with the expander, touch and codecs, so SCCB uses
 * the bus that board_lichuang_szp.c already created (pin_sccb_sda = -1).
 */

#include "camera_dvp.h"

#include <stdlib.h>
#include <string.h>

#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "img_converters.h"
#include "soc/gpio_num.h"

/* True horizontal mirror for a YUV422 frame, in place: reverse the pixel
 * pairs in each row and swap the two luma bytes inside each pair (chroma
 * stays with its pair). The sensor's hmirror register op proved a no-op on
 * this board -- cammode7/8 set it to 1 and photos still came out mirrored
 * (IMG_0039, reversed text) -- so the flip is done here, in software. */
static void flip_yuv422_h(uint8_t *buf, int w, int h)
{
    const int pairs = w / 2;
    for (int y = 0; y < h; y++) {
        uint8_t *row = buf + (size_t)y * (size_t)w * 2;
        for (int j = 0; j < pairs / 2; j++) {
            uint8_t *a = row + (size_t)j * 4;
            uint8_t *b = row + (size_t)(pairs - 1 - j) * 4;
            const uint8_t t0 = b[2], t1 = b[1], t2 = b[0], t3 = b[3];
            b[0] = a[2]; b[1] = a[1]; b[2] = a[0]; b[3] = a[3];
            a[0] = t0;   a[1] = t1;   a[2] = t2;   a[3] = t3;
        }
        if (pairs % 2) {
            uint8_t *m = row + (size_t)(pairs / 2) * 4;
            const uint8_t t = m[0]; m[0] = m[2]; m[2] = t;
        }
    }
}


static const char *TAG = "camera.dvp";

#define CAM_PIN_XCLK GPIO_NUM_5
#define CAM_PIN_D7 GPIO_NUM_9
#define CAM_PIN_D6 GPIO_NUM_4
#define CAM_PIN_D5 GPIO_NUM_6
#define CAM_PIN_D4 GPIO_NUM_15
#define CAM_PIN_D3 GPIO_NUM_17
#define CAM_PIN_D2 GPIO_NUM_8
#define CAM_PIN_D1 GPIO_NUM_18
#define CAM_PIN_D0 GPIO_NUM_16
#define CAM_PIN_VSYNC GPIO_NUM_3
#define CAM_PIN_HREF GPIO_NUM_46
#define CAM_PIN_PCLK GPIO_NUM_7

#define CAM_XCLK_HZ 20000000    /* GC0308's documented stable maximum */
#define CAM_JPEG_QUALITY 12     /* jpge: 0-63, lower is better */
#define CAM_POWER_SETTLE_MS 100 /* 2.8 V rail and sensor POR */
#define CAM_I2C_PORT 0          /* the board's bus: expander, touch, codecs */

static const camera_dvp_config_t *s_config;
static size_t s_heap_before; /* internal heap when this capture began */

static void power_off(void)
{
    if (s_config && s_config->power) {
        esp_err_t err = s_config->power(false);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "camera power-down failed: %s", esp_err_to_name(err));
        }
    }
}

/* Powers the sensor and starts esp32-camera; the caller tears both down. */
static esp_err_t camera_start(void)
{
    if (!s_config || !s_config->power) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = s_config->power(true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "camera power-up failed: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(CAM_POWER_SETTLE_MS));

    const camera_config_t config = {
        .pin_pwdn = -1,      /* the expander's bit 2, not a GPIO */
        .pin_reset = -1,
        .pin_xclk = CAM_PIN_XCLK,
        .pin_sccb_sda = -1,  /* reuse the board's I2C0 bus */
        .pin_sccb_scl = -1,
        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href = CAM_PIN_HREF,
        .pin_pclk = CAM_PIN_PCLK,
        .xclk_freq_hz = CAM_XCLK_HZ,
        /* ESP32-S3 makes XCLK in LCD_CAM; these stay unused there. */
        .ledc_timer = LEDC_TIMER_1,
        .ledc_channel = LEDC_CHANNEL_2, /* CHANNEL_1 belongs to the status LED */
        .pixel_format = PIXFORMAT_YUV422,
        .frame_size = FRAMESIZE_VGA,
        .jpeg_quality = CAM_JPEG_QUALITY, /* meaningless for YUV422 input */
        .fb_count = 2,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
        .sccb_i2c_port = CAM_I2C_PORT,
    };
    err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init failed: %s", esp_err_to_name(err));
        power_off();
        /* "did not respond" is what callers show for a silent sensor. */
        if (err == ESP_ERR_NOT_FOUND || err == ESP_ERR_CAMERA_NOT_DETECTED) {
            return ESP_ERR_TIMEOUT;
        }
        return err;
    }
    /* Kept from the cammode7 attempt; the op proved a no-op on this board.
     * The real mirror fix is flip_yuv422_h() at the capture site below. */
    sensor_t *hm_sensor = esp_camera_sensor_get();
    if (hm_sensor) {
        hm_sensor->set_hmirror(hm_sensor, 1);
    }
    /* Discard the first frames while AEC/AGC settles, as the board's
     * original gc0308 backend did; the caller grabs the next one. */
    int settle_frames = 0;
    for (int i = 0; i < 2; i++) {
        camera_fb_t *settle_fb = esp_camera_fb_get();
        if (settle_fb) {
            settle_frames++;
            esp_camera_fb_return(settle_fb);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (settle_frames == 0) {
        ESP_LOGE(TAG, "no frames during settle; init looks bad");
        esp_camera_deinit();
        power_off();
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

/* Every other pixel pair of every other row: 640x480 YUV422 becomes
 * 320x240 YUV422, pairs copied whole so chroma stays with its luma. */
static void thumb_downsample(const uint8_t *src, uint8_t *dst)
{
    for (int y = 0; y < 240; y++) {
        const uint8_t *srow = src + (size_t)y * 2 * 640 * 2;
        uint8_t *drow = dst + (size_t)y * 320 * 2;
        for (int x = 0; x < 160; x++) {
            memcpy(drow + (size_t)x * 4, srow + (size_t)x * 8, 4);
        }
    }
}

static esp_err_t dvp_capture_once(camera_frame_t *out)
{
    s_heap_before = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    const int64_t started_us = esp_timer_get_time();
    esp_err_t err = camera_start();
    if (err != ESP_OK) {
        return err;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "no frame from the sensor");
        esp_camera_deinit();
        power_off();
        return ESP_ERR_TIMEOUT;
    }

    flip_yuv422_h(fb->buf, fb->width, fb->height);
    /* A 320x240 delivery copy of the same flipped frame: the full VGA
     * base64 is too large for a chat to restore reliably, the small one
     * is not. Best effort -- any failure just leaves the capture
     * without a thumbnail, it never fails the capture. */
    uint8_t *thumb = NULL;
    size_t thumb_len = 0;
    if (fb->width == 640 && fb->height == 480) {
        uint8_t *small = heap_caps_malloc(320 * 240 * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (small) {
            thumb_downsample(fb->buf, small);
            camera_fb_t tfb = { .buf = small, .len = 320 * 240 * 2,
                                .width = 320, .height = 240, .format = PIXFORMAT_YUV422 };
            if (!frame2jpg(&tfb, 50, &thumb, &thumb_len) || !thumb || thumb_len < 4 ||
                thumb[0] != 0xFF || thumb[1] != 0xD8 ||
                thumb[thumb_len - 2] != 0xFF || thumb[thumb_len - 1] != 0xD9) {
                free(thumb);
                thumb = NULL;
                thumb_len = 0;
            }
            free(small);
        }
    }
    uint8_t *jpeg = NULL;
    size_t jpeg_len = 0;
    const int width = fb->width;
    const int height = fb->height;
    bool ok = frame2jpg(fb, CAM_JPEG_QUALITY, &jpeg, &jpeg_len);
    esp_camera_fb_return(fb);
    esp_camera_deinit();
    power_off();

    if (!ok || !jpeg || jpeg_len < 4) {
        ESP_LOGE(TAG, "JPEG encoding failed");
        free(jpeg);
        free(thumb);
        return ESP_ERR_NO_MEM;
    }
    /* Never hand a half-written frame to the command layer. */
    if (jpeg[0] != 0xFF || jpeg[1] != 0xD8 || jpeg[jpeg_len - 2] != 0xFF ||
        jpeg[jpeg_len - 1] != 0xD9) {
        ESP_LOGE(TAG, "malformed JPEG from encoder (%zu bytes)", jpeg_len);
        free(jpeg);
        return ESP_FAIL;
    }

    out->jpeg = jpeg;
    out->len = jpeg_len;
    out->thumb = thumb;
    out->thumb_len = thumb_len;
    out->width = width;
    out->height = height;
    out->priv = NULL;
    ESP_LOGI(TAG, "capture %dx%d q%d: %zu bytes in %lld ms", width, height, CAM_JPEG_QUALITY,
             jpeg_len, (esp_timer_get_time() - started_us) / 1000);
    return ESP_OK;
}

static esp_err_t dvp_capture(camera_frame_t *out)
{
    esp_err_t err = dvp_capture_once(out);
    if (err == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "capture timed out; retrying with a fresh power cycle");
        vTaskDelay(pdMS_TO_TICKS(150));
        err = dvp_capture_once(out);
    }
    return err;
}

static void dvp_release(camera_frame_t *frame)
{
    free((void *)frame->thumb);
    frame->thumb = NULL;
    frame->thumb_len = 0;
    free((void *)frame->jpeg);
    frame->jpeg = NULL;
    frame->len = 0;
    const size_t now = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    ESP_LOGI(TAG, "internal heap %u -> %u (%+d bytes)", (unsigned)s_heap_before, (unsigned)now,
             (int)now - (int)s_heap_before);
}

static const camera_driver_t s_driver = {
    .name = "GC0308 (Lichuang SZP DVP)",
    .max_width = 640,
    .max_height = 480,
    /* capture() powers, initializes and tears down per frame. */
    .init = NULL,
    .capture = dvp_capture,
    .release = dvp_release,
    .stream_start = NULL,
    .stream_stop = NULL,
};

const camera_driver_t *camera_dvp(const camera_dvp_config_t *config)
{
    s_config = config;
    return &s_driver;
}
