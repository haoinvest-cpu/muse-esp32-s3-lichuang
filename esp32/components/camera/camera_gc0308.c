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

#include "camera_gc0308.h"

#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_check.h"
#include "esp_log.h"
#include "img_converters.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
 * gc0308 on the Lichuang SZP ESP32-S3, via espressif/esp32-camera.
 *
 * Power: the sensor's PWDN is PCA9557 bit 2 (active high). The board's
 * power() callback drives it. The driver powers the sensor only while a
 * capture or stream is active.
 *
 * I2C: SCCB shares IO1/IO2 with the board's I2C_NUM_0 bus. esp32-camera
 * uses the legacy I2C driver, which can't share a port with the board's
 * new-driver bus, so SCCB runs on I2C_NUM_1 on the same pins.
 *
 * LEDC: XCLK must not use TIMER_0/CHANNEL_0 (the LCD backlight). Uses
 * TIMER_1/CHANNEL_2.
 */

static const char *TAG = "camera.gc0308";

/* Lichuang SZP ESP32-S3 gc0308 wiring (from wiki.lckfb.com). */
#define GC0308_PIN_XCLK  5
#define GC0308_PIN_PCLK  7
#define GC0308_PIN_VSYNC 3
#define GC0308_PIN_HREF  46
#define GC0308_PIN_SDA   1
#define GC0308_PIN_SCL   2
#define GC0308_PIN_D0    16
#define GC0308_PIN_D1    18
#define GC0308_PIN_D2    8
#define GC0308_PIN_D3    17
#define GC0308_PIN_D4    15
#define GC0308_PIN_D5    6
#define GC0308_PIN_D6    4
#define GC0308_PIN_D7    9

#define GC0308_XCLK_FREQ_HZ 20000000
#define GC0308_SCCB_I2C_PORT I2C_NUM_1
#define GC0308_LEDC_TIMER LEDC_TIMER_1
#define GC0308_LEDC_CHANNEL LEDC_CHANNEL_2

/* The sensor has no JPEG encoder: stills are captured as YUV422 and encoded
 * to JPEG in software. Quality for the software encoder, 1-100 (higher is
 * better); 80 matches esp32-camera's own examples. */
#define GC0308_JPEG_QUALITY 50

typedef struct {
    camera_gc0308_config_t cfg;
    bool powered;               /* sensor PWDN released */
    bool driver_init;           /* esp_camera_init() succeeded */
    atomic_bool streaming;
    TaskHandle_t stream_task;
    camera_frame_cb_t on_frame;
    void *stream_ctx;
} gc0308_state_t;

static gc0308_state_t s_state;
static const camera_driver_t s_driver;

static esp_err_t gc0308_power(bool on)
{
    if (s_state.cfg.power) {
        esp_err_t err = s_state.cfg.power(on);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "power(%d) failed: %s", on, esp_err_to_name(err));
            return err;
        }
    }
    s_state.powered = on;
    if (on) {
        /* Let the sensor's internal regulator stabilize. */
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return ESP_OK;
}

static esp_err_t gc0308_driver_init(void)
{
    if (s_state.driver_init) {
        return ESP_OK;
    }
    camera_config_t config = {
        .pin_pwdn = -1,  /* PCA9557, via power() */
        .pin_reset = -1, /* PCA9557; power-up resets the sensor */
        .pin_xclk = GC0308_PIN_XCLK,
        .pin_sccb_sda = GC0308_PIN_SDA,
        .pin_sccb_scl = GC0308_PIN_SCL,
        .pin_d7 = GC0308_PIN_D7,
        .pin_d6 = GC0308_PIN_D6,
        .pin_d5 = GC0308_PIN_D5,
        .pin_d4 = GC0308_PIN_D4,
        .pin_d3 = GC0308_PIN_D3,
        .pin_d2 = GC0308_PIN_D2,
        .pin_d1 = GC0308_PIN_D1,
        .pin_d0 = GC0308_PIN_D0,
        .pin_vsync = GC0308_PIN_VSYNC,
        .pin_href = GC0308_PIN_HREF,
        .pin_pclk = GC0308_PIN_PCLK,
        .xclk_freq_hz = GC0308_XCLK_FREQ_HZ,
        .ledc_timer = GC0308_LEDC_TIMER,
        .ledc_channel = GC0308_LEDC_CHANNEL,
        /* gc0308 can't emit JPEG: capture YUV422, convert in software. */
        .pixel_format = PIXFORMAT_YUV422,
        .frame_size = FRAMESIZE_VGA,  /* 640x480, gc0308 max */
        .jpeg_quality = 12,          /* unused without PIXFORMAT_JPEG */
        .fb_count = 2,               /* double-buffer for streaming */
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
        .sccb_i2c_port = GC0308_SCCB_I2C_PORT,
    };
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init failed: %s", esp_err_to_name(err));
        return err;
    }
    /* Tweak the sensor for indoor light; defaults are fine otherwise. */
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_brightness(s, 0);
        s->set_contrast(s, 0);
        s->set_saturation(s, 0);
    }
    s_state.driver_init = true;
    /* Discard the first frames while AEC/AGC settles. */
    for (int i = 0; i < 2; i++) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return ESP_OK;
}

static void gc0308_driver_deinit(void)
{
    if (!s_state.driver_init) {
        return;
    }
    esp_camera_deinit();
    s_state.driver_init = false;
}

static esp_err_t gc0308_init(void)
{
    /* Nothing to do until the first capture/stream; power() is called then.
     * Validates the config now so a missing power callback fails early. */
    return ESP_OK;
}

static esp_err_t gc0308_capture(camera_frame_t *out)
{
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));

    ESP_RETURN_ON_ERROR(gc0308_power(true), TAG, "power on");
    esp_err_t err = gc0308_driver_init();
    if (err != ESP_OK) {
        gc0308_power(false);
        return err;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "esp_camera_fb_get failed");
        gc0308_driver_deinit();
        gc0308_power(false);
        return ESP_FAIL;
    }
    /* The sensor has no JPEG encoder: convert the YUV422 frame in software. */
    uint8_t *jpg = NULL;
    size_t jpg_len = 0;
    int w = fb->width, h = fb->height;
    bool converted = frame2jpg(fb, GC0308_JPEG_QUALITY, &jpg, &jpg_len);
    esp_camera_fb_return(fb);
    if (!converted || !jpg) {
        ESP_LOGE(TAG, "frame2jpg failed");
        gc0308_driver_deinit();
        gc0308_power(false);
        return ESP_FAIL;
    }

    /* The JPEG stays out until release(): keep driver+power alive. */
    out->jpeg = jpg;
    out->len = jpg_len;
    out->width = w;
    out->height = h;
    out->priv = jpg;  /* freed in release() */
    ESP_LOGI(TAG, "captured %dx%d YUV422 -> JPEG, %u bytes", w, h, (unsigned)jpg_len);
    return ESP_OK;
}

static void gc0308_release(camera_frame_t *frame)
{
    if (!frame || !frame->priv) {
        return;
    }
    free(frame->priv);  /* the software-encoded JPEG buffer */
    frame->jpeg = NULL;
    frame->priv = NULL;
    /* Power down now that the frame is back. */
    gc0308_driver_deinit();
    gc0308_power(false);
}

static void gc0308_stream_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "stream task started");
    while (atomic_load(&s_state.streaming)) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        if (fb->format == PIXFORMAT_YUV422 && atomic_load(&s_state.streaming)) {
            uint8_t *jpg = NULL;
            size_t jpg_len = 0;
            if (frame2jpg(fb, GC0308_JPEG_QUALITY, &jpg, &jpg_len) && jpg) {
                camera_frame_t frame = {
                    .jpeg = jpg,
                    .len = jpg_len,
                    .width = fb->width,
                    .height = fb->height,
                    .priv = NULL,  /* freed when callback returns */
                };
                s_state.on_frame(&frame, s_state.stream_ctx);
                free(jpg);
            }
        }
        esp_camera_fb_return(fb);
    }
    ESP_LOGI(TAG, "stream task stopping");
    s_state.stream_task = NULL;
    vTaskDelete(NULL);
}

static esp_err_t gc0308_stream_start(camera_frame_cb_t on_frame, void *ctx)
{
    if (!on_frame) {
        return ESP_ERR_INVALID_ARG;
    }
    if (atomic_exchange(&s_state.streaming, true)) {
        return ESP_ERR_INVALID_STATE;  /* already streaming */
    }

    ESP_RETURN_ON_ERROR(gc0308_power(true), TAG, "power on");
    esp_err_t err = gc0308_driver_init();
    if (err != ESP_OK) {
        atomic_store(&s_state.streaming, false);
        gc0308_power(false);
        return err;
    }

    /* Smaller frames for the preview, to keep up. */
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_framesize(s, FRAMESIZE_QVGA);  /* 320x240 */
    }

    s_state.on_frame = on_frame;
    s_state.stream_ctx = ctx;
    BaseType_t ret = xTaskCreate(gc0308_stream_task, "gc0308_stream", 4096,
                                 NULL, 5, &s_state.stream_task);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "failed to start stream task");
        atomic_store(&s_state.streaming, false);
        gc0308_driver_deinit();
        gc0308_power(false);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static void gc0308_stream_stop(void)
{
    if (!atomic_exchange(&s_state.streaming, false)) {
        return;  /* not streaming */
    }
    /* Wait for the task to exit (it checks the flag each frame). */
    for (int i = 0; i < 50 && s_state.stream_task; i++) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (s_state.stream_task) {
        ESP_LOGW(TAG, "stream task did not exit, deleting");
        vTaskDelete(s_state.stream_task);
        s_state.stream_task = NULL;
    }
    /* Restore full resolution for stills. */
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_framesize(s, FRAMESIZE_VGA);
    }
    gc0308_driver_deinit();
    gc0308_power(false);
}

static const camera_driver_t s_driver = {
    .name = "gc0308 (Lichuang SZP ESP32-S3)",
    .max_width = 640,
    .max_height = 480,
    .init = gc0308_init,
    .capture = gc0308_capture,
    .release = gc0308_release,
    .stream_start = gc0308_stream_start,
    .stream_stop = gc0308_stream_stop,
};

const camera_driver_t *camera_gc0308(const camera_gc0308_config_t *config)
{
    if (!config) {
        return NULL;
    }
    memset(&s_state, 0, sizeof(s_state));
    s_state.cfg = *config;
    return &s_driver;
}
