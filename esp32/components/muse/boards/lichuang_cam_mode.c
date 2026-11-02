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

#include "lichuang_cam_mode.h"

#include <dirent.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "img_converters.h"
#include "soc/gpio_num.h"

#include "muse_board.h"
#include "muse_chat.h"
#include "muse_mem.h"
#include "muse_sd.h"
#include "muse_state.h"
#include "muse_ui.h"
#include "muse_voice.h"

/*
 * Live viewfinder + stills on the Lichuang SZP's gc0308, through
 * esp32-camera. The sensor is single-purposed by design (components/camera's
 * catalog backend powers it for one frame and tears it down again), so this
 * mode owns it directly instead of going through camera_stream_start().
 *
 *   preview: RGB565 / QVGA, two PSRAM frame buffers, GRAB_LATEST
 *   photo:   YUV422 / VGA, one frame copied to PSRAM, encoded by jpge
 *
 * Switching between the two means re-initializing the driver: the sensor's
 * output format and size are set at init, and the gc0308 has no runtime
 * switch that esp32-camera exposes. Each switch costs a power settle plus
 * the driver's setup, which is why preview frames are what runs between
 * photos and nothing runs at all outside the mode.
 *
 * Pins are the board's verified DVP table (components/camera/camera_dvp.c);
 * SCCB rides the board's I2C0 (expander, touch, codecs) rather than a bus of
 * its own. The sensor's PWDN is PCA9557 bit 2, which the board supplies.
 */

static const char *TAG = "lichuang.cam";

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

#define CAM_XCLK_HZ 20000000        /* gc0308's documented stable maximum */
#define CAM_I2C_PORT 0              /* the board's bus */
#define POWER_SETTLE_MS 100         /* 2.8 V rail and sensor POR */
#define SETTLE_FRAMES 2             /* AEC/AGC needs a couple of frames */
#define JPEG_QUALITY 60             /* jpge: 1-100, higher is better. 60 keeps a
                                     * VGA frame well under frame2jpg's fixed
                                     * 128 KB output; the camera app's 88 can
                                     * overflow it on a busy scene. */

#define VIEW_W 320                  /* the panel, and QVGA */
#define VIEW_H 240
#define VIEW_FRAME_MS 66            /* ~15 fps: the SPI panel sets the pace */
#define SHOT_W 640                  /* the sensor's full frame */
#define SHOT_H 480
#define SHOT_BYTES ((size_t)SHOT_W * SHOT_H * 2)   /* YUV422 */
#define THUMB_W (SHOT_W / 2)        /* the delivery copy: the chat can take this */
#define THUMB_H (SHOT_H / 2)
#define THUMB_BYTES ((size_t)THUMB_W * THUMB_H * 2)   /* YUV422, 153600 B */
#define THUMB_QUALITY 50            /* half of the photo's 60: ~5-7 KB */

#define FREEZE_MS 1500              /* the photo stays up as confirmation */
#define ENTRY_HINT_MS 2500          /* how long the key hint stays up */
#define PREVIEW_TRIES 3             /* sensor re-init attempts before giving up */
#define MAX_SHOTS 9999

/* After leaving with new photos, the chat gets one text turn saying so; the
 * voice loop needs a moment to be back before it can take one, and the API
 * gives no acknowledgement; the note is offered once, never repeated. */
#define NOTE_DELAY_MS 3000

#define DCIM_DIR MUSE_SD_MOUNT_POINT "/DCIM"

/* 1: a photo is wanted, 2: leave. Set from the input task. */
enum { WANT_NONE, WANT_PHOTO, WANT_LEAVE };

static esp_err_t (*s_power)(bool on);
static atomic_bool s_active;
static atomic_int s_want;
static bool s_cam_open;        /* esp_camera_init() succeeded, not yet deinit */
static uint8_t *s_stage;       /* VIEW_W*VIEW_H*2, big-endian RGB565 for the UI */
static uint8_t *s_copy;        /* SHOT_BYTES: the photo, before encoding */
static int64_t s_hint_until_us;
static atomic_int s_saved;     /* photos written to the card this session */

/* Talk key, only while active: the release decides photo vs. leave. */
static bool s_key_down;
static bool s_key_taken;       /* the hold already left: the release does nothing */
static int64_t s_key_down_us;

static void log_memory(const char *what)
{
    ESP_LOGI(TAG, "%s: free internal %u (largest %u), psram %u", what,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}

static esp_err_t sensor_up(bool preview)
{
    if (!s_power) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = s_power(true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "sensor power: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(POWER_SETTLE_MS));

    camera_config_t config = {
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
        .ledc_timer = LEDC_TIMER_1,
        .ledc_channel = LEDC_CHANNEL_2,   /* CHANNEL_1 belongs to the status LED */
        .pixel_format = preview ? PIXFORMAT_RGB565 : PIXFORMAT_YUV422,
        .frame_size = FRAMESIZE_VGA,   /* QVGA subsample starved this sensor; VGA is the proven window */
        .jpeg_quality = JPEG_QUALITY,     /* meaningless for raw formats */
        .fb_count = preview ? 2 : 1,   /* one photo buffer: see take_photo */
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
        .sccb_i2c_port = CAM_I2C_PORT,
    };
    err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s init failed: %s", preview ? "preview" : "photo",
                 esp_err_to_name(err));
        s_power(false);
        return err;
    }
    s_cam_open = true;
    /* The gc0308's default output is mirrored; the same-board reference
     * (xiaozhi's SZPI board, QVGA RGB565 on this panel) turns it off, and the
     * photo takes the same setting so what's saved matches the viewfinder. */
    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor) {
        sensor->set_hmirror(sensor, 1);
    }
    /* The first frames are dark while AEC/AGC settles; drop them. */
    for (int i = 0; i < SETTLE_FRAMES; i++) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            esp_camera_fb_return(fb);
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    return ESP_OK;
}

/* Driver down and the rail off: the sensor draws nothing between photos. */
static void sensor_down(void)
{
    if (s_cam_open) {
        esp_camera_deinit();
        s_cam_open = false;
    }
    if (s_power) {
        s_power(false);
    }
}

/* Puts the viewfinder back after a photo, or after a failed try. Returns false
 * when the sensor stays silent: the caller leaves the mode rather than sit in
 * front of a black screen. */
static bool preview_up(void)
{
    for (int i = 0; i < PREVIEW_TRIES; i++) {
        if (sensor_up(true) == ESP_OK) {
            return true;
        }
        sensor_down();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    return false;
}

/* The viewfinder's frame straight onto the panel. Both sides want the same
 * byte order: DVP sensors hand RGB565 out high byte first, which is what the
 * UI's image takes (esp32-camera agrees -- its own JPEG converter treats
 * RGB565 as big-endian). The UI copies the pixels, so the frame buffer can be
 * handed back to the driver as soon as this returns. */
static void draw_rgb565(const uint8_t *src, size_t len)
{
    if (len >= (size_t)VIEW_W * 2 * VIEW_H * 2 * 2 && s_stage) {
        /* VGA frame: every other pixel of every other row into the stage. */
        uint8_t *dst = s_stage;
        for (int y = 0; y < VIEW_H; y++) {
            const uint8_t *row = src + (size_t)y * 2 * VIEW_W * 2 * 2;
            for (int x = 0; x < VIEW_W; x++) {
                const uint8_t *px = row + (size_t)(VIEW_W - 1 - x) * 2 * 2;
                *dst++ = px[0];
                *dst++ = px[1];
            }
        }
        muse_ui_image_draw(0, 0, VIEW_W, VIEW_H, (const uint16_t *)s_stage);
        return;
    }
    if (len < (size_t)VIEW_W * VIEW_H * 2) {
        return;
    }
    muse_ui_image_draw(0, 0, VIEW_W, VIEW_H, (const uint16_t *)src);
}

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

/* The photo's full VGA frame onto the 320x240 panel: every other row and one
 * pixel of each YUYV pair, so the frozen frame the user sees is the photo that
 * was taken, at the panel's size. */
static void draw_yuv422(const uint8_t *yuv, int w, int h)
{
    if (!s_stage || w < VIEW_W * 2 || h < VIEW_H * 2) {
        return;
    }
    const int xstep = w / VIEW_W;
    const int ystep = h / VIEW_H;
    uint8_t *dst = s_stage;
    for (int y = 0; y < VIEW_H; y++) {
        const uint8_t *row = yuv + (size_t)y * ystep * w * 2;
        for (int x = 0; x < VIEW_W; x++) {
            const int sx = x * xstep;
            const uint8_t *p = row + (size_t)(sx & ~1) * 2;
            const int Y = p[0];
            const int U = p[1] - 128;
            const int V = p[3] - 128;
            int c = Y - 16;
            int r = (298 * c + 409 * V + 128) >> 8;
            int g = (298 * c - 100 * U - 208 * V + 128) >> 8;
            int b = (298 * c + 516 * U + 128) >> 8;
            r = r < 0 ? 0 : r > 255 ? 255 : r;
            g = g < 0 ? 0 : g > 255 ? 255 : g;
            b = b < 0 ? 0 : b > 255 ? 255 : b;
            const uint16_t rgb = (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
            *dst++ = (uint8_t)(rgb >> 8);
            *dst++ = (uint8_t)(rgb & 0xFF);
        }
    }
    muse_ui_image_draw(0, 0, VIEW_W, VIEW_H, (const uint16_t *)s_stage);
}

/* The camera app's naming and its "never overwrite" rule: IMG_0001.jpg in
 * /DCIM, continuing from the highest number on the card, so the counter needs
 * no home in NVS and survives reboots and card swaps. */
static int next_photo_index(void)
{
    DIR *dir = opendir(DCIM_DIR);
    if (!dir) {
        return 1;
    }
    int highest = 0;
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        int n = 0;
        char tail[2];
        if (sscanf(entry->d_name, "IMG_%4d.jpg%1s", &n, tail) == 1 && n > highest) {
            highest = n;
        }
    }
    closedir(dir);
    return highest + 1;
}

static esp_err_t save_jpeg(const uint8_t *jpg, size_t len, char *name, size_t cap)
{
    if (muse_sd_mount() != ESP_OK) {
        return ESP_ERR_NOT_FOUND;   /* no card: the caller says so on screen */
    }
    mkdir(DCIM_DIR, 0775);
    int index = next_photo_index();
    for (int tries = 0; tries < 16 && index <= MAX_SHOTS; tries++, index++) {
        snprintf(name, cap, "IMG_%04d.jpg", index);
        char path[64];
        snprintf(path, sizeof(path), "%s/%s", DCIM_DIR, name);
        struct stat st;
        if (stat(path, &st) == 0) {
            continue;
        }
        FILE *f = fopen(path, "wb");
        if (!f) {
            ESP_LOGW(TAG, "open %s failed", path);
            muse_sd_unmount();   /* a yanked card leaves the VFS stale */
            return ESP_FAIL;
        }
        size_t written = fwrite(jpg, 1, len, f);
        bool ok = fclose(f) == 0 && written == len;
        if (!ok) {
            ESP_LOGW(TAG, "write %s failed (%zu/%zu)", path, written, len);
            remove(path);
            muse_sd_unmount();
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "saved %s (%zu bytes)", path, len);
        atomic_fetch_add(&s_saved, 1);
        return ESP_OK;
    }
    return ESP_FAIL;
}

/* ---- The delivery copy: a 320x240 JPEG of the photo, next to it ----
 *
 * The full VGA frame is 16-19 KB, ~25k characters in base64, which is past
 * what the chat can paste back reliably; this halved copy is what gets sent.
 * It is made from the raw YUV422 the photo was encoded from, at capture time,
 * because the board has no JPEG decoder to shrink a finished photo with.
 */

/* Every other pixel pair of every other row: 640x480 YUV422 becomes 320x240
 * YUV422. A YUYV pair is four bytes (Y0 U Y1 V), so taking the pair whole
 * keeps each chroma sample with its own two luma samples -- no decode, no
 * colour conversion. */
static void thumb_downsample(const uint8_t *src, uint8_t *dst)
{
    for (int y = 0; y < THUMB_H; y++) {
        const uint8_t *row = src + (size_t)(y * 2) * SHOT_W * 2;
        for (int x = 0; x < THUMB_W / 2; x++) {   /* 160 output pairs */
            const uint8_t *px = row + (size_t)x * 8;   /* source pair 2x */
            dst[0] = px[0];
            dst[1] = px[1];
            dst[2] = px[2];
            dst[3] = px[3];
            dst += 4;
        }
    }
}

/* The copy's name is the photo's with S before .jpg (IMG_0037.jpg becomes
 * IMG_0037S.jpg). Failure here is logged only: the photo is already on the
 * card, so its count and the screen's SAVED status stay as they are. */
static void save_thumb(const char *photo, const uint8_t *jpg, size_t len)
{
    const size_t base = strlen(photo);
    if (base < 5 || base > 16 || strcmp(photo + base - 4, ".jpg") != 0) {
        ESP_LOGW(TAG, "delivery copy of %s: unexpected name", photo);
        return;
    }
    char name[24];
    snprintf(name, sizeof(name), "%.*sS.jpg", (int)(base - 4), photo);
    char path[64];
    snprintf(path, sizeof(path), "%s/%s", DCIM_DIR, name);
    struct stat st;
    if (stat(path, &st) == 0) {
        ESP_LOGW(TAG, "%s already exists; delivery copy skipped", path);
        return;
    }
    FILE *f = fopen(path, "wb");
    if (!f) {
        /* Log only: unlike save_jpeg, a failed copy must not take the card
         * (and the SHT31 log riding on it) down with it. */
        ESP_LOGW(TAG, "open %s failed", path);
        return;
    }
    size_t written = fwrite(jpg, 1, len, f);
    bool ok = fclose(f) == 0 && written == len;
    if (!ok) {
        ESP_LOGW(TAG, "write %s failed (%zu/%zu)", path, written, len);
        remove(path);
        return;
    }
    ESP_LOGI(TAG, "saved %s (%zu bytes)", path, len);
}

/* One copy from the frame that was just saved. The 150 KB downsample buffer
 * lives for this call only, and every failure just skips the copy: a photo
 * that reached the card must never be lost to its preview. */
static void save_thumb_frame(const uint8_t *yuv, int w, int h, const char *photo)
{
    if (w != SHOT_W || h != SHOT_H) {
        ESP_LOGW(TAG, "no delivery copy for %dx%d", w, h);
        return;
    }
    uint8_t *small = heap_caps_malloc(THUMB_BYTES, MUSE_BIG_CAPS);
    if (!small) {
        ESP_LOGW(TAG, "no memory for the delivery copy; photo kept");
        return;
    }
    thumb_downsample(yuv, small);

    camera_fb_t copy = {
        .buf = small,
        .len = THUMB_BYTES,
        .width = THUMB_W,
        .height = THUMB_H,
        .format = PIXFORMAT_YUV422,
    };
    uint8_t *jpg = NULL;
    size_t jlen = 0;
    bool encoded = frame2jpg(&copy, THUMB_QUALITY, &jpg, &jlen);
    if (encoded && jpg && jlen > 4 &&
        jpg[0] == 0xFF && jpg[1] == 0xD8 && jpg[jlen - 2] == 0xFF && jpg[jlen - 1] == 0xD9) {
        save_thumb(photo, jpg, jlen);
        ESP_LOGI(TAG, "delivery copy %dx%d: %zu bytes", THUMB_W, THUMB_H, jlen);
    } else {
        ESP_LOGW(TAG, "delivery copy encoding failed; photo kept");
    }
    free(jpg);
    free(small);
}

/*
 * One photo, the whole swap: preview down, VGA/YUV422 up, one frame copied out
 * of the driver's buffer, the sensor off again, the copy encoded in software
 * and written to the card, the shot frozen on screen, preview back.
 *
 * A failed switch or a silent sensor is not fatal: the caller shows the error
 * and brings the preview back, so the mode never ends in a black screen.
 * `why` carries what to put on screen; the shot stays frozen there until the
 * caller has shown it.
 */
static void take_photo(const char **why)
{
    sensor_down();
    vTaskDelay(pdMS_TO_TICKS(40));

    const int64_t started = esp_timer_get_time();
    camera_fb_t *fb = NULL;
    size_t len = 0;
    int w = 0, h = 0;
    pixformat_t format = PIXFORMAT_YUV422;
    bool raw = false;
    /* The copy is taken BEFORE the photo init, into a frame-buffer-sized
     * hole the preview just vacated; the photo init (fb_count 1) takes the
     * other. Allocating the copy after the init failed on 2026-11-02: the
     * photo's two buffers had already eaten both holes and a perfect
     * 614400-byte frame was thrown away for want of somewhere to put it. */
    free(s_copy);
    s_copy = heap_caps_malloc(SHOT_BYTES, MUSE_BIG_CAPS);
    if (sensor_up(false) == ESP_OK && (fb = esp_camera_fb_get()) != NULL) {
        len = fb->len;
        w = (int)fb->width;
        h = (int)fb->height;
        format = fb->format;
        raw = format == PIXFORMAT_YUV422 && len <= SHOT_BYTES && s_copy;
        if (raw) {
            /* The copy, not the driver's buffer: the sensor goes down now and
             * the encoder works from memory we own. */
            memcpy(s_copy, fb->buf, len);
            flip_yuv422_h(s_copy, w, h);
        }
        esp_camera_fb_return(fb);
    }
    sensor_down();

    if (!raw) {
        ESP_LOGE(TAG, "no usable frame (%dx%d format %d len %zu)", w, h, (int)format, len);
        *why = "CAMERA ERROR";
        return;
    }

    draw_yuv422(s_copy, w, h);   /* the shot appears while it encodes */

    camera_fb_t copy = {
        .buf = s_copy,
        .len = len,
        .width = (size_t)w,
        .height = (size_t)h,
        .format = PIXFORMAT_YUV422,
    };
    uint8_t *jpg = NULL;
    size_t jlen = 0;
    bool encoded = frame2jpg(&copy, JPEG_QUALITY, &jpg, &jlen);
    if (!encoded || !jpg || jlen < 4 ||
        jpg[0] != 0xFF || jpg[1] != 0xD8 || jpg[jlen - 2] != 0xFF || jpg[jlen - 1] != 0xD9) {
        ESP_LOGE(TAG, "JPEG encoding failed");
        free(jpg);
        *why = "PHOTO FAILED";
    } else {
        char name[32];
        esp_err_t err = save_jpeg(jpg, jlen, name, sizeof(name));
        if (err == ESP_OK) {
            static char saved[48];
            snprintf(saved, sizeof(saved), "SAVED %s", name);
            *why = saved;
        } else if (err == ESP_ERR_NOT_FOUND) {
            *why = "NO SD CARD";
        } else {
            *why = "SAVE FAILED";
        }
        ESP_LOGI(TAG, "photo %dx%d: %zu bytes in %lld ms", w, h, jlen,
                 (esp_timer_get_time() - started) / 1000);
        free(jpg);
        /* After the photo is safe, while the raw frame is still here: the
         * delivery copy comes from it, never from the encoded photo. */
        if (err == ESP_OK) {
            save_thumb_frame(s_copy, w, h, name);
        }
    }
    free(s_copy);
    s_copy = NULL;
}

/* One text turn for the chat, saying what's new on the card. Its own little
 * PSRAM task, so leave_mode doesn't sit through the delay or the queue send. */
#if CONFIG_MUSE_HATCH
static void photo_note_task(void *arg)
{
    const int photos = (int)(intptr_t)arg;
    char text[384];
    snprintf(text, sizeof(text),
             "【相机模式】我刚拍了 %d 张新照片，存在板子的 SD 卡上。请先用 camera.saved_list "
             "看清单，把还没发到主聊的新照片，用 camera.fetch 逐张取它们的交付小图"
             "（文件名在.jpg 前加 S 的那份）发到主聊，并逐张说说每张拍的是什么；"
             "原图太大，不要直接取原图。", photos);

    vTaskDelay(pdMS_TO_TICKS(NOTE_DELAY_MS));
    char *first = strdup(text);
    if (!first) {
        ESP_LOGE(TAG, "no memory for the photo note");
        vTaskDeleteWithCaps(NULL);
        return;
    }
    ESP_LOGI(TAG, "%d new photo(s): text turn offered", photos);
    muse_hatch_text_turn(first);   /* takes ownership; may drop it if busy */
    /* No blind re-offer (review, 2026-11-02): the API gives no ack, and a
     * second identical turn 10 s later would interrupt the very reply the
     * first one started -- fetching photos takes longer than that. A dropped
     * note loses nothing: the photos stay on the card and can be fetched on
     * request at any time. */
    vTaskDeleteWithCaps(NULL);
}
#endif

static void leave_mode(void)
{
#if CONFIG_MUSE_HATCH
    /* Read before active goes false: a quick re-entry resets the counter, and
     * this session's note must still know how many it took. */
    const int photos = atomic_load(&s_saved);
#endif
    sensor_down();
    free(s_stage);
    s_stage = NULL;
    free(s_copy);
    s_copy = NULL;
    muse_ui_cam_status(NULL);
    muse_ui_image_hide();
    muse_state_set_caption("%s", "");
    atomic_store(&s_active, false);   /* the input layer has the key again */
    muse_state_poke();                /* the mode's own time isn't idle time */
    muse_voice_resume();
    log_memory("camera mode off");

    /* Tell the chat about the new photos only when there are some: leaving
     * without a shot must not put anything in the conversation. */
#if CONFIG_MUSE_HATCH
    if (photos > 0 &&
        xTaskCreateWithCaps(photo_note_task, "cam_note", 6144,
                            (void *)(intptr_t)photos, 3, NULL, MUSE_BIG_CAPS) != pdPASS) {
        ESP_LOGE(TAG, "%d new photo(s), but no task for the note", photos);
    }
#endif
}

static void cam_task(void *arg)
{
    (void)arg;
    atomic_store(&s_want, WANT_NONE);
    atomic_store(&s_saved, 0);   /* this session's photos only */
    log_memory("camera mode on");

    /* The camera goes up BEFORE the voice loop parks: parking saves the
     * interrupted note and tears the chat stream down, and that churn
     * fragments the internal heap -- on 2026-11-02 the driver's 30720-byte
     * DMA buffer found a largest free block of only 26624 and init failed
     * before a single frame. Grab the camera at entry, while the heap is
     * still in the shape the idle system left it. */

    s_stage = heap_caps_malloc((size_t)VIEW_W * VIEW_H * 2, MUSE_BIG_CAPS);
    /* s_copy is NOT parked for the session: held across it, its 600 KB
     * starved the photo init's frame buffers (malloc failed, 2026-11-02).
     * take_photo borrows it for the shot only. */
    if (!s_stage) {
        ESP_LOGE(TAG, "no memory for the viewfinder");
        muse_ui_cam_status("CAMERA MEMORY ERROR");
        vTaskDelay(pdMS_TO_TICKS(1500));
        leave_mode();
        vTaskDeleteWithCaps(NULL);
        return;
    }

    if (!preview_up()) {
        muse_ui_cam_status("CAMERA NOT READY");
        vTaskDelay(pdMS_TO_TICKS(1500));
        leave_mode();
        vTaskDeleteWithCaps(NULL);
        return;
    }
    /* The voice loop parks within a poll; if it never started there is nothing
     * to wait for. */
    muse_voice_suspend();
    for (int i = 0; i < 100 && !muse_voice_suspended(); i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (!muse_voice_suspended()) {
        ESP_LOGW(TAG, "voice didn't report suspended");
    }
    muse_state_set_caption("%s", "");

    s_hint_until_us = esp_timer_get_time() + ENTRY_HINT_MS * 1000;
    muse_ui_cam_status("TAP: PHOTO  HOLD: EXIT");

    int misses = 0;
    bool alive = true;
    while (alive) {
        int want = atomic_load(&s_want);
        if (want == WANT_LEAVE) {
            break;
        }
        if (want == WANT_PHOTO) {
            atomic_store(&s_want, WANT_NONE);
            s_hint_until_us = 0;
            const char *note = "PHOTO FAILED";
            take_photo(&note);
            muse_ui_cam_status(note);
            vTaskDelay(pdMS_TO_TICKS(FREEZE_MS));   /* the confirmation look */
            muse_ui_cam_status(NULL);
            if (!preview_up()) {
                muse_ui_cam_status("CAMERA ERROR");
                vTaskDelay(pdMS_TO_TICKS(FREEZE_MS));
                muse_ui_cam_status(NULL);
                break;
            }
            continue;
        }

        const int64_t frame_at = esp_timer_get_time();
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            if (++misses >= 3) {
                ESP_LOGW(TAG, "no frames; restarting the viewfinder");
                sensor_down();
                if (!preview_up()) {
                    muse_ui_cam_status("CAMERA ERROR");
                    vTaskDelay(pdMS_TO_TICKS(FREEZE_MS));
                    alive = false;
                    break;
                }
                misses = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        misses = 0;
        if (fb->format == PIXFORMAT_RGB565) {
            draw_rgb565(fb->buf, fb->len);
        }
        esp_camera_fb_return(fb);

        if (s_hint_until_us && esp_timer_get_time() > s_hint_until_us) {
            s_hint_until_us = 0;
            muse_ui_cam_status(NULL);
        }
        const int64_t spent_ms = (esp_timer_get_time() - frame_at) / 1000;
        if (spent_ms < VIEW_FRAME_MS) {
            vTaskDelay(pdMS_TO_TICKS(VIEW_FRAME_MS - spent_ms));
        }
    }

    leave_mode();
    vTaskDeleteWithCaps(NULL);
}

esp_err_t lichuang_cam_mode_prepare(esp_err_t (*power)(bool on))
{
    s_power = power;
    return power ? ESP_OK : ESP_ERR_INVALID_ARG;
}

bool lichuang_cam_mode_active(void)
{
    return atomic_load(&s_active);
}

bool lichuang_cam_mode_enter(void)
{
    if (!s_power) {
        return false;
    }
    if (atomic_exchange(&s_active, true)) {
        return true;
    }
    if (muse_board->width != VIEW_W || muse_board->height != VIEW_H) {
        atomic_store(&s_active, false);
        return false;
    }
    s_key_down = false;
    s_key_taken = false;
    /* The stack is PSRAM, so the encoder's occasional deep frame is cheap;
     * the size matches main/app.c's camera_capture task, which runs the same
     * frame2jpg path. */
    if (xTaskCreateWithCaps(cam_task, "cam_mode", 16384, NULL, 4, NULL, MUSE_BIG_CAPS) != pdPASS) {
        ESP_LOGE(TAG, "could not start the camera task");
        atomic_store(&s_active, false);
        return false;
    }
    return true;
}

void lichuang_cam_mode_shutter(void)
{
    int none = WANT_NONE;
    atomic_compare_exchange_strong(&s_want, &none, WANT_PHOTO);
}

void lichuang_cam_mode_leave(void)
{
    atomic_store(&s_want, WANT_LEAVE);
}

void lichuang_cam_mode_key(bool pressed)
{
    const int64_t now = esp_timer_get_time();
    if (pressed) {
        s_key_down = true;
        s_key_taken = false;
        s_key_down_us = now;
        return;
    }
    if (!s_key_down) {
        return;   /* the press that entered the mode: its release is not a photo */
    }
    s_key_down = false;
    if (s_key_taken) {
        s_key_taken = false;
        return;
    }
    const int64_t held_ms = (now - s_key_down_us) / 1000;
    if (held_ms >= 800) {
        lichuang_cam_mode_leave();
    } else if (held_ms < 500) {
        lichuang_cam_mode_shutter();
    }
}

void lichuang_cam_mode_tick(void)
{
    if (s_key_down && !s_key_taken &&
        (esp_timer_get_time() - s_key_down_us) / 1000 >= 800) {
        s_key_taken = true;   /* the release must not also count */
        lichuang_cam_mode_leave();
    }
}
