/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#include "lichuang_camera.h"

#include <dirent.h>
#include <stdlib.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#include "lichuang_camera.h"

#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "camera.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "mbedtls/base64.h"
#include "muse_board.h"
#include "muse_mem.h"
#include "muse_sd.h"
#include "muse_state.h"
#include "muse_ui.h"
#include "rom/tjpgd.h"

#include "lichuang_cam_mode.h"

/*
 * The Lichuang's camera.capture and live preview, on the board's camera
 * (components/camera; board_lichuang_szp.c registers the gc0308). The
 * camera is powered only while a capture or the preview runs.
 */

static const char *TAG = "lichuang.camera";
static SemaphoreHandle_t s_lock;     /* s_last */
static uint8_t *s_last;              /* the preview's newest frame, a JPEG */
static size_t s_last_len;
static atomic_bool s_preview_active;
static atomic_bool s_busy;           /* a capture, or the preview starting or stopping */

#define CAMERA_PREVIEW_SIZE 412
#define CAMERA_JPEG_POOL 3100

#define CAMERA_DCIM MUSE_SD_MOUNT_POINT "/DCIM"
#define PHOTO_NAME_MAX 32              /* long enough for IMG_0001.jpg, with room */
#define SAVED_LIST_MAX 50              /* the most camera.saved_list reports */
#define FETCH_MAX_BYTES (512 * 1024)   /* a photo bigger than this is an error */

typedef struct {
    const uint8_t *data;
    size_t length;
    size_t offset;
} jpeg_source_t;

static UINT jpeg_read(JDEC *jd, BYTE *buffer, UINT count)
{
    jpeg_source_t *src = jd->device;
    size_t remain = src->length - src->offset;
    if (count > remain) count = remain;
    if (buffer && count) memcpy(buffer, src->data + src->offset, count);
    src->offset += count;
    return count;
}

static UINT jpeg_draw(JDEC *jd, void *bitmap, JRECT *rect)
{
    int left = rect->left;
    int top = rect->top;
    int right = rect->right > CAMERA_PREVIEW_SIZE - 1 ? CAMERA_PREVIEW_SIZE - 1 : rect->right;
    int bottom = rect->bottom > CAMERA_PREVIEW_SIZE - 1 ? CAMERA_PREVIEW_SIZE - 1 : rect->bottom;
    if (left > right || top > bottom) return 1;

    int source_w = rect->right - rect->left + 1;
    int width = right - left + 1;
    int height = bottom - top + 1;
    uint16_t pixels[16 * 16];
    uint8_t *pixel_bytes = (uint8_t *)pixels;
    const uint8_t *rgb = bitmap;
    for (int y = top; y <= bottom; y++) {
        for (int x = left; x <= right; x++) {
            size_t si = ((size_t)(y - rect->top) * source_w + (x - rect->left)) * 3;
            size_t di = ((size_t)(y - top) * width + (x - left)) * 2;
            uint16_t px = ((rgb[si] & 0xF8) << 8) | ((rgb[si + 1] & 0xFC) << 3) | (rgb[si + 2] >> 3);
            pixel_bytes[di] = px >> 8;
            pixel_bytes[di + 1] = px & 0xFF;
        }
    }
    muse_ui_image_draw(left, top, width, height, pixels);
    return 1;
}

static void preview_draw(const uint8_t *jpeg, size_t len)
{
    void *pool = heap_caps_malloc(CAMERA_JPEG_POOL, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!pool) return;
    jpeg_source_t src = { .data = jpeg, .length = len };
    JDEC jd;
    if (jd_prepare(&jd, jpeg_read, pool, CAMERA_JPEG_POOL, &src) == JDR_OK) {
        if (jd.width > 416 || jd.height > 416) {
            muse_state_set_caption("CAMERA FRAME TOO LARGE");
        } else {
            jd_decomp(&jd, jpeg_draw, 0);
        }
    }
    free(pool);
}

/* Each streamed frame: kept for a capture during the preview, and drawn. */
static void on_frame(const camera_frame_t *frame, void *ctx)
{
    (void)ctx;
    if (!s_preview_active) return;
    uint8_t *copy = heap_caps_malloc(frame->len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (copy) {
        memcpy(copy, frame->jpeg, frame->len);
        xSemaphoreTake(s_lock, portMAX_DELAY);
        free(s_last);
        s_last = copy;
        s_last_len = frame->len;
        xSemaphoreGive(s_lock);
    }
    preview_draw(frame->jpeg, frame->len);
}

esp_err_t lichuang_camera_prepare(void)
{
    if (!s_lock) s_lock = xSemaphoreCreateMutex();
    return s_lock ? ESP_OK : ESP_ERR_NO_MEM;
}

/* `jpeg` as base64, for camera.capture's result. */
static char *to_base64(const uint8_t *jpeg, size_t len)
{
    size_t cap = (len + 2) / 3 * 4 + 1, out = 0;
    char *b64 = heap_caps_malloc(cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (b64 && mbedtls_base64_encode((unsigned char *)b64, cap, &out, jpeg, len) != 0) {
        free(b64);
        return NULL;
    }
    return b64;
}

static bool capture_frame(char **jpeg_base64, char **thumb_base64, const char **error)
{
#if CONFIG_MUSE_LICHUANG_CAMERA
    if (lichuang_cam_mode_active()) {
        /* Camera mode has the sensor initialized for its viewfinder. */
        *error = "camera busy";
        return false;
    }
#endif
    if (s_preview_active) {
        /* The preview has the camera: the frame it's showing is the photo. */
        xSemaphoreTake(s_lock, portMAX_DELAY);
        *jpeg_base64 = s_last ? to_base64(s_last, s_last_len) : NULL;
        bool had = s_last != NULL;
        xSemaphoreGive(s_lock);
        *error = !had ? "no camera frame yet" : *jpeg_base64 ? NULL : "camera memory allocation failed";
        return *jpeg_base64 != NULL;
    }
    camera_frame_t frame;
    esp_err_t err = camera_capture(&frame);
    if (err != ESP_OK) {
        *error = err == ESP_ERR_TIMEOUT ? "gc0308 camera did not respond"
                 : err == ESP_ERR_NO_MEM ? "camera memory allocation failed"
                 : err == ESP_ERR_NOT_SUPPORTED ? "camera unavailable"
                                                 : "camera capture failed";
        return false;
    }
    *jpeg_base64 = to_base64(frame.jpeg, frame.len);
    *thumb_base64 = frame.thumb ? to_base64(frame.thumb, frame.thumb_len) : NULL;
    camera_release(&frame);
    *error = *jpeg_base64 ? NULL : "camera memory allocation failed";
    return *jpeg_base64 != NULL;
}

bool lichuang_camera_capture(char **jpeg_base64, char **thumb_base64, const char **error)
{
    if (!jpeg_base64 || !thumb_base64 || !error) return false;
    *jpeg_base64 = NULL;
    *thumb_base64 = NULL;
    if (lichuang_camera_prepare() != ESP_OK) {
        *error = "camera memory allocation failed";
        return false;
    }
    if (atomic_exchange(&s_busy, true)) {
        *error = "camera busy";
        return false;
    }
    bool ok = capture_frame(jpeg_base64, thumb_base64, error);
    atomic_store(&s_busy, false);
    return ok;
}

/* ---- The photos already on the card (camera.saved_list / camera.fetch) ----
 *
 * Camera mode writes IMG_%04d.jpg into /DCIM; these two commands let the chat
 * read those back without touching the sensor. The card file is never
 * modified: fetch reads a copy into PSRAM, base64s it and drops the copy.
 */

/* What both commands accept: a bare file name under /DCIM, of the shape
 * camera mode writes. Anything with a path in it is refused before it can
 * escape the directory. */
static bool photo_name_ok(const char *name)
{
    if (!name) return false;
    size_t len = strlen(name);
    if (len < 8 || len >= PHOTO_NAME_MAX) return false;   /* IMG_ + n + .jpg */
    if (strncmp(name, "IMG_", 4) != 0) return false;
    if (strcmp(name + len - 4, ".jpg") != 0) return false;
    if (strchr(name, '/') || strchr(name, '\\') || strstr(name, "..")) return false;
    return true;
}

/* The camera mode's delivery copy sits next to its photo under the same name
 * with S before .jpg (IMG_0037S.jpg). It is a real file -- camera.fetch takes
 * it -- but it is not a photo, so saved_list leaves it out. */
static bool photo_is_thumb(const char *name)
{
    if (strlen(name) != 13) return false;
    if (strncmp(name, "IMG_", 4) != 0 || name[8] != 'S' ||
        strcmp(name + 9, ".jpg") != 0) {
        return false;
    }
    for (int i = 4; i < 8; i++) {
        if (name[i] < '0' || name[i] > '9') return false;
    }
    return true;
}

typedef struct {
    char name[PHOTO_NAME_MAX];
    long bytes;
} saved_photo_t;

/* Keeps the list sorted by name and at most SAVED_LIST_MAX long, so the
 * result is the first 50 names of the directory in ascending order however
 * readdir orders them. */
static void saved_insert(saved_photo_t *list, int *count, const char *name, long bytes)
{
    int at = *count;
    if (at == SAVED_LIST_MAX) {
        if (strcmp(name, list[at - 1].name) >= 0) return;
        at--;
    } else {
        (*count)++;
    }
    while (at > 0 && strcmp(list[at - 1].name, name) > 0) {
        list[at] = list[at - 1];
        at--;
    }
    snprintf(list[at].name, sizeof(list[at].name), "%.*s",
             (int)sizeof(list[at].name) - 1, name);
    list[at].bytes = bytes;
}

static cJSON *error_json(const char *code, const char *message)
{
    cJSON *result = cJSON_CreateObject();
    if (!result) return NULL;
    cJSON_AddBoolToObject(result, "ok", false);
    cJSON *error = cJSON_AddObjectToObject(result, "error");
    if (error) {
        cJSON_AddStringToObject(error, "code", code);
        cJSON_AddStringToObject(error, "message", message);
    }
    return result;
}

cJSON *lichuang_camera_saved_list(void)
{
    if (lichuang_cam_mode_active()) {
        return error_json("camera_busy", "camera busy");
    }

    saved_photo_t photos[SAVED_LIST_MAX];
    int count = 0;
    /* No card or no DCIM yet is not an error: the list is simply empty. */
    if (muse_sd_mount() == ESP_OK) {
        DIR *dir = opendir(CAMERA_DCIM);
        if (dir) {
            struct dirent *entry;
            while ((entry = readdir(dir)) != NULL) {
                if (!photo_name_ok(entry->d_name)) continue;
                if (photo_is_thumb(entry->d_name)) continue;
                char path[64];
                snprintf(path, sizeof(path), "%s/%.*s", CAMERA_DCIM,
                         PHOTO_NAME_MAX - 1, entry->d_name);
                struct stat st;
                if (stat(path, &st) != 0 || !S_ISREG(st.st_mode)) {
                    continue;
                }
                saved_insert(photos, &count, entry->d_name, (long)st.st_size);
            }
            closedir(dir);
        }
    }

    cJSON *result = cJSON_CreateObject();
    if (!result) return NULL;
    cJSON_AddBoolToObject(result, "ok", true);
    cJSON *payload = cJSON_AddObjectToObject(result, "payload");
    if (!payload) {
        cJSON_Delete(result);
        return NULL;
    }
    cJSON_AddNumberToObject(payload, "count", count);
    cJSON *files = cJSON_AddArrayToObject(payload, "files");
    for (int i = 0; files && i < count; i++) {
        cJSON *item = cJSON_CreateObject();
        if (!item) break;
        cJSON_AddStringToObject(item, "name", photos[i].name);
        cJSON_AddNumberToObject(item, "bytes", (double)photos[i].bytes);
        cJSON_AddItemToArray(files, item);
    }
    return result;
}

bool lichuang_camera_fetch(const char *name, char **jpeg_base64, const char **error)
{
    if (!jpeg_base64 || !error) return false;
    *jpeg_base64 = NULL;
    if (lichuang_cam_mode_active()) {
        *error = "camera busy";
        return false;
    }
    if (!photo_name_ok(name)) {
        *error = "invalid photo name";
        return false;
    }
    /* Shares s_busy with camera.capture: one reader of the card at a time,
     * and no capture starting while a 512 KB read is under way. */
    if (atomic_exchange(&s_busy, true)) {
        *error = "camera busy";
        return false;
    }

    const char *why = NULL;
    char *b64 = NULL;
    uint8_t *buf = NULL;
    char path[64];
    snprintf(path, sizeof(path), "%s/%.*s", CAMERA_DCIM, PHOTO_NAME_MAX - 1, name);

    if (muse_sd_mount() != ESP_OK) {
        why = "no sd card";
        goto out;
    }
    struct stat st;
    if (stat(path, &st) != 0 || !S_ISREG(st.st_mode)) {
        why = "photo not found";
        goto out;
    }
    if (st.st_size <= 0) {
        why = "photo is empty";
        goto out;
    }
    if ((unsigned long)st.st_size > FETCH_MAX_BYTES) {
        why = "photo too large";
        goto out;
    }
    buf = heap_caps_malloc((size_t)st.st_size, MUSE_BIG_CAPS);
    if (!buf) {
        why = "camera memory allocation failed";
        goto out;
    }
    FILE *f = fopen(path, "rb");
    if (!f) {
        why = "photo not found";
        goto out;
    }
    size_t got = fread(buf, 1, (size_t)st.st_size, f);
    fclose(f);
    if (got != (size_t)st.st_size) {
        why = "photo read failed";
        goto out;
    }
    b64 = to_base64(buf, got);
    if (!b64) why = "camera memory allocation failed";

out:
    free(buf);   /* the copy, never the card */
    atomic_store(&s_busy, false);
    *jpeg_base64 = b64;
    *error = why;
    return b64 != NULL;
}

static void preview_toggle_task(void *arg)
{
    (void)arg;
    if (!s_preview_active) {
        /* The view first, so the stream's first frames land on it. */
        s_preview_active = true;
        muse_state_set_caption("LIVE VIEW • TAP IMAGE TO TAKE PHOTO");
        muse_ui_image_hide();
        // muse_ui_camera_hint(true);
        muse_board->display_lock(-1);
        muse_ui_show_face();
        muse_board->display_unlock();
        esp_err_t err = camera_stream_start(on_frame, NULL);
        if (err != ESP_OK) {
            s_preview_active = false;
            // muse_ui_camera_hint(false);
            ESP_LOGW(TAG, "preview didn't start: %s", esp_err_to_name(err));
            muse_state_set_caption(err == ESP_ERR_TIMEOUT ? "GC0308 CAMERA NOT READY" : "CAMERA PREVIEW FAILED");
        }
    } else {
        s_preview_active = false;
        camera_stream_stop();
        xSemaphoreTake(s_lock, portMAX_DELAY);
        bool have_frame = s_last != NULL;
        free(s_last);
        s_last = NULL;
        xSemaphoreGive(s_lock);
        // muse_ui_camera_hint(false);
        muse_state_set_caption(have_frame ? "PHOTO CAPTURED" : "NO CAMERA FRAME • TRY AGAIN");
    }
    atomic_store(&s_busy, false);
    vTaskDeleteWithCaps(NULL);
}

bool lichuang_camera_preview_active(void)
{
    return s_preview_active;
}

void lichuang_camera_preview_toggle(void)
{
    if (lichuang_camera_prepare() != ESP_OK) {
        muse_state_set_caption("CAMERA MEMORY ERROR");
        return;
    }
    if (atomic_exchange(&s_busy, true)) return;
    if (xTaskCreateWithCaps(preview_toggle_task, "camera_view", 8192, NULL, 4, NULL,
                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
        != pdPASS) {
        ESP_LOGE(TAG, "could not start camera preview task");
        atomic_store(&s_busy, false);
    }
}
