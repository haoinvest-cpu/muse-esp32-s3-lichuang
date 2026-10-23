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

#include "muse_weather.h"

#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "cJSON.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "muse_sht31.h"

static const char *TAG = "weather";

/* The Pi hub: sht31-hub on the hotspot gateway the board already joins.
 * Plain HTTP on the home LAN; nothing here talks to the internet. */
#define WEATHER_HUB_BASE "http://192.168.4.1:8021"

#define WEATHER_PERIOD_US    (1800LL * 1000 * 1000)   /* forecast: 30 min */
#define WEATHER_RETRY_US     (120LL * 1000 * 1000)    /* after a failure */
#define WEATHER_FIRST_US     (15LL * 1000 * 1000)     /* first try at +15 s */
#define WEATHER_LOOP_MS      15000
#define WEATHER_HTTP_TIMEOUT_MS 12000
#define WEATHER_JSON_CAP     (16 * 1024)
/* time() past this (2024-01-01 UTC) is a wall clock, not uptime. */
#define WEATHER_WALL_EPOCH_MIN 1704067200

static SemaphoreHandle_t s_lock;
static muse_weather_t s_wx;
static bool s_started;
static bool s_tz_set;

/* ---- one HTTP round trip, response accumulated in PSRAM ---- */

typedef struct {
    char *buf;
    size_t len, cap;
    bool oom;
} resp_t;

static esp_err_t on_http(esp_http_client_event_t *evt)
{
    if (evt->event_id == HTTP_EVENT_ON_DATA) {
        resp_t *r = evt->user_data;
        if (r->len + (size_t)evt->data_len + 1 > r->cap) {
            r->oom = true;
            return ESP_FAIL;
        }
        memcpy(r->buf + r->len, evt->data, (size_t)evt->data_len);
        r->len += (size_t)evt->data_len;
        r->buf[r->len] = '\0';
    }
    return ESP_OK;
}

/* GET base+path into a fresh PSRAM buffer (caller frees). NULL on any
 * failure: network down, non-200, or a response over cap. */
static char *http_get(const char *path, size_t cap)
{
    resp_t r = { .cap = cap };
    r.buf = heap_caps_malloc(cap, MALLOC_CAP_SPIRAM);
    if (!r.buf) {
        return NULL;
    }
    r.buf[0] = '\0';
    char url[64];
    snprintf(url, sizeof(url), WEATHER_HUB_BASE "%s", path);
    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .timeout_ms = WEATHER_HTTP_TIMEOUT_MS,
        .event_handler = on_http,
        .user_data = &r,
    };
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (!c) {
        free(r.buf);
        return NULL;
    }
    esp_err_t err = esp_http_client_perform(c);
    int status = esp_http_client_get_status_code(c);
    esp_http_client_cleanup(c);
    if (err != ESP_OK || status != 200 || r.oom) {
        free(r.buf);
        return NULL;
    }
    return r.buf;
}

static bool http_post_reading(float temp_c, float rh)
{
    char body[48];
    snprintf(body, sizeof(body), "{\"t\":%.1f,\"rh\":%.0f}",
             (double)temp_c, (double)rh);
    esp_http_client_config_t cfg = {
        .url = WEATHER_HUB_BASE "/reading",
        .method = HTTP_METHOD_POST,
        .timeout_ms = WEATHER_HTTP_TIMEOUT_MS,
    };
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (!c) {
        return false;
    }
    esp_http_client_set_header(c, "Content-Type", "application/json");
    esp_http_client_set_post_field(c, body, (int)strlen(body));
    esp_err_t err = esp_http_client_perform(c);
    int status = esp_http_client_get_status_code(c);
    esp_http_client_cleanup(c);
    return err == ESP_OK && status == 200;
}

/* ---- clock ---- */

/* The hub's epoch is the board's first wall clock: nothing else in this
 * firmware sets the time. Setting it also gives the SHT31 SD log real
 * timestamps and the weather tile a real clock. Fixed UTC+8 (the board
 * lives in China, no DST), the same convention as muse_sht31.c. */
static void sync_clock(int64_t epoch)
{
    if (epoch < WEATHER_WALL_EPOCH_MIN) {
        return;
    }
    time_t now = time(NULL);
    long long diff = (long long)epoch - (long long)now;
    if (diff > 5 || diff < -5) {
        struct timeval tv = { .tv_sec = (time_t)epoch, .tv_usec = 0 };
        if (settimeofday(&tv, NULL) == 0) {
            ESP_LOGI(TAG, "clock set from hub");
        }
    }
    if (!s_tz_set) {
        setenv("TZ", "CST-8", 1);
        tzset();
        s_tz_set = true;
    }
}

/* ---- payload parsing ---- */

static void copy_str(char *dst, size_t cap, const cJSON *obj, const char *key)
{
    const cJSON *v = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (cJSON_IsString(v) && v->valuestring) {
        snprintf(dst, cap, "%s", v->valuestring);
    }
}

static double get_num(const cJSON *obj, const char *key, double dflt)
{
    const cJSON *v = cJSON_GetObjectItemCaseSensitive(obj, key);
    return cJSON_IsNumber(v) ? v->valuedouble : dflt;
}

static bool fetch_weather(void)
{
    char *raw = http_get("/weather", WEATHER_JSON_CAP);
    if (!raw) {
        return false;
    }
    cJSON *root = cJSON_Parse(raw);
    free(raw);
    if (!root) {
        return false;
    }
    muse_weather_t w = { 0 };
    w.epoch = (int64_t)get_num(root, "epoch", 0);
    copy_str(w.cond, sizeof(w.cond), root, "cond");
    w.temp = (float)get_num(root, "temp", 0);
    w.hi = (int)get_num(root, "hi", 0);
    w.lo = (int)get_num(root, "lo", 0);
    w.precip = (int)get_num(root, "precip", -1);
    copy_str(w.advice, sizeof(w.advice), root, "advice");
    copy_str(w.speak, sizeof(w.speak), root, "speak");
    const cJSON *days = cJSON_GetObjectItemCaseSensitive(root, "days");
    if (cJSON_IsArray(days)) {
        int n = cJSON_GetArraySize(days);
        if (n > 3) {
            n = 3;
        }
        for (int i = 0; i < n; i++) {
            const cJSON *d = cJSON_GetArrayItem(days, i);
            if (!cJSON_IsObject(d)) {
                continue;
            }
            copy_str(w.days[i].name, sizeof(w.days[i].name), d, "name");
            copy_str(w.days[i].cond, sizeof(w.days[i].cond), d, "cond");
            w.days[i].lo = (int)get_num(d, "lo", 0);
            w.days[i].hi = (int)get_num(d, "hi", 0);
            w.days[i].precip = (int)get_num(d, "precip", -1);
        }
    }
    bool ok = w.cond[0] != '\0';
    cJSON_Delete(root);
    if (!ok) {
        return false;
    }
    sync_clock(w.epoch);
    xSemaphoreTake(s_lock, portMAX_DELAY);
    w.version = s_wx.version + 1;
    w.valid = true;
    s_wx = w;
    xSemaphoreGive(s_lock);
    ESP_LOGI(TAG, "weather: %s %.1f C, today %d..%d C",
             w.cond, (double)w.temp, w.lo, w.hi);
    return true;
}

/* ---- the task ---- */

static void weather_task(void *arg)
{
    (void)arg;
    int64_t next_fetch_us = esp_timer_get_time() + WEATHER_FIRST_US;
    int64_t last_posted_ts = 0;
    bool last_posted_wall = false;
    int fail_streak = 0;
    for (;;) {
        int64_t now = esp_timer_get_time();
        if (now >= next_fetch_us) {
            if (fetch_weather()) {
                fail_streak = 0;
                next_fetch_us = now + WEATHER_PERIOD_US;
            } else {
                fail_streak++;
                if (fail_streak == 1 || fail_streak % 10 == 0) {
                    ESP_LOGW(TAG, "hub fetch failed (%d in a row)", fail_streak);
                }
                next_fetch_us = now + WEATHER_RETRY_US;
            }
        }
        /* Durable record: hand each new 5-minute point to the hub. A
         * failed post is retried on the next pass; the point stays the
         * newest until the sampler adds another, so nothing is lost
         * unless the board is down for a whole bucket. */
        int n = muse_sht31_history_count();
        if (n > 0) {
            float t, rh;
            int64_t ts;
            bool wall;
            if (muse_sht31_history_point(n - 1, &t, &rh, &ts, &wall) &&
                (ts != last_posted_ts || wall != last_posted_wall)) {
                if (http_post_reading(t, rh)) {
                    last_posted_ts = ts;
                    last_posted_wall = wall;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(WEATHER_LOOP_MS));
    }
}

esp_err_t muse_weather_start(void)
{
    if (s_started) {
        return ESP_OK;
    }
    if (!s_lock) {
        s_lock = xSemaphoreCreateMutex();
        if (!s_lock) {
            return ESP_ERR_NO_MEM;
        }
    }
    /* 8192: the HTTP client plus cJSON parsing runs deep. The stack
     * lives in PSRAM too (WithCaps): an internal-RAM stack here cost
     * 8 KB of the scarcest memory on the board for the task's whole
     * life, and the chat session's VM-credential lookup -- which must
     * create its own 8 KB internal-stack task on demand -- started
     * failing for want of one contiguous block. This task only does
     * plain HTTP and JSON, so a PSRAM stack is safe for it. */
    if (xTaskCreatePinnedToCoreWithCaps(weather_task, "weather", 8192, NULL, 4, NULL,
                                        tskNO_AFFINITY, MALLOC_CAP_SPIRAM) != pdPASS) {
        ESP_LOGE(TAG, "task failed to start");
        return ESP_ERR_NO_MEM;
    }
    s_started = true;
    ESP_LOGI(TAG, "hub task started (%s)", WEATHER_HUB_BASE);
    return ESP_OK;
}

bool muse_weather_enabled(void)
{
    return s_started;
}

bool muse_weather_get(muse_weather_t *out)
{
    if (!out || !s_started) {
        return false;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool valid = s_wx.valid;
    if (valid) {
        *out = s_wx;
    }
    xSemaphoreGive(s_lock);
    return valid;
}
