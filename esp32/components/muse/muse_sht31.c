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

#include "muse_sht31.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "minimp3.h"
#include "muse_audio.h"
#include "muse_sd.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "sht31";

/* The module answers at 0x44 unless its address pin is strapped high. */
#define SHT31_ADDR_DEFAULT 0x44
#define SHT31_ADDR_ALT     0x45

#define SHT31_PERIOD_MS 30000
/* High-repeatability single shot takes up to 15.5 ms; 20 ms covers it. */
#define SHT31_MEASURE_WAIT_MS 20
#define SHT31_XFER_TIMEOUT_MS 100
/* The cached reading goes stale after this many failed samples in a row. */
#define SHT31_STALE_AFTER 2

/* History: one averaged point per this many samples (10 x 30 s = 5 min). */
#define SHT31_SAMPLES_PER_POINT 10

/*
 * Spoken alerts. A crossing speaks once, then that direction stays quiet
 * until the reading retreats past the hysteresis band (re-arm) AND the
 * cooldown has elapsed. Tune here; there is no settings UI for these.
 */
#define SHT31_ALERT_HIGH_C        30.0f
#define SHT31_ALERT_LOW_C         15.0f
#define SHT31_ALERT_HYSTERESIS_C  0.5f
#define SHT31_ALERT_COOLDOWN_US   (30LL * 60 * 1000 * 1000)

/*
 * SD log: the board's TF slot, mounted by muse_sd (the camera writes photos to
 * the same card). One CSV per local day in /sdcard/sht31, one row per sample.
 * Local time is a fixed UTC offset (the board lives in China, no DST); the
 * firmware has no clock source, so until something sets the time the log falls
 * back to one no-time.csv whose timestamps are seconds since boot (the header
 * says so).
 */
#define SHT31_SD_DIR           MUSE_SD_MOUNT_POINT "/sht31"
#define SHT31_SD_RETRY_US      (10LL * 60 * 1000 * 1000)
#define SHT31_LOCAL_UTC_OFFSET_S (8 * 3600)
/* time() past this (2024-01-01 UTC) is a wall clock, not uptime. */
#define SHT31_WALL_EPOCH_MIN   1704067200

static i2c_master_dev_handle_t s_dev;
static bool s_present;

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static float s_temp_c, s_rh;
static bool s_valid;
static int64_t s_last_ok_us;

/* History ring, oldest at s_hist_head - s_hist_count (mod capacity). */
typedef struct {
    float temp_c, rh;
    int64_t ts;   /* wall epoch seconds when wall, else boot seconds */
    bool wall;
} sht31_point_t;
/* Allocated in PSRAM by muse_sht31_start(): as a static this ring cost
 * ~7 KB of internal DRAM, which starved other subsystems' task stacks
 * (the chat session's VM lookup task failed to spawn, silently). */
static sht31_point_t *s_hist;
static int s_hist_head;    /* next write slot */
static int s_hist_count;
static float s_acc_t, s_acc_rh;   /* running sums for the open bucket */
static int s_acc_n;

/* Alert state: armed flags re-arm past the hysteresis band. */
static bool s_alert_high_armed = true, s_alert_low_armed = true;
static int64_t s_alert_high_last_us, s_alert_low_last_us;   /* 0 = never */

/* SD state: mounted once by the sampler; retries on a slow cadence. */
static bool s_sd_mounted;
static int64_t s_sd_next_try_us;

/* SHT3x CRC-8 over one 16-bit word: polynomial 0x31, initial value 0xFF. */
static uint8_t sht31_crc(const uint8_t *word)
{
    uint8_t crc = 0xFF;
    for (int i = 0; i < 2; i++) {
        crc ^= word[i];
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : crc << 1;
        }
    }
    return crc;
}

/* One single-shot measurement (0x2400: high repeatability, clock stretching
 * off, so the read waits the full conversion time first). */
static esp_err_t sht31_sample(float *temp_c, float *rh)
{
    const uint8_t measure[] = { 0x24, 0x00 };
    esp_err_t err = i2c_master_transmit(s_dev, measure, sizeof(measure), SHT31_XFER_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(SHT31_MEASURE_WAIT_MS));
    uint8_t buf[6];   /* temp word + CRC, humidity word + CRC */
    err = i2c_master_receive(s_dev, buf, sizeof(buf), SHT31_XFER_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    if (sht31_crc(buf) != buf[2] || sht31_crc(buf + 3) != buf[5]) {
        return ESP_ERR_INVALID_CRC;
    }
    uint16_t raw_temp = (uint16_t)(buf[0] << 8 | buf[1]);
    uint16_t raw_rh = (uint16_t)(buf[3] << 8 | buf[4]);
    *temp_c = -45.0f + 175.0f * (float)raw_temp / 65535.0f;
    *rh = 100.0f * (float)raw_rh / 65535.0f;
    if (*rh > 100.0f) {
        *rh = 100.0f;
    }
    return ESP_OK;
}

/* The one timestamp used everywhere below: wall-clock epoch seconds when
 * the system clock has been set (nothing in this firmware sets it yet, so
 * normally this is seconds since boot and *wall is false). */
static void sht31_stamp(int64_t *ts, bool *wall)
{
    time_t now = time(NULL);
    if (now > SHT31_WALL_EPOCH_MIN) {
        *ts = (int64_t)now;
        *wall = true;
    } else {
        *ts = esp_timer_get_time() / 1000000;
        *wall = false;
    }
}

/* Caller holds s_lock. */
static void sht31_hist_push_locked(float temp_c, float rh, int64_t ts, bool wall)
{
    if (!s_hist) {
        return;   /* history unavailable (PSRAM allocation failed) */
    }
    sht31_point_t *p = &s_hist[s_hist_head];
    p->temp_c = temp_c;
    p->rh = rh;
    p->ts = ts;
    p->wall = wall;
    s_hist_head = (s_hist_head + 1) % MUSE_SHT31_HISTORY_POINTS;
    if (s_hist_count < MUSE_SHT31_HISTORY_POINTS) {
        s_hist_count++;
    }
}

/* ---- speech: the Pi's local TTS service (edge-tts) ----
 *
 * The text is POSTed to the tts_server on the Pi hotspot gateway
 * (POST /tts, raw UTF-8 body, MP3 streamed back), decoded with
 * minimp3, resampled to the codec's 16 kHz and written to the
 * speaker. The server takes requests from the hotspot without its
 * key (the key is blank in its env file; the hotspot is WPA2 and
 * home-only), so no credential is compiled in here.
 *
 * Everything runs on a one-shot task with a PSRAM stack: internal
 * RAM is too scarce to hold speech's buffers and stack (the same
 * lesson as the weather task), and a fetch + decode takes seconds,
 * which no caller may sit through. One utterance at a time; a
 * second one while speaking is dropped, not queued.
 */
#define SHT31_TTS_URL      "http://192.168.4.1:8020/tts"
#define SHT31_TTS_MAX_MP3  (768 * 1024)
#define SHT31_TTS_STACK    8192

static bool s_speaking;

/* Linear-interpolation resampler, the same fixed-point shape the
 * chat session uses for reply audio. */
typedef struct {
    uint32_t step;   /* Q16 input samples per output sample */
    uint32_t pos;
    int16_t prev;
} sht31_rs_t;

static void sht31_rs_init(sht31_rs_t *r, int in_rate, int out_rate)
{
    r->step = (uint32_t)(((uint64_t)in_rate << 16) / out_rate);
    r->pos = 0;
    r->prev = 0;
}

/* out must hold n * out_rate / in_rate + 2 samples. */
static size_t sht31_rs_run(sht31_rs_t *r, const int16_t *in, size_t n, int16_t *out)
{
    size_t o = 0;
    if (!n) {
        return 0;
    }
    while ((r->pos >> 16) < n) {
        size_t i = r->pos >> 16;
        int32_t a = i ? in[i - 1] : r->prev;
        int32_t b = in[i];
        out[o++] = (int16_t)(a + (int32_t)(((int64_t)(b - a) * (int64_t)(r->pos & 0xffff)) >> 16));
        r->pos += r->step;
    }
    r->pos -= (uint32_t)n << 16;
    r->prev = in[n - 1];
    return o;
}

static void sht31_speak_task(void *arg)
{
    char *text = arg;
    esp_http_client_handle_t c = NULL;
    uint8_t *mp3 = NULL;
    mp3dec_t *dec = NULL;
    int16_t *pcm = NULL, *pcm16 = NULL;
    size_t len = 0;

    esp_http_client_config_t cfg = {
        .url = SHT31_TTS_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 20000,
    };
    c = esp_http_client_init(&cfg);
    if (!c) {
        goto out;
    }
    esp_http_client_set_header(c, "Content-Type", "text/plain; charset=utf-8");
    int tlen = (int)strlen(text);
    if (esp_http_client_open(c, tlen) != ESP_OK ||
        esp_http_client_write(c, text, tlen) != tlen ||
        esp_http_client_fetch_headers(c) < 0 ||
        esp_http_client_get_status_code(c) != 200) {
        ESP_LOGW(TAG, "speak: TTS request failed (HTTP %d)",
                 esp_http_client_get_status_code(c));
        goto out;
    }
    size_t cap = 32 * 1024;
    mp3 = heap_caps_malloc(cap, MALLOC_CAP_SPIRAM);
    if (!mp3) {
        goto out;
    }
    for (;;) {
        if (len == cap) {
            if (cap >= SHT31_TTS_MAX_MP3) {
                break;   /* absurdly long; play what we have */
            }
            size_t ncap = cap * 2;
            if (ncap > SHT31_TTS_MAX_MP3) {
                ncap = SHT31_TTS_MAX_MP3;
            }
            uint8_t *nb = heap_caps_realloc(mp3, ncap, MALLOC_CAP_SPIRAM);
            if (!nb) {
                break;
            }
            mp3 = nb;
            cap = ncap;
        }
        int n = esp_http_client_read(c, (char *)mp3 + len, cap - len);
        if (n < 0) {
            ESP_LOGW(TAG, "speak: TTS read failed");
            len = 0;
            goto out;
        }
        if (n == 0) {
            break;
        }
        len += (size_t)n;
    }
    esp_http_client_cleanup(c);
    c = NULL;
    if (!len) {
        goto out;
    }

    dec = heap_caps_malloc(sizeof(mp3dec_t), MALLOC_CAP_SPIRAM);
    pcm = heap_caps_malloc(MINIMP3_MAX_SAMPLES_PER_FRAME * sizeof(int16_t),
                           MALLOC_CAP_SPIRAM);
    pcm16 = heap_caps_malloc((MINIMP3_MAX_SAMPLES_PER_FRAME + 64) * sizeof(int16_t),
                             MALLOC_CAP_SPIRAM);
    if (!dec || !pcm || !pcm16) {
        goto out;
    }
    mp3dec_init(dec);
    sht31_rs_t rs = {0, 0, 0};
    int rs_rate = -1;   /* forces sht31_rs_init on the first frame */
    size_t off = 0;
    while (off < len) {
        mp3dec_frame_info_t info;
        int samples = mp3dec_decode_frame(dec, mp3 + off, (int)(len - off), pcm, &info);
        if (!info.frame_bytes) {
            break;
        }
        off += (size_t)info.frame_bytes;
        if (!samples) {
            continue;
        }
        if (info.channels == 2) {
            for (int k = 0; k < samples; k++) {
                pcm[k] = (int16_t)((pcm[2 * k] + pcm[2 * k + 1]) / 2);
            }
        }
        if (rs_rate != info.hz) {
            sht31_rs_init(&rs, info.hz, MUSE_AUDIO_RATE);
            rs_rate = info.hz;
        }
        size_t n16 = sht31_rs_run(&rs, pcm, (size_t)samples, pcm16);
        if (n16 && muse_audio_write(pcm16, n16) != ESP_OK) {
            break;
        }
    }
    ESP_LOGI(TAG, "speak: played %u bytes of MP3", (unsigned)len);

out:
    if (c) {
        esp_http_client_cleanup(c);
    }
    heap_caps_free(mp3);
    heap_caps_free(dec);
    heap_caps_free(pcm);
    heap_caps_free(pcm16);
    free(text);
    s_speaking = false;
    vTaskDelete(NULL);
}

bool muse_sht31_speak(const char *text)
{
    if (!text || !text[0]) {
        return false;
    }
    if (s_speaking) {
        ESP_LOGI(TAG, "speak: busy, dropping: %s", text);
        return false;
    }
    char *copy = strdup(text);
    if (!copy) {
        return false;
    }
    s_speaking = true;
    if (xTaskCreatePinnedToCoreWithCaps(sht31_speak_task, "sht31_speak", SHT31_TTS_STACK,
                                        copy, 4, NULL, tskNO_AFFINITY,
                                        MALLOC_CAP_SPIRAM) != pdPASS) {
        s_speaking = false;
        free(copy);
        return false;
    }
    return true;
}

/* One-shot task so speaking can never stall the sampler. */
static void sht31_alert_task(void *arg)
{
    char *text = arg;
    muse_sht31_speak(text);
    free(text);
    vTaskDelete(NULL);
}

static void sht31_fire_alert(const char *fmt, float value)
{
    char buf[96];
    snprintf(buf, sizeof(buf), fmt, value);
    char *copy = strdup(buf);
    if (!copy || xTaskCreatePinnedToCoreWithCaps(sht31_alert_task, "sht31_alert", 4096, copy, 5, NULL, tskNO_AFFINITY, MALLOC_CAP_SPIRAM) != pdPASS) {
        free(copy);
        ESP_LOGW(TAG, "alert dropped: %s", buf);
        return;
    }
    ESP_LOGI(TAG, "alert: %s", buf);
}

static void sht31_check_alerts(float temp_c)
{
    int64_t now = esp_timer_get_time();
    if (temp_c <= SHT31_ALERT_HIGH_C - SHT31_ALERT_HYSTERESIS_C) {
        s_alert_high_armed = true;
    }
    if (temp_c >= SHT31_ALERT_LOW_C + SHT31_ALERT_HYSTERESIS_C) {
        s_alert_low_armed = true;
    }
    if (temp_c >= SHT31_ALERT_HIGH_C && s_alert_high_armed &&
        now - s_alert_high_last_us >= SHT31_ALERT_COOLDOWN_US) {
        s_alert_high_armed = false;
        s_alert_high_last_us = now;
        sht31_fire_alert("温度已经 %.1f 度了，注意防暑降温", temp_c);
    } else if (temp_c <= SHT31_ALERT_LOW_C && s_alert_low_armed &&
               now - s_alert_low_last_us >= SHT31_ALERT_COOLDOWN_US) {
        s_alert_low_armed = false;
        s_alert_low_last_us = now;
        sht31_fire_alert("温度已经降到 %.1f 度了，注意保暖", temp_c);
    }
}

static bool sht31_sd_mount(void)
{
    /* The card is shared with the camera's photos; muse_sd owns the mount. */
    if (muse_sd_mount() != ESP_OK) {
        return false;
    }
    mkdir(SHT31_SD_DIR, 0775);
    ESP_LOGI(TAG, "SD card mounted, logging to %s", SHT31_SD_DIR);
    return true;
}

static void sht31_sd_unmount(void)
{
    muse_sd_unmount();
    s_sd_mounted = false;
}

/* "Local" broken-down time for a wall stamp: fixed offset, no TZ games. */
static void sht31_local_tm(int64_t wall_ts, struct tm *out)
{
    time_t local = (time_t)wall_ts + SHT31_LOCAL_UTC_OFFSET_S;
    gmtime_r(&local, out);
}

/* Append one sample row, mounting/retrying as needed. Never blocks the
 * sampler for long: one open/write/close of a few dozen bytes, and mount
 * attempts are minutes apart. Any failure unmounts and waits a cycle. */
static void sht31_sd_log(float temp_c, float rh, int64_t ts, bool wall)
{
    int64_t now_us = esp_timer_get_time();
    if (!s_sd_mounted) {
        if (now_us < s_sd_next_try_us) {
            return;
        }
        s_sd_next_try_us = now_us + SHT31_SD_RETRY_US;
        s_sd_mounted = sht31_sd_mount();
        if (!s_sd_mounted) {
            return;
        }
    }

    char path[64], stamp[96];
    if (wall) {
        struct tm tmv;
        sht31_local_tm(ts, &tmv);
        /* strftime, not snprintf of the tm fields: the compiler cannot
         * bound a struct tm's ints, so -Wformat-truncation (an error
         * here) fires on the snprintf form even though the real output
         * always fits. The strings produced are identical. */
        snprintf(path, sizeof(path), SHT31_SD_DIR "/");
        size_t plen = strlen(path);
        strftime(path + plen, sizeof(path) - plen, "%Y-%m-%d.csv", &tmv);
        strftime(stamp, sizeof(stamp), "%Y-%m-%dT%H:%M:%S", &tmv);
    } else {
        snprintf(path, sizeof(path), SHT31_SD_DIR "/no-time.csv");
        snprintf(stamp, sizeof(stamp), "%lld", (long long)ts);
    }

    struct stat st;
    bool need_header = stat(path, &st) != 0 || st.st_size == 0;
    FILE *f = fopen(path, "a");
    if (!f) {
        ESP_LOGW(TAG, "SD open failed: %s", path);
        sht31_sd_unmount();
        s_sd_next_try_us = now_us + SHT31_SD_RETRY_US;
        return;
    }
    bool ok = true;
    if (need_header) {
        ok = fprintf(f, wall ? "# sht31 log; local time UTC+8, ISO 8601\n"
                               "time,temp_c,rh_pct\n"
                             : "# sht31 log; time is seconds since boot "
                               "(no wall clock)\ntime,temp_c,rh_pct\n") >= 0;
    }
    ok = ok && fprintf(f, "%s,%.1f,%.1f\n", stamp, temp_c, rh) >= 0;
    if (fclose(f) != 0) {
        ok = false;
    }
    if (!ok) {
        ESP_LOGW(TAG, "SD write failed: %s", path);
        sht31_sd_unmount();
        s_sd_next_try_us = now_us + SHT31_SD_RETRY_US;
        return;
    }
    static bool hwm_logged;
    if (!hwm_logged) {
        hwm_logged = true;
        ESP_LOGI(TAG, "sampler stack high water: %u bytes free",
                 (unsigned)uxTaskGetStackHighWaterMark(NULL));
    }
}

static void sht31_task(void *arg)
{
    (void)arg;
    int failures = 0;
    for (;;) {
        float temp_c, rh;
        esp_err_t err = sht31_sample(&temp_c, &rh);
        if (err == ESP_OK) {
            if (failures) {
                ESP_LOGI(TAG, "reading again after %d failed sample(s)", failures);
            }
            failures = 0;
            int64_t ts;
            bool wall;
            sht31_stamp(&ts, &wall);
            portENTER_CRITICAL(&s_lock);
            s_temp_c = temp_c;
            s_rh = rh;
            s_valid = true;
            s_last_ok_us = esp_timer_get_time();
            /* Fold the sample into the open 5-minute history bucket. */
            s_acc_t += temp_c;
            s_acc_rh += rh;
            if (++s_acc_n >= SHT31_SAMPLES_PER_POINT) {
                sht31_hist_push_locked(s_acc_t / s_acc_n, s_acc_rh / s_acc_n, ts, wall);
                s_acc_t = s_acc_rh = 0;
                s_acc_n = 0;
            }
            portEXIT_CRITICAL(&s_lock);
            ESP_LOGI(TAG, "%.1f C, %.1f %%RH", temp_c, rh);
            sht31_check_alerts(temp_c);
            sht31_sd_log(temp_c, rh, ts, wall);
        } else {
            failures++;
            if (failures == SHT31_STALE_AFTER) {
                portENTER_CRITICAL(&s_lock);
                s_valid = false;
                portEXIT_CRITICAL(&s_lock);
            }
            /* The first failure, then one line per 20: a sensor unplugged
             * for a day must not fill the log. */
            if (failures == 1 || failures % 20 == 0) {
                ESP_LOGW(TAG, "sample failed (%d in a row): %s", failures, esp_err_to_name(err));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(SHT31_PERIOD_MS));
    }
}

esp_err_t muse_sht31_start(i2c_master_bus_handle_t bus)
{
    if (s_dev) {
        return ESP_OK;   /* already running */
    }
    uint8_t addr = SHT31_ADDR_DEFAULT;
    if (i2c_master_probe(bus, addr, SHT31_XFER_TIMEOUT_MS) != ESP_OK) {
        addr = SHT31_ADDR_ALT;
        if (i2c_master_probe(bus, addr, SHT31_XFER_TIMEOUT_MS) != ESP_OK) {
            ESP_LOGW(TAG, "no SHT31 at 0x%02x or 0x%02x", SHT31_ADDR_DEFAULT, SHT31_ADDR_ALT);
            return ESP_ERR_NOT_FOUND;
        }
    }
    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 100000,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "add device: %s", esp_err_to_name(err));
        return err;
    }
    s_present = true;

    if (!s_hist) {
        s_hist = heap_caps_malloc(sizeof(sht31_point_t) * MUSE_SHT31_HISTORY_POINTS,
                                  MALLOC_CAP_SPIRAM);
        if (!s_hist) {
            ESP_LOGW(TAG, "history ring allocation failed; history disabled");
        }
    }

    /* Soft reset (0x30A2), so a module left mid-measurement by an earlier
     * run starts clean. Best effort: the measurements below still work. */
    const uint8_t reset[] = { 0x30, 0xA2 };
    err = i2c_master_transmit(s_dev, reset, sizeof(reset), SHT31_XFER_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "soft reset: %s", esp_err_to_name(err));
    } else {
        vTaskDelay(pdMS_TO_TICKS(2));   /* the reset takes up to 1.5 ms */
    }

    /* 6144, not the phase-1 3072: the sampler also mounts FAT and does
     * stdio on the SD card, which runs deep. (8192 in the first phase-2
     * build, together with the then-static history ring, squeezed internal
     * DRAM enough to break unrelated task creation; the sampler logs its
     * stack high-water mark after the first SD write so this can be
     * trimmed further with evidence.) */
    if (xTaskCreatePinnedToCoreWithCaps(sht31_task, "sht31", 6144, NULL, 5, NULL, tskNO_AFFINITY, MALLOC_CAP_SPIRAM) != pdPASS) {
        ESP_LOGE(TAG, "sampler task failed to start");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "SHT31 at 0x%02x, sampling every %d s", addr, SHT31_PERIOD_MS / 1000);
    return ESP_OK;
}

bool muse_sht31_present(void)
{
    return s_present;
}

bool muse_sht31_get(float *temp_c, float *rh)
{
    return muse_sht31_get_ex(temp_c, rh, NULL);
}

bool muse_sht31_get_ex(float *temp_c, float *rh, uint32_t *age_s)
{
    portENTER_CRITICAL(&s_lock);
    bool valid = s_valid;
    float t = s_temp_c, h = s_rh;
    int64_t at_us = s_last_ok_us;
    portEXIT_CRITICAL(&s_lock);
    if (!valid) {
        return false;
    }
    if (temp_c) {
        *temp_c = t;
    }
    if (rh) {
        *rh = h;
    }
    if (age_s) {
        int64_t age_us = esp_timer_get_time() - at_us;
        *age_s = (uint32_t)(age_us > 0 ? age_us / 1000000 : 0);
    }
    return true;
}

int muse_sht31_history_count(void)
{
    portENTER_CRITICAL(&s_lock);
    int n = s_hist_count;
    portEXIT_CRITICAL(&s_lock);
    return n;
}

/* Caller holds s_lock. idx 0 is the oldest point held. */
static const sht31_point_t *sht31_hist_at_locked(int idx)
{
    int start = (s_hist_head - s_hist_count + MUSE_SHT31_HISTORY_POINTS) %
                MUSE_SHT31_HISTORY_POINTS;
    return &s_hist[(start + idx) % MUSE_SHT31_HISTORY_POINTS];
}

bool muse_sht31_history_point(int idx, float *temp_c, float *rh,
                              int64_t *ts, bool *ts_is_wall)
{
    portENTER_CRITICAL(&s_lock);
    if (idx < 0 || idx >= s_hist_count) {
        portEXIT_CRITICAL(&s_lock);
        return false;
    }
    sht31_point_t p = *sht31_hist_at_locked(idx);
    portEXIT_CRITICAL(&s_lock);
    if (temp_c) {
        *temp_c = p.temp_c;
    }
    if (rh) {
        *rh = p.rh;
    }
    if (ts) {
        *ts = p.ts;
    }
    if (ts_is_wall) {
        *ts_is_wall = p.wall;
    }
    return true;
}

int muse_sht31_history_import(const int64_t *ts, const float *temps,
                              const float *rhs, int n)
{
    if (!ts || !temps || !rhs || n <= 0) {
        return 0;
    }
    portENTER_CRITICAL(&s_lock);
    if (!s_hist || s_hist_count > 0) {
        portEXIT_CRITICAL(&s_lock);
        return 0;   /* no ring, or live data already: never clobber */
    }
    s_hist_head = 0;
    s_acc_t = s_acc_rh = 0;
    s_acc_n = 0;
    int taken = 0;
    for (int i = 0; i < n; i++) {
        sht31_hist_push_locked(temps[i], rhs[i], ts[i], true);
        taken++;
    }
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGI(TAG, "history imported: %d points", taken);
    return taken;
}

bool muse_sht31_history_extremes(float *min_c, float *max_c,
                                 int64_t *min_ts, int64_t *max_ts,
                                 bool *ts_is_wall)
{
    portENTER_CRITICAL(&s_lock);
    if (s_hist_count == 0) {
        portEXIT_CRITICAL(&s_lock);
        return false;
    }
    sht31_point_t mn = *sht31_hist_at_locked(0);
    sht31_point_t mx = mn;
    for (int i = 1; i < s_hist_count; i++) {
        const sht31_point_t *p = sht31_hist_at_locked(i);
        if (p->temp_c < mn.temp_c) {
            mn = *p;
        }
        if (p->temp_c > mx.temp_c) {
            mx = *p;
        }
    }
    portEXIT_CRITICAL(&s_lock);
    if (min_c) {
        *min_c = mn.temp_c;
    }
    if (max_c) {
        *max_c = mx.temp_c;
    }
    if (min_ts) {
        *min_ts = mn.ts;
    }
    if (max_ts) {
        *max_ts = mx.ts;
    }
    if (ts_is_wall) {
        *ts_is_wall = mn.wall && mx.wall;
    }
    return true;
}
