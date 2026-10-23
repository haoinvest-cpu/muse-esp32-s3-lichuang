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
 * Voice wake for the Lichuang SZP, built on Espressif's esp-sr AFE
 * (WakeNet only; end-of-turn from the fetch frame energy). See
 * muse_wake.h.
 *
 * The voice pipeline owns the microphone: muse_voice is the only task
 * that reads it. So this module never reads audio itself -- muse_voice
 * taps its chunks in (idle chunks for wake detection, record chunks for
 * end-of-turn), and the wake task below feeds them to the AFE. Feeding is
 * phase-gated by the state machine, so Muse's own replies (never
 * tapped in) and button-started turns (record chunks dropped while
 * LISTENING) cannot trigger or extend a wake turn.
 *
 * Memory: the ring, the feed staging buffer and this task's stack live
 * in PSRAM; the AFE is asked to prefer PSRAM by its own defaults. The
 * free-internal-RAM lines logged at start and at AFE creation are the
 * checkpoints to compare against muse_app's "ready: free heap" line.
 * The AFE is created lazily when the setting is first switched on and
 * destroyed when it is switched off, so the default (off) costs only
 * the ring and the parked task.
 */
#include "muse_wake.h"

#include "esp_err.h"
#include "sdkconfig.h"

#if CONFIG_MUSE_BOARD_LICHUANG_SZP

#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_afe_config.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h" /* esp_afe_handle_from_config lives here, not in the iface header */
#include "model_path.h"

#include "muse_input.h"
#include "muse_mem.h"
#include "muse_settings.h"

static const char *TAG = "muse_wake";

#define WAKE_RING_FRAMES 16000   /* 1 s of mono 16 kHz audio, PSRAM */
#define WAKE_REARM_MS 300        /* ignore wake hits this long after re-arming */
#define WAKE_SETTLE_MAX_MS 6000  /* backstop: listen again even if idle never taps */
#define WAKE_SPEECH_DBFS (-45.0f)  /* fetch data_volume at/above this = speech */
/* Optional esp-sr tuning fields (vad_mode, vad_min_noise_ms,
 * memory_alloc_mode, afe_perferred_core/priority in esp_afe_config.h)
 * are deliberately left at afe_config_init() defaults for W1: the
 * end-of-turn timing lives in this file (MUSE_WAKE_SILENCE_MS), not
 * in the AFE's VAD hangover. Tune them only against the resolved
 * esp-sr version's esp_afe_config.h. */

typedef enum {
    WAKE_OFF,        /* setting off, or AFE not built yet */
    WAKE_LISTENING,  /* idle audio goes to the AFE, watching for the wake word */
    WAKE_IN_TURN,    /* we posted PTT_DOWN; record audio goes to the energy check */
    WAKE_SETTLING,   /* we posted PTT_UP; wait for the pipeline to go idle */
} wake_state_t;

static volatile wake_state_t s_state = WAKE_OFF;
static volatile bool s_voice_idle;   /* feed_idle tapped while IN_TURN/SETTLING */
static volatile bool s_pipeline_alive;   /* feed_idle called at least once */

static int16_t *s_ring;
static volatile uint32_t s_ring_head;   /* frames in (producer: muse_voice) */
static volatile uint32_t s_ring_tail;   /* frames out (consumer: wake task) */
static uint32_t s_ring_dropped;

static srmodel_list_t *s_models;
static const esp_afe_sr_iface_t *s_afe_handle;
static esp_afe_sr_data_t *s_afe;
static int16_t *s_feed_buf;
static int s_feed_chunksize;

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

/* Single producer (the voice task), single consumer (the wake task):
 * plain head/tail counters are enough, no lock. A full ring drops the
 * incoming chunk -- detection prefers fresh audio over old audio. */
static void ring_push(const int16_t *pcm, size_t frames)
{
    uint32_t head = s_ring_head;
    if (head - s_ring_tail + frames > WAKE_RING_FRAMES) {
        s_ring_dropped += frames;
        return;
    }
    for (size_t i = 0; i < frames; i++) {
        s_ring[(head + i) % WAKE_RING_FRAMES] = pcm[i];
    }
    s_ring_head = head + frames;
}

static size_t ring_avail(void)
{
    return s_ring_head - s_ring_tail;
}

static void ring_pop(int16_t *out, size_t frames)
{
    uint32_t tail = s_ring_tail;
    for (size_t i = 0; i < frames; i++) {
        out[i] = s_ring[(tail + i) % WAKE_RING_FRAMES];
    }
    s_ring_tail = tail + frames;
}

void muse_wake_feed_idle(const int16_t *pcm, size_t frames)
{
    s_pipeline_alive = true;   /* the voice pipeline is reading the mic */
    if (s_state == WAKE_IN_TURN || s_state == WAKE_SETTLING) {
        s_voice_idle = true;   /* the pipeline is back in its idle loop */
        return;
    }
    if (s_state != WAKE_LISTENING) {
        return;
    }
    ring_push(pcm, frames);
}

void muse_wake_feed_record(const int16_t *pcm, size_t frames)
{
    if (s_state != WAKE_IN_TURN) {
        return;   /* a button-started turn: not ours to end */
    }
    ring_push(pcm, frames);
}

static bool afe_setup(void)
{
    unsigned before = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    if (!s_models) {
        s_models = esp_srmodel_init("model");
        if (!s_models) {
            ESP_LOGE(TAG, "no models found: is the \"model\" partition flashed?");
            return false;
        }
        for (int i = 0; i < s_models->num; i++) {
            ESP_LOGI(TAG, "model: %s", s_models->model_name[i]);
        }
    }
    /* "M": the one mono channel muse_audio_read hands out. No AEC (no
     * playback reference channel exists) and no SE/NS: WakeNet runs on
     * the raw channel in LOW_COST mode. */
    afe_config_t *cfg = afe_config_init("M", s_models, AFE_TYPE_SR, AFE_MODE_LOW_COST);
    if (!cfg) {
        ESP_LOGE(TAG, "afe_config_init failed");
        return false;
    }
    cfg->aec_init = false;
    cfg->se_init = false;
    cfg->ns_init = false;
    cfg->vad_init = false;
    cfg->wakenet_init = true;
    /* No AFE VAD: the probe (boot log 2026-10-23) priced a WakeNet-only
     * AFE at 13,844 B of internal RAM against 37,820 B with the VAD
     * also built, and this board's steady state has almost none spare.
     * End-of-turn uses the fetch result's data_volume instead (it is
     * reported with or without VadNet). AFE creation with the default
     * alloc mode also ate internal RAM by the tens of KB, which starved
     * the later I2S DMA allocation and killed audio init. Push the
     * AFE's buffers to PSRAM; internal RAM is the scarce resource on
     * this board, PSRAM is not. */
    cfg->memory_alloc_mode = AFE_MEMORY_ALLOC_MORE_PSRAM;
    /* The AFE logs a ringbuffer-empty warning on every fetch deadline
     * (~10/s) whenever the voice pipeline is not feeding it (playback,
     * connecting); that spam buries every other log line. Errors only. */
    esp_log_level_set("AFE", ESP_LOG_ERROR);
    s_afe_handle = esp_afe_handle_from_config(cfg);
    s_afe = s_afe_handle ? s_afe_handle->create_from_config(cfg) : NULL;
    afe_config_free(cfg);
    if (!s_afe) {
        ESP_LOGE(TAG, "AFE creation failed");
        return false;
    }
    s_feed_chunksize = s_afe_handle->get_feed_chunksize(s_afe);
    int nch = s_afe_handle->get_feed_channel_num(s_afe);
    s_feed_buf = heap_caps_malloc((size_t)s_feed_chunksize * sizeof(int16_t), MUSE_BIG_CAPS);
    if (nch != 1 || s_feed_chunksize <= 0 || !s_feed_buf) {
        ESP_LOGE(TAG, "unexpected AFE feed format: %d frames x %d ch", s_feed_chunksize, nch);
        s_afe_handle->destroy(s_afe);
        s_afe = NULL;
        free(s_feed_buf);
        s_feed_buf = NULL;
        return false;
    }
    ESP_LOGI(TAG, "AFE up: feed %d frames x %d ch, free internal %u -> %u",
             s_feed_chunksize, nch, before, heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    return true;
}

static void afe_teardown(void)
{
    if (s_afe) {
        s_afe_handle->destroy(s_afe);
        s_afe = NULL;
    }
    free(s_feed_buf);
    s_feed_buf = NULL;
    /* s_models stays loaded: re-enabling is cheap, and the model
     * partition mapping is not ours to churn. */
    ESP_LOGI(TAG, "AFE down, free internal %u", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
}

static void wake_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "wake task started, free internal %u (baseline, AFE not built)",
             heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    int64_t turn_start = 0, last_speech = 0, settle_start = 0, rearm_until = 0;
    bool heard = false;
    float turn_peak_dbfs = -100.0f;
    for (;;) {
        if (!muse_settings_wake_on()) {
            if (s_afe) {
                if (s_state == WAKE_IN_TURN) {
                    muse_input_post_ptt(MUSE_PTT_UP, false);   /* don't strand a recording */
                }
                afe_teardown();
            }
            s_state = WAKE_OFF;
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }
        if (!s_afe) {
            /* Build the AFE only once the audio pipeline is alive and
             * the boot storm has passed. AFE creation takes ~14K of
             * internal RAM; building it at boot starved audio init
             * (I2S DMA) and the pairing timers — and an AFE with no
             * mic feed is useless anyway. */
            if (!s_pipeline_alive ||
                heap_caps_get_free_size(MALLOC_CAP_INTERNAL) < 20000) {
                static bool s_gate_logged;
                if (!s_gate_logged) {
                    s_gate_logged = true;
                    ESP_LOGI(TAG, "deferring AFE until the audio pipeline is up");
                }
                vTaskDelay(pdMS_TO_TICKS(250));
                continue;
            }
            if (afe_setup()) {
                s_state = WAKE_LISTENING;
                rearm_until = now_ms() + WAKE_REARM_MS;
                ESP_LOGI(TAG, "listening for the wake word");
            } else {
                vTaskDelay(pdMS_TO_TICKS(5000));   /* stay off, retry later */
            }
            continue;
        }

        while (ring_avail() >= (size_t)s_feed_chunksize) {
            if (s_state == WAKE_IN_TURN) {
                /* Record audio is flowing, so the posted press landed
                 * and the pipeline IS recording: any idle flag is a
                 * stale tap from the transition, not a refusal. */
                s_voice_idle = false;
            }
            ring_pop(s_feed_buf, s_feed_chunksize);
            s_afe_handle->feed(s_afe, s_feed_buf);
        }
        afe_fetch_result_t *res = s_afe_handle->fetch_with_delay(s_afe, pdMS_TO_TICKS(100));
        int64_t now = now_ms();
        if (res && res->ret_value == ESP_OK) {
            if (s_state == WAKE_LISTENING) {
                if (now >= rearm_until &&
                    (res->wakeup_state == WAKENET_DETECTED ||
                     res->wakeup_state == WAKENET_CHANNEL_VERIFIED)) {
                    ESP_LOGI(TAG, "wake word heard: starting a turn");
                    muse_input_post_ptt(MUSE_PTT_DOWN, false);
                    s_state = WAKE_IN_TURN;
                    turn_start = last_speech = now;
                    heard = false;
                    turn_peak_dbfs = -100.0f;
                    s_voice_idle = false;
                }
            } else if (s_state == WAKE_IN_TURN) {
                /* Speech = a loud enough fetch frame. data_volume is
                 * the frame's mean-square energy in dBFS, reported with
                 * or without the AFE's own VAD (not built here: it
                 * costs ~14 KB of internal RAM). At this board's 36 dB
                 * mic gain the quiet-room floor sits near -52 dBFS and
                 * close speech near -25 dBFS. */
                if (res->data_volume > turn_peak_dbfs) {
                    turn_peak_dbfs = res->data_volume;
                }
                if (res->data_volume >= WAKE_SPEECH_DBFS) {
                    heard = true;
                    last_speech = now;
                }
            }
        }

        if (s_state == WAKE_IN_TURN) {
            const char *why = NULL;
            bool post_up = true;
            if (s_voice_idle && now - turn_start >= 1000) {
                /* Idle with no record audio a full second after the
                 * press: the press really was refused. (Inside the
                 * first second the flag is unreliable: the pipeline
                 * taps idle once more before it starts recording, and
                 * acting on that killed live turns at ~100 ms, leaving
                 * the recording to run to its 15 s cap.) */
                why = "pipeline already idle";   /* e.g. the press was refused */
                post_up = false;
            } else if (heard && now - last_speech >= MUSE_WAKE_SILENCE_MS) {
                why = "silence";
            } else if (!heard && now - turn_start >= MUSE_WAKE_NOSPEECH_MS) {
                why = "no speech";
            } else if (now - turn_start >= MUSE_WAKE_TURN_MAX_MS) {
                why = "time cap";
            }
            if (why) {
                ESP_LOGI(TAG, "turn over (%s, peak %.1f dBFS)", why, turn_peak_dbfs);
                if (post_up) {
                    muse_input_post_ptt(MUSE_PTT_UP, false);
                }
                s_state = WAKE_SETTLING;
                settle_start = now;
                s_voice_idle = false;
            }
        } else if (s_state == WAKE_SETTLING) {
            if ((s_voice_idle && now - settle_start >= 1000) ||
                now - settle_start >= WAKE_SETTLE_MAX_MS) {
                s_state = WAKE_LISTENING;
                rearm_until = now + WAKE_REARM_MS;
                ESP_LOGI(TAG, "listening again (dropped %u frames so far)",
                         (unsigned)s_ring_dropped);
            }
        }
    }
}

esp_err_t muse_wake_start(void)
{
    static bool started;
    if (started) {
        return ESP_OK;
    }
    s_ring = heap_caps_malloc(WAKE_RING_FRAMES * sizeof(int16_t), MUSE_BIG_CAPS);
    if (!s_ring) {
        return ESP_ERR_NO_MEM;
    }
    /* 8192 stack in INTERNAL RAM, deliberately: wake_task calls
     * esp_srmodel_init(), which mmaps the model partition, and mmap
     * freezes the cache -- a task whose stack lives in PSRAM cannot
     * survive that (IDF asserts in s_task_stack_is_sane_when_cache_
     * frozen). Learned the hard way 2026-10-22: PSRAM stack here =
     * assert + reboot loop the moment wake is enabled. The sample
     * ring and the AFE's own buffers stay in PSRAM. */
    if (xTaskCreatePinnedToCore(wake_task, "muse_wake", 8192, NULL, 5, NULL,
                                tskNO_AFFINITY) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    started = true;
    return ESP_OK;
}

#else   /* not the Lichuang SZP: voice wake is not built */

void muse_wake_feed_idle(const int16_t *pcm, size_t frames)
{
    (void)pcm;
    (void)frames;
}

void muse_wake_feed_record(const int16_t *pcm, size_t frames)
{
    (void)pcm;
    (void)frames;
}

esp_err_t muse_wake_start(void)
{
    return ESP_ERR_NOT_SUPPORTED;
}

#endif
