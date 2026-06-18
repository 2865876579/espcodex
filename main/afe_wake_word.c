#include "afe_wake_word.h"
#include "audio_out.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_wn_models.h"
#include "model_path.h"
#include "driver/i2s_std.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "afe_wake";

// AFE v2.x: handle from esp_afe_handle_from_config() + data from create_from_config()
static const esp_afe_sr_iface_t *s_afe_handle = NULL;
static esp_afe_sr_data_t        *s_afe_data   = NULL;

// 唤醒词回调
static wake_word_callback_t s_wake_cb = NULL;

// 录音采集缓存 (PSRAM)
// ★ 借鉴 xiaozhi：用 spinlock 保护 capture buffer 的并发访问
static int16_t *s_capture_buf   = NULL;
static volatile int  s_capture_max   = 0;
static volatile int  s_capture_idx   = 0;
static volatile bool s_capture_done  = false;
static volatile bool s_capture_seen_speech = false;
static volatile bool s_capture_vad_speech = false;
static volatile int  s_capture_vad_speech_samples = 0;
static volatile int  s_capture_last_voice_idx = 0;
static portMUX_TYPE s_capture_lock = portMUX_INITIALIZER_UNLOCKED;

#define CAPTURE_SAMPLE_RATE          16000
#define CAPTURE_NO_SPEECH_MS         1500
#define CAPTURE_END_SILENCE_MS       500
#define CAPTURE_MIN_VAD_SPEECH_MS    300
#define CAPTURE_AC_AVG_THRESHOLD     350
#define CAPTURE_PEAK_THRESHOLD       2500
#define CAPTURE_ACTIVE_LEVEL         1000
// 防重复触发冷却 (fetch 周期数)
#define COOLDOWN_TICKS  150
#define AFE_INTERNAL_PRIORITY 1
#define AFE_FEED_TASK_PRIORITY 8
#define AFE_FETCH_TASK_PRIORITY 3
#define AFE_FEED_TASK_STACK_BYTES (2048 * 3)
#define AFE_FETCH_TASK_STACK_BYTES 8192
#define AFE_FEED_TASK_CORE 0
#define AFE_INTERNAL_CORE 1
static volatile int s_cooldown = 0;

// I2S1 RX 句柄
static i2s_chan_handle_t s_rx_chan = NULL;

// feed/fetch 参数
static int s_feed_chunksize  = 0;
static int s_feed_channels   = 0;
static int s_fetch_chunksize = 0;

static bool capture_chunk_has_voice(const int16_t *data, int samples)
{
    if (!data || samples <= 0) {
        return false;
    }

    int64_t sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += data[i];
    }
    int dc = (int)(sum / samples);

    int64_t ac_sum = 0;
    int peak = 0;
    int active = 0;
    for (int i = 0; i < samples; i++) {
        int delta = (int)data[i] - dc;
        int a = delta >= 0 ? delta : -delta;
        ac_sum += a;
        if (a > peak) {
            peak = a;
        }
        if (a >= CAPTURE_ACTIVE_LEVEL) {
            active++;
        }
    }

    int ac_avg = (int)(ac_sum / samples);
    int min_active = samples / 20;
    if (min_active < 8) {
        min_active = 8;
    }

    return ac_avg >= CAPTURE_AC_AVG_THRESHOLD
        || (peak >= CAPTURE_PEAK_THRESHOLD && active >= min_active);
}


// ============================================================
//  Feed 任务: I2S1 DMA → 32→16 提取 → 累积 → feed AFE
//  参考小智项目：独立 feed/fetch 双任务，feed 按时序严格调用
// ============================================================
static void afe_feed_task(void *arg)
{
    int ch = s_feed_channels;
    int feed_bytes = s_feed_chunksize * ch * (int)sizeof(int16_t);

    // feed 间隔: chunksize/16000 秒
    int interval_ms = (s_feed_chunksize * 1000) / 16000;
    if (interval_ms < 10) interval_ms = 10;

    int16_t *feed_buf = heap_caps_calloc(s_feed_chunksize, ch * sizeof(int16_t),
                                          MALLOC_CAP_SPIRAM);
    if (!feed_buf) {
        ESP_LOGE(TAG, "feed malloc failed");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "feed task started, chunk=%d", s_feed_chunksize);

    // DMA 累积缓冲：每帧 511×2×4=4088 bytes (32-bit stereo I2S, 驱动限制511帧)
    const int dma_frame_num    = 511;  // 对齐 audio_out.c DMA_FRAME_NUM
    const int dma_frame_bytes  = dma_frame_num * 2 * (int)sizeof(int32_t);
    const int acc_capacity     = s_feed_chunksize + dma_frame_num;
    int32_t *acc = heap_caps_malloc(acc_capacity * 2 * sizeof(int32_t),
                                     MALLOC_CAP_SPIRAM);
    if (!acc) {
        ESP_LOGE(TAG, "acc malloc failed");
        free(feed_buf);
        vTaskDelete(NULL);
        return;
    }
    int acc_samples = 0;

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(interval_ms);
    int cycle = 0;

    while (1) {
        // 读一个 DMA 帧
        size_t bytes_read = 0;
        esp_err_t err = i2s_channel_read(s_rx_chan,
            acc + acc_samples * 2,
            dma_frame_bytes, &bytes_read,
            pdMS_TO_TICKS(interval_ms * 2));
        int samples_read = (int)(bytes_read / (2 * sizeof(int32_t)));

        if (err == ESP_OK && samples_read > 0) {
            acc_samples += samples_read;
        }

        // 累积够一帧 → 提取 mono 16-bit → feed
        if (acc_samples >= s_feed_chunksize) {
            cycle++;
            memset(feed_buf, 0, feed_bytes);
            for (int i = 0; i < s_feed_chunksize; i++) {
                // INMP441: 24-bit 左对齐在 32-bit slot → 取高 16bit
                int16_t mic = (int16_t)(acc[i * 2] >> 16);
                feed_buf[i * ch] = mic;
                if (ch >= 2) {
                    feed_buf[i * ch + 1] = mic;
                }
            }

            // 第一帧快速验证 I2S 格式（仅首次）
            if (cycle == 1) {
                int16_t mic0 = (int16_t)(acc[0] >> 16);
                ESP_LOGI(TAG, "mic working, first sample=%d", mic0);
            }

            // 剩余数据前移
            int remain = acc_samples - s_feed_chunksize;
            if (remain > 0) {
                memmove(acc, acc + s_feed_chunksize * 2,
                        remain * 2 * sizeof(int32_t));
            }
            acc_samples = remain;

            s_afe_handle->feed(s_afe_data, feed_buf);
        }
        vTaskDelayUntil(&last_wake, period);
    }
}


// ============================================================
//  Fetch 任务: fetch AFE → 唤醒检测 + 录音采集
//  参考小智项目：fetch 负责唤醒词 + 命令词检测
// ============================================================
static void afe_fetch_task(void *arg)
{
    while (1) {
        afe_fetch_result_t *res = s_afe_handle->fetch_with_delay
            ? s_afe_handle->fetch_with_delay(s_afe_data, portMAX_DELAY)
            : s_afe_handle->fetch(s_afe_data);
        if (!res || res->ret_value == ESP_FAIL) {
            vTaskDelay(1);
            continue;
        }

        // 冷却计数
        if (s_cooldown > 0) s_cooldown--;

        // 唤醒词检测
        if (res->wakeup_state == WAKENET_DETECTED
            && s_cooldown == 0) {
            ESP_LOGI(TAG, "*** WAKE WORD DETECTED! ***");
            s_cooldown = COOLDOWN_TICKS;
            // ★ 不 disable wakenet — 否则音频通路关闭，采集不到数据
            if (s_wake_cb) s_wake_cb();
        }

        // 录音采集: 从 AFE 降噪输出中拷贝，优先用 AFE VAD 做端点检测。
        // ★ 借鉴 xiaozhi：临界区保护 buffer 访问，防止主循环 free 时还在写
        portENTER_CRITICAL(&s_capture_lock);
        bool capturing = (s_capture_buf != NULL && !s_capture_done
                          && s_capture_idx < s_capture_max && res->data != NULL);
        int16_t *buf_ptr = s_capture_buf;
        int cur_idx = s_capture_idx;
        int cur_max = s_capture_max;
        portEXIT_CRITICAL(&s_capture_lock);

        if (capturing) {
            int fetch_samples = res->data_size / (int)sizeof(int16_t);
            int remain = cur_max - cur_idx;
            int to_copy = (fetch_samples < remain) ? fetch_samples : remain;
            int copy_start = cur_idx;
            memcpy(buf_ptr + cur_idx, res->data, to_copy * sizeof(int16_t));
            // safe: only fetch task writes s_capture_idx forward
            s_capture_idx = cur_idx + to_copy;

            bool vad_speech = (res->vad_state == VAD_SPEECH);
            bool energy_speech = capture_chunk_has_voice(buf_ptr + copy_start, to_copy);
            if (vad_speech) {
                s_capture_seen_speech = true;
                s_capture_vad_speech_samples += to_copy;
                if (s_capture_vad_speech_samples >=
                    (CAPTURE_SAMPLE_RATE * CAPTURE_MIN_VAD_SPEECH_MS / 1000)) {
                    s_capture_vad_speech = true;
                }
                s_capture_last_voice_idx = s_capture_idx;
            } else if (!s_capture_seen_speech && energy_speech) {
                s_capture_seen_speech = true;
                s_capture_last_voice_idx = s_capture_idx;
            }

            int no_speech_limit = CAPTURE_SAMPLE_RATE * CAPTURE_NO_SPEECH_MS / 1000;
            int end_silence_limit = CAPTURE_SAMPLE_RATE * CAPTURE_END_SILENCE_MS / 1000;
            if (s_capture_idx >= cur_max) {
                s_capture_done = true;
            } else if (!s_capture_seen_speech && s_capture_idx >= no_speech_limit) {
                s_capture_done = true;
            } else if (s_capture_seen_speech &&
                       (s_capture_idx - s_capture_last_voice_idx) >= end_silence_limit) {
                s_capture_done = true;
            }
        }
    }
}


// ============================================================
//  公开 API
// ============================================================

int afe_wake_word_init(wake_word_callback_t cb)
{
    s_wake_cb = cb;

    s_rx_chan = audio_out_get_rx_chan();
    if (!s_rx_chan) {
        ESP_LOGE(TAG, "I2S1 RX handle is NULL — call audio_out_init() first");
        return -1;
    }

    // 1. 加载模型分区
    srmodel_list_t *models = esp_srmodel_init("model");
    if (!models || models->num <= 0) {
        ESP_LOGE(TAG, "esp_srmodel_init failed — no models in 'model' partition");
        return -1;
    }
    // 2. 动态获取唤醒词模型名
    char *wn_name = esp_srmodel_filter(models, ESP_WN_PREFIX, NULL);
    if (!wn_name) {
        ESP_LOGE(TAG, "no wake word model found in partition");
        return -1;
    }

    // 3. 创建 AFE 配置（单麦 M，无参考通道）
    afe_config_t *cfg = afe_config_init("M", models,
                                        AFE_TYPE_SR, AFE_MODE_HIGH_PERF);
    if (!cfg) {
        ESP_LOGE(TAG, "afe_config_init failed");
        return -1;
    }

    // 4. 配置
    cfg->wakenet_model_name = wn_name;
    cfg->wakenet_init       = true;
    cfg->wakenet_mode       = DET_MODE_95;
    cfg->aec_init           = false;
    cfg->se_init            = false;
    cfg->ns_init            = false;
    cfg->vad_init           = true;
    cfg->vad_mode           = VAD_MODE_3;
    cfg->vad_model_name     = NULL;
    cfg->vad_min_speech_ms  = 96;
    cfg->vad_min_noise_ms   = 300;
    cfg->vad_delay_ms       = 128;
    cfg->agc_init           = false;
    cfg->afe_perferred_core     = AFE_INTERNAL_CORE;
    cfg->afe_perferred_priority = AFE_INTERNAL_PRIORITY;
    cfg->memory_alloc_mode      = AFE_MEMORY_ALLOC_MORE_PSRAM;

    // 5. 校验（调整不兼容参数）
    cfg = afe_config_check(cfg);

    // ★ 在 check 之后设 gain，防止被覆盖
    cfg->afe_perferred_core     = AFE_INTERNAL_CORE;
    cfg->afe_perferred_priority = AFE_INTERNAL_PRIORITY;
    cfg->afe_linear_gain = 1.0f;

    // 6. 获取 handle 并创建实例
    s_afe_handle = esp_afe_handle_from_config(cfg);
    if (!s_afe_handle) {
        ESP_LOGE(TAG, "esp_afe_handle_from_config failed");
        afe_config_free(cfg);
        return -1;
    }

    s_afe_data = s_afe_handle->create_from_config(cfg);
    if (!s_afe_data) {
        ESP_LOGE(TAG, "create_from_config failed");
        afe_config_free(cfg);
        return -1;
    }

    // 7. 降到最低阈值（提高灵敏度）
    s_afe_handle->set_wakenet_threshold(s_afe_data, 1, 0.1f);

    // 8. 查询 feed/fetch 参数
    s_feed_channels   = s_afe_handle->get_channel_num(s_afe_data);
    s_feed_chunksize  = s_afe_handle->get_feed_chunksize(s_afe_data);
    s_fetch_chunksize = s_afe_handle->get_fetch_chunksize(s_afe_data);
    int samp_rate     = s_afe_handle->get_samp_rate(s_afe_data);

    ESP_LOGI(TAG, "AFE init OK: feed=%d(ch=%d) fetch=%d rate=%d",
             s_feed_chunksize, s_feed_channels, s_fetch_chunksize, samp_rate);

    // 9. 释放配置（不再需要）
    afe_config_free(cfg);

    // Start fetch before feed so AFE has a reader before microphone frames arrive.
    BaseType_t fetch_ret = xTaskCreate(afe_fetch_task, "afe_fetch",
                                       AFE_FETCH_TASK_STACK_BYTES, NULL,
                                       AFE_FETCH_TASK_PRIORITY, NULL);
    if (fetch_ret != pdPASS) {
        ESP_LOGE(TAG, "afe_fetch task create failed: %ld", (long)fetch_ret);
        return -1;
    }

    BaseType_t feed_ret = xTaskCreatePinnedToCore(afe_feed_task, "afe_feed",
                                                 AFE_FEED_TASK_STACK_BYTES, NULL,
                                                 AFE_FEED_TASK_PRIORITY, NULL,
                                                 AFE_FEED_TASK_CORE);
    if (feed_ret != pdPASS) {
        ESP_LOGE(TAG, "afe_feed task create failed: %ld", (long)feed_ret);
        return -1;
    }

    ESP_LOGI(TAG, "AFE pipeline started, listening...");
    return 0;
}


void afe_capture_start(int max_samples)
{
    // ★ 借鉴 xiaozhi：临界区保护，防止 fetch task 正在 memcpy 时被 free
    portENTER_CRITICAL(&s_capture_lock);
    if (s_capture_buf) {
        int16_t *old = s_capture_buf;
        s_capture_buf = NULL;  // 先置 NULL，让 fetch task 跳过
        portEXIT_CRITICAL(&s_capture_lock);
        free(old);
    } else {
        portEXIT_CRITICAL(&s_capture_lock);
    }

    s_capture_max  = max_samples;
    s_capture_idx  = 0;
    s_capture_done = false;
    s_capture_seen_speech = false;
    s_capture_vad_speech = false;
    s_capture_vad_speech_samples = 0;
    s_capture_last_voice_idx = 0;

    s_capture_buf  = heap_caps_malloc(max_samples * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    if (!s_capture_buf) {
        ESP_LOGE(TAG, "capture malloc failed (%d)", max_samples);
        s_capture_max = 0;
        s_capture_seen_speech = false;
        s_capture_vad_speech = false;
        s_capture_vad_speech_samples = 0;
        s_capture_last_voice_idx = 0;
        s_capture_done = true;
    }
}

int16_t *afe_capture_finish(int *out_samples)
{
    // ★ 临界区：原子地取走 buffer 并置 NULL
    portENTER_CRITICAL(&s_capture_lock);
    if (out_samples) *out_samples = s_capture_idx;
    int16_t *buf = s_capture_buf;
    s_capture_buf  = NULL;
    portEXIT_CRITICAL(&s_capture_lock);

    s_capture_idx  = 0;
    s_capture_max  = 0;
    s_capture_seen_speech = false;
    s_capture_vad_speech = false;
    s_capture_vad_speech_samples = 0;
    s_capture_last_voice_idx = 0;
    s_capture_done = false;
    return buf;
}

int afe_capture_samples(void)
{
    return s_capture_idx;
}

int afe_capture_read_from(int sample_offset, int16_t *out, int max_samples)
{
    if (!out || max_samples <= 0 || sample_offset < 0) {
        return 0;
    }

    portENTER_CRITICAL(&s_capture_lock);
    int16_t *buf = s_capture_buf;
    int available = (buf && s_capture_idx > sample_offset)
                    ? s_capture_idx - sample_offset : 0;
    if (available > max_samples) available = max_samples;
    if (available > 0 && buf) {
        memcpy(out, buf + sample_offset, available * sizeof(int16_t));
    }
    portEXIT_CRITICAL(&s_capture_lock);

    return available;
}

bool afe_capture_is_done(void)
{
    return s_capture_done;
}

bool afe_capture_seen_speech(void)
{
    return s_capture_seen_speech;
}

bool afe_capture_had_speech(void)
{
    return s_capture_vad_speech;
}
