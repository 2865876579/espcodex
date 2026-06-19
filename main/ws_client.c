#include "ws_client.h"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_websocket_client.h"
#include "esp_err.h"
#include "cJSON.h"
#include "audio_out.h"
#include "opus.h"
#include "mbedtls/base64.h"

static const char *TAG = "ws_client";

static esp_websocket_client_handle_t s_client = NULL;
static volatile bool s_connected = false;
static volatile bool s_tts_active = false;
static volatile bool s_turn_done = false;
static volatile bool s_dialog_end = false;
static volatile bool s_pending_dialog_end = false;
static QueueHandle_t s_audio_queue = NULL;  // 音频数据队列（WebSocket → Audio Task）
static volatile uint32_t s_tts_chunks_queued = 0;
static volatile uint32_t s_tts_chunks_dropped = 0;
static OpusDecoder *s_decoder = NULL;  // ★ 模块级，避免每次 TTS 懒初始化

// 借鉴 xiaozhi：用 spinlock 保护 consume 操作的原子性，避免 WebSocket 任务
// 和主循环同时 consume 标志位时丢失事件
static portMUX_TYPE s_event_spinlock = portMUX_INITIALIZER_UNLOCKED;

#define OPUS_SAMPLE_RATE    16000
#define OPUS_CHANNELS       1
#define AUDIO_QUEUE_DEPTH   512
#define AUDIO_PLAYER_STACK_BYTES 24576
#define AUDIO_QUEUE_SEND_TIMEOUT_MS 500
#define AUDIO_END_SEND_TIMEOUT_MS 5000
#define AUDIO_PLAYBACK_DRAIN_MS 80

// 队列元素：一个 Opus 编码帧
typedef struct {
    uint8_t *data;   // heap 分配，audio task 负责 free
    size_t   len;
} audio_chunk_t;

static void begin_tts_stream(void)
{
    s_tts_active = true;
    s_turn_done = false;
    s_pending_dialog_end = false;
    s_tts_chunks_queued = 0;
    s_tts_chunks_dropped = 0;

    // ★ 预创建 Opus 解码器，避免首帧到达时才 malloc 导致丢帧
    if (!s_decoder) {
        int err = 0;
        s_decoder = opus_decoder_create(OPUS_SAMPLE_RATE, OPUS_CHANNELS, &err);
        if (err != OPUS_OK || !s_decoder) {
            ESP_LOGE(TAG, "Opus decoder create failed: %d", err);
            s_decoder = NULL;
        }
    }
}

static void end_tts_stream(bool dialog_end)
{
    if (dialog_end) {
        s_pending_dialog_end = true;
    }

    audio_chunk_t end = { .data = NULL, .len = 0 };
    if (xQueueSend(s_audio_queue, &end,
                   pdMS_TO_TICKS(AUDIO_END_SEND_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "TTS end marker enqueue failed");
        if (s_pending_dialog_end) {
            s_dialog_end = true;
            s_pending_dialog_end = false;
        }
        s_tts_active = false;
        s_turn_done = true;
    }
}


// ============================================================
static bool enqueue_opus_frame(const uint8_t *data, size_t len, const char *source)
{
    if (!data || len == 0 || len >= 4096) {
        s_tts_chunks_dropped++;
        return false;
    }

    uint8_t *opus_data = malloc(len);
    if (!opus_data) {
        s_tts_chunks_dropped++;
        return false;
    }
    memcpy(opus_data, data, len);

    audio_chunk_t chunk = { .data = opus_data, .len = len };
    if (xQueueSend(s_audio_queue, &chunk,
                   pdMS_TO_TICKS(AUDIO_QUEUE_SEND_TIMEOUT_MS)) == pdTRUE) {
        s_tts_chunks_queued++;
        return true;
    }

    free(opus_data);
    s_tts_chunks_dropped++;
    return false;
}
//  音频播放任务 —— 独立栈，不阻塞 WebSocket
//  负责：Opus 解码 → Mono→Stereo → I2S 输出
//  借鉴 xiaozhi：TX 常开，空闲写静音填充，不产生开关跳变
// ============================================================
static void audio_player_task(void *arg)
{
    // 大数组在这个任务栈里（不影响 WebSocket 任务）
    int16_t pcm[960];           // 60ms @16kHz
    int16_t stereo[960 * 2];    // mono → stereo

    audio_chunk_t chunk;
    int played_frames = 0;
    bool tx_active = false;     // 跟踪 TX 是否已开启

    while (1) {
        if (xQueueReceive(s_audio_queue, &chunk, pdMS_TO_TICKS(500)) != pdTRUE) {
            // ★ xiaozhi: TX 常开，空闲时写静音填充，不产生开关跳变
            if (tx_active) {
                memset(stereo, 0, sizeof(stereo));
                audio_out_write((const uint8_t *)stereo, sizeof(stereo));
                played_frames = 0;
            }
            continue;
        }

        // NULL 数据 = 流结束信号
        if (chunk.data == NULL) {
            // ★ 刷静音填满整个 DMA 环，根除残留音频回绕
            audio_out_flush_silence();
            if (played_frames > 0) {
                vTaskDelay(pdMS_TO_TICKS(AUDIO_PLAYBACK_DRAIN_MS));
            }
            played_frames = 0;
            tx_active = false;
            if (s_tts_active) {
                s_tts_active = false;
                if (s_pending_dialog_end) {
                    s_dialog_end = true;
                    s_pending_dialog_end = false;
                }
                s_turn_done = true;
            }
            continue;
        }

        // Opus → PCM（解码器由 begin_tts_stream 预创建）
        if (!s_decoder) {
            free(chunk.data);
            continue;
        }
        int samples = opus_decode(s_decoder, chunk.data, chunk.len, pcm, 960, 0);
        free(chunk.data);  // 尽早释放

        if (samples < 0) {
            // ★ xiaozhi: FEC → PLC 软降级
            samples = opus_decode(s_decoder, NULL, 0, pcm, 960, 1);  // FEC 纠错
        }
        if (samples < 0) {
            samples = opus_decode(s_decoder, NULL, 0, pcm, 960, 0);  // PLC 插值
        }
        if (samples <= 0) {
            // ★ FEC/PLC 都失败 → 填静音帧，DMA 不断流，不产生 pop
            if (samples < 0) ESP_LOGW(TAG, "Opus decode failed (%d), filling silence", samples);
            memset(pcm, 0, sizeof(pcm));
            samples = 960;
        }

        // TX 已常开，记录播放状态即可
        tx_active = true;

        // Mono → Stereo（MAX98357A 接收立体声，只取左声道也能响）
        for (int i = 0; i < samples; i++) {
            stereo[i * 2]     = pcm[i];
            stereo[i * 2 + 1] = pcm[i];
        }

        // I2S 输出 —— 这个任务可以慢慢等 DMA 空间
        audio_out_write((const uint8_t *)stereo, samples * 4);
        played_frames++;

    }
}


// ============================================================
//  WebSocket 事件回调 —— 在 websocket_task 中执行
//  只做轻量工作：JSON 解析 + base64 解码 + 入队列
//  绝不阻塞！（不调用 I2S、不做 Opus 解码）
// ============================================================
static void ws_event_handler(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data)
{
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;

    switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED:
        s_connected = true;
        printf("[*] 已连接云端\n");
        {
            const char *hello_json =
                "{\"type\":\"hello\",\"version\":1,\"transport\":\"websocket\","
                "\"audio_params\":{\"format\":\"opus\",\"sample_rate\":16000,"
                "\"channels\":1,\"frame_duration\":60}}";
            esp_websocket_client_send_text(s_client, hello_json, strlen(hello_json),
                                           pdMS_TO_TICKS(1000));
        }
        break;

    case WEBSOCKET_EVENT_DISCONNECTED:
        s_connected = false;
        s_tts_active = false;
        // 通知 audio task 重置解码器 + 停止播放
        {
            audio_chunk_t end = { .data = NULL, .len = 0 };
            xQueueSend(s_audio_queue, &end, 0);
        }
        break;

    case WEBSOCKET_EVENT_DATA:
        if (data->op_code == 0x02 && data->data_len > 0) {
            if (!s_tts_active) {
                s_tts_chunks_dropped++;
                break;
            }
            enqueue_opus_frame((const uint8_t *)data->data_ptr,
                               (size_t)data->data_len,
                               "binary");
            break;
        }

        if (data->op_code == 0x01 && data->data_len > 0) {
            cJSON *json = cJSON_ParseWithLength(data->data_ptr, data->data_len);
            if (!json) break;

            cJSON *type = cJSON_GetObjectItem(json, "type");
            if (type && cJSON_IsString(type)) {

                if (strcmp(type->valuestring, "tts_audio_start") == 0) {
                    begin_tts_stream();
                }
                else if (strcmp(type->valuestring, "tts") == 0) {
                    cJSON *state = cJSON_GetObjectItem(json, "state");
                    if (state && cJSON_IsString(state)) {
                        if (strcmp(state->valuestring, "start") == 0) {
                            begin_tts_stream();
                        } else if (strcmp(state->valuestring, "stop") == 0) {
                            cJSON *dialog_end = cJSON_GetObjectItem(json, "dialog_end");
                            end_tts_stream(dialog_end && cJSON_IsTrue(dialog_end));
                        } else if (strcmp(state->valuestring, "sentence_start") == 0) {
                            // ★ 流式显示 AI 回复文本
                            cJSON *text = cJSON_GetObjectItem(json, "text");
                            if (text && cJSON_IsString(text)) {
                                printf("[小安] %s\n", text->valuestring);
                            }
                        }
                    }
                }
                else if (strcmp(type->valuestring, "tts_audio_chunk") == 0) {
                    cJSON *audio = cJSON_GetObjectItem(json, "audio");
                    if (audio && cJSON_IsString(audio)) {
                        const char *b64 = audio->valuestring;
                        size_t b64_len = strlen(b64);
                        size_t out_len = 0;
                        mbedtls_base64_decode(NULL, 0, &out_len, (const unsigned char *)b64, b64_len);

                        if (out_len > 0 && out_len < 4096) {
                            uint8_t *opus_data = malloc(out_len);
                            if (opus_data) {
                                size_t actual = 0;
                                mbedtls_base64_decode(opus_data, out_len, &actual, (const unsigned char *)b64, b64_len);

                                // 入队列（非阻塞，队满就丢，保护 websocket 任务）
                                audio_chunk_t chunk = { .data = opus_data, .len = actual };
                                if (xQueueSend(s_audio_queue, &chunk,
                                               pdMS_TO_TICKS(AUDIO_QUEUE_SEND_TIMEOUT_MS)) == pdTRUE) {
                                    s_tts_chunks_queued++;
                                    opus_data = NULL;
                                } else {
                                    s_tts_chunks_dropped++;
                                }
                                free(opus_data);
                            }
                        }
                    }
                }
                else if (strcmp(type->valuestring, "tts_audio_end") == 0) {
                    cJSON *dialog_end = cJSON_GetObjectItem(json, "dialog_end");
                    end_tts_stream(dialog_end && cJSON_IsTrue(dialog_end));
                }
                else if (strcmp(type->valuestring, "status") == 0) {
                    cJSON *msg = cJSON_GetObjectItem(json, "msg");
                    if (msg && cJSON_IsString(msg)) {
                        printf("[状态] %s\n", msg->valuestring);
                    }
                    s_turn_done = true;
                }
                else if (strcmp(type->valuestring, "stt_result") == 0) {
                    cJSON *text = cJSON_GetObjectItem(json, "text");
                    if (text && cJSON_IsString(text)) {
                        printf("[你] %s\n", text->valuestring);
                    }
                }
                else if (strcmp(type->valuestring, "dialog_end") == 0) {
                    end_tts_stream(true);
                }
                else if (strcmp(type->valuestring, "pong") == 0) {
                    // 心跳，忽略
                }
            }
            cJSON_Delete(json);
        }
        break;

    case WEBSOCKET_EVENT_ERROR:
        break;

    default:
        break;
    }
}


// ============================================================
//  公开 API
// ============================================================

void ws_client_start(const char *uri)
{
    // 音频队列：深度 512（只存指针，内存开销极小）
    s_audio_queue = xQueueCreate(AUDIO_QUEUE_DEPTH, sizeof(audio_chunk_t));

    // 音频播放任务：prio=9 高于 feed(8)，确保 DMA 不断流
    xTaskCreatePinnedToCore(audio_player_task, "audio_player", AUDIO_PLAYER_STACK_BYTES, NULL, 9, NULL, 1);

    // WebSocket 客户端：16KB 栈（库内部帧解析也需要栈空间！）
    esp_websocket_client_config_t ws_cfg = {
        .uri = uri,
        .buffer_size = 8192,
        .reconnect_timeout_ms = 10000,
        .network_timeout_ms = 15000,
        .ping_interval_sec = 2,           // 每 3 秒发 ping 防止云 SLB 杀空闲连接
        .pingpong_timeout_sec = 0,          // 应用层 keepalive 已覆盖，协议层不管
        .disable_auto_reconnect = false,
        .task_stack = 16384,
    };

    s_client = esp_websocket_client_init(&ws_cfg);
    esp_websocket_register_events(s_client, WEBSOCKET_EVENT_ANY, ws_event_handler, NULL);
    esp_websocket_client_start(s_client);
}


void ws_client_restart(void)
{
    if (!s_client) return;

    s_connected = false;
    s_tts_active = false;
    s_turn_done = false;
    s_dialog_end = false;
    s_pending_dialog_end = false;

    if (s_audio_queue) {
        audio_chunk_t end = { .data = NULL, .len = 0 };
        xQueueSend(s_audio_queue, &end, 0);
    }

    esp_websocket_client_stop(s_client);
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_websocket_client_start(s_client);
}

bool ws_client_send_text(const char *text)
{
    if (!s_client || !esp_websocket_client_is_connected(s_client)) {
        ESP_LOGW(TAG, "WebSocket not connected, cannot send");
        return false;
    }

    cJSON *msg = cJSON_CreateObject();
    cJSON_AddStringToObject(msg, "type", "text");
    cJSON_AddStringToObject(msg, "text", text);
    char *json_str = cJSON_PrintUnformatted(msg);
    int json_len = strlen(json_str);

    int sent = esp_websocket_client_send_text(s_client, json_str, json_len, portMAX_DELAY);
    free(json_str);
    cJSON_Delete(msg);
    return sent >= 0;
}

bool ws_client_send_raw(const char *json_str)
{
    if (!s_client || !esp_websocket_client_is_connected(s_client)) {
        return false;
    }
    int json_len = strlen(json_str);
    int sent = esp_websocket_client_send_text(s_client, json_str, json_len, portMAX_DELAY);
    return sent >= 0;
}

bool ws_client_send_binary(const uint8_t *data, int len)
{
    if (!s_client || !esp_websocket_client_is_connected(s_client)) {
        ESP_LOGW(TAG, "WebSocket not connected, cannot send binary");
        return false;
    }
    if (!data || len <= 0) {
        return false;
    }

    int sent = esp_websocket_client_send_bin(s_client, (const char *)data, len, portMAX_DELAY);
    return sent >= 0;
}

bool ws_client_is_connected(void)
{
    return s_connected && s_client && esp_websocket_client_is_connected(s_client);
}

bool ws_client_is_tts_active(void)
{
    return s_tts_active;
}

void ws_client_clear_events(void)
{
    // ★ 借鉴 xiaozhi：临界区保护，防止和 ws_event_handler 同时改标志位
    portENTER_CRITICAL(&s_event_spinlock);
    s_turn_done = false;
    s_dialog_end = false;
    s_pending_dialog_end = false;
    portEXIT_CRITICAL(&s_event_spinlock);
}

bool ws_client_consume_turn_done(void)
{
    // ★ 临界区保护原子 consume
    portENTER_CRITICAL(&s_event_spinlock);
    bool value = s_turn_done;
    s_turn_done = false;
    portEXIT_CRITICAL(&s_event_spinlock);
    return value;
}

bool ws_client_consume_dialog_end(void)
{
    // ★ 临界区保护原子 consume
    portENTER_CRITICAL(&s_event_spinlock);
    bool value = s_dialog_end;
    s_dialog_end = false;
    portEXIT_CRITICAL(&s_event_spinlock);
    return value;
}
