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

#define OPUS_SAMPLE_RATE    16000
#define OPUS_CHANNELS       1
#define AUDIO_QUEUE_DEPTH   512
#define AUDIO_PLAYER_STACK_BYTES 24576
#define AUDIO_QUEUE_SEND_TIMEOUT_MS 100
#define AUDIO_END_SEND_TIMEOUT_MS 5000
#define AUDIO_PLAYBACK_DRAIN_MS 350

// 队列元素：一个 Opus 编码帧
typedef struct {
    uint8_t *data;   // heap 分配，audio task 负责 free
    size_t   len;
} audio_chunk_t;


// ============================================================
static bool enqueue_opus_frame(const uint8_t *data, size_t len, const char *source)
{
    if (!data || len == 0 || len >= 4096) {
        s_tts_chunks_dropped++;
        ESP_LOGW(TAG, "Invalid TTS opus frame from %s: len=%u dropped=%lu",
                 source ? source : "unknown", (unsigned)len,
                 (unsigned long)s_tts_chunks_dropped);
        return false;
    }

    uint8_t *opus_data = malloc(len);
    if (!opus_data) {
        s_tts_chunks_dropped++;
        ESP_LOGW(TAG, "TTS opus frame malloc failed from %s: len=%u dropped=%lu",
                 source ? source : "unknown", (unsigned)len,
                 (unsigned long)s_tts_chunks_dropped);
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
    ESP_LOGW(TAG, "TTS audio queue full from %s, drop=%lu queued=%lu waiting=%u",
             source ? source : "unknown", (unsigned long)s_tts_chunks_dropped,
             (unsigned long)s_tts_chunks_queued,
             s_audio_queue ? (unsigned)uxQueueMessagesWaiting(s_audio_queue) : 0);
    return false;
}
//  音频播放任务 —— 独立栈，不阻塞 WebSocket
//  负责：Opus 解码 → Mono→Stereo → I2S 输出
// ============================================================
static void audio_player_task(void *arg)
{
    // 大数组在这个任务栈里（不影响 WebSocket 任务）
    int16_t pcm[960];           // 60ms @16kHz
    int16_t stereo[960 * 2];    // mono → stereo

    OpusDecoder *decoder = NULL;
    audio_chunk_t chunk;
    int played_frames = 0;

    while (1) {
        if (xQueueReceive(s_audio_queue, &chunk, pdMS_TO_TICKS(5000)) != pdTRUE) {
            continue;
        }

        // NULL 数据 = 流结束信号，销毁解码器
        if (chunk.data == NULL) {
            if (decoder) {
                opus_decoder_destroy(decoder);
                decoder = NULL;
                ESP_LOGI(TAG, "Opus decoder reset");
            }
            if (played_frames > 0) {
                vTaskDelay(pdMS_TO_TICKS(AUDIO_PLAYBACK_DRAIN_MS));
                played_frames = 0;
            }
            if (s_tts_active) {
                s_tts_active = false;
                if (s_pending_dialog_end) {
                    s_dialog_end = true;
                    s_pending_dialog_end = false;
                    ESP_LOGI(TAG, "Dialog end after playback");
                }
                s_turn_done = true;
                printf("dialog turn done\n");
            }
            continue;
        }

        // 懒初始化 Opus 解码器
        if (!decoder) {
            int err = 0;
            decoder = opus_decoder_create(OPUS_SAMPLE_RATE, OPUS_CHANNELS, &err);
            if (err != OPUS_OK || !decoder) {
                ESP_LOGE(TAG, "Opus decoder create failed: %d", err);
                free(chunk.data);
                continue;
            }
            ESP_LOGI(TAG, "Opus decoder ready");
        }

        // Opus → PCM
        int samples = opus_decode(decoder, chunk.data, chunk.len, pcm, 960, 0);
        free(chunk.data);  // 尽早释放

        if (samples < 0) {
            ESP_LOGE(TAG, "Opus decode error: %d", samples);
            continue;
        }
        if (samples == 0) {
            continue;  // FEC 或空帧
        }

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
        ESP_LOGI(TAG, "WebSocket connected to cloud");
        break;

    case WEBSOCKET_EVENT_DISCONNECTED:
        s_connected = false;

        s_tts_active = false;
        ESP_LOGW(TAG, "WebSocket disconnected");
        // 通知 audio task 重置解码器
        {
            audio_chunk_t end = { .data = NULL, .len = 0 };
            xQueueSend(s_audio_queue, &end, 0);
        }
        break;

    case WEBSOCKET_EVENT_DATA:
        if (data->op_code == 0x02 && data->data_len > 0) {
            if (!s_tts_active) {
                s_tts_chunks_dropped++;
                ESP_LOGW(TAG, "TTS binary frame while inactive: len=%d dropped=%lu",
                         data->data_len, (unsigned long)s_tts_chunks_dropped);
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
                    s_tts_active = true;
                    s_turn_done = false;
                    s_pending_dialog_end = false;

                    s_tts_chunks_queued = 0;
                    s_tts_chunks_dropped = 0;
                    cJSON *text = cJSON_GetObjectItem(json, "text");
                    ESP_LOGI(TAG, "TTS text received, len=%d",
                             (text && cJSON_IsString(text)) ? (int)strlen(text->valuestring) : 0);
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
                                    ESP_LOGW(TAG, "TTS audio queue full, drop=%lu queued=%lu waiting=%u",
                                             (unsigned long)s_tts_chunks_dropped,
                                             (unsigned long)s_tts_chunks_queued,
                                             s_audio_queue ? (unsigned)uxQueueMessagesWaiting(s_audio_queue) : 0);
                                }
                                free(opus_data);
                            }
                        }
                    }
                }
                else if (strcmp(type->valuestring, "tts_audio_end") == 0) {
                    cJSON *chunks_json = cJSON_GetObjectItem(json, "chunks");
                    int total = chunks_json ? chunks_json->valueint : 0;
                    cJSON *dialog_end = cJSON_GetObjectItem(json, "dialog_end");
                    if (dialog_end && cJSON_IsTrue(dialog_end)) {
                        s_pending_dialog_end = true;
                        ESP_LOGI(TAG, "Dialog end requested by cloud");
                    }
                    ESP_LOGI(TAG, "TTS end rx (%d frames queued=%lu dropped=%lu waiting=%u)",
                             total,
                             (unsigned long)s_tts_chunks_queued,
                             (unsigned long)s_tts_chunks_dropped,
                             s_audio_queue ? (unsigned)uxQueueMessagesWaiting(s_audio_queue) : 0);

                    audio_chunk_t end = { .data = NULL, .len = 0 };
                    if (xQueueSend(s_audio_queue, &end,
                                   pdMS_TO_TICKS(AUDIO_END_SEND_TIMEOUT_MS)) != pdTRUE) {
                        ESP_LOGE(TAG, "TTS end marker enqueue failed");
                        if (s_pending_dialog_end) {
                            s_dialog_end = true;
                            s_pending_dialog_end = false;
                        }
                        s_turn_done = true;
                    }
                }
                else if (strcmp(type->valuestring, "status") == 0) {
                    cJSON *msg = cJSON_GetObjectItem(json, "msg");
                    ESP_LOGI(TAG, "STATUS received, len=%d",
                             (msg && cJSON_IsString(msg)) ? (int)strlen(msg->valuestring) : 0);
                    s_turn_done = true;
                }
                else if (strcmp(type->valuestring, "stt_result") == 0) {
                    cJSON *text = cJSON_GetObjectItem(json, "text");
                    ESP_LOGI(TAG, "STT text received, len=%d",
                             (text && cJSON_IsString(text)) ? (int)strlen(text->valuestring) : 0);
                }
                else if (strcmp(type->valuestring, "dialog_end") == 0) {
                    s_dialog_end = true;
                    s_turn_done = true;
                    s_tts_active = false;
                    ESP_LOGI(TAG, "Dialog end");
                }
                else if (strcmp(type->valuestring, "pong") == 0) {
                    // 心跳，忽略
                }
            }
            cJSON_Delete(json);
        }
        break;

    case WEBSOCKET_EVENT_ERROR:
        ESP_LOGE(TAG, "WebSocket error");
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
    // 音频队列：深度 256（只存指针，内存开销极小）
    s_audio_queue = xQueueCreate(AUDIO_QUEUE_DEPTH, sizeof(audio_chunk_t));

    // 音频播放任务：16KB 栈，优先级 5
    // 音频播放任务：16KB 栈，优先级 4（需要及时取队列 + 写 I2S）
    xTaskCreate(audio_player_task, "audio_player", AUDIO_PLAYER_STACK_BYTES, NULL, 3, NULL);

    // WebSocket 客户端：16KB 栈（库内部帧解析也需要栈空间！）
    esp_websocket_client_config_t ws_cfg = {
        .uri = uri,
        .buffer_size = 8192,
        .reconnect_timeout_ms = 10000,
        .network_timeout_ms = 15000,
        .ping_interval_sec = 10,
        .pingpong_timeout_sec = 0,
        .disable_auto_reconnect = false,
        .task_stack = 16384,
    };

    s_client = esp_websocket_client_init(&ws_cfg);
    esp_websocket_register_events(s_client, WEBSOCKET_EVENT_ANY, ws_event_handler, NULL);
    esp_websocket_client_start(s_client);

    ESP_LOGI(TAG, "WebSocket client starting, uri=%s", uri);
}


void ws_client_restart(void)
{
    if (!s_client) {
        return;
    }

    ESP_LOGW(TAG, "Restarting WebSocket client");
    s_connected = false;
    s_tts_active = false;
    s_turn_done = false;
    s_dialog_end = false;
    s_pending_dialog_end = false;

    if (s_audio_queue) {
        audio_chunk_t end = { .data = NULL, .len = 0 };
        xQueueSend(s_audio_queue, &end, 0);
    }

    esp_err_t stop_ret = esp_websocket_client_stop(s_client);
    if (stop_ret != ESP_OK) {
        ESP_LOGW(TAG, "WebSocket stop returned 0x%x", stop_ret);
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    esp_err_t start_ret = esp_websocket_client_start(s_client);
    if (start_ret != ESP_OK) {
        ESP_LOGW(TAG, "WebSocket restart start returned 0x%x", start_ret);
    }
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
    ESP_LOGI(TAG, "Sent text (%d bytes)", json_len);

    free(json_str);
    cJSON_Delete(msg);
    return sent >= 0;
}

bool ws_client_send_raw(const char *json_str)
{
    if (!s_client || !esp_websocket_client_is_connected(s_client)) {
        ESP_LOGW(TAG, "WebSocket not connected, cannot send");
        return false;
    }

    int json_len = strlen(json_str);
    ESP_LOGI(TAG, "Sending raw JSON (%d bytes)", json_len);
    int sent = esp_websocket_client_send_text(s_client, json_str, json_len, portMAX_DELAY);
    ESP_LOGI(TAG, "Raw JSON sent (%d bytes)", json_len);
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
    s_turn_done = false;
    s_dialog_end = false;
    s_pending_dialog_end = false;
}

bool ws_client_consume_turn_done(void)
{
    bool value = s_turn_done;
    s_turn_done = false;
    return value;
}

bool ws_client_consume_dialog_end(void)
{
    bool value = s_dialog_end;
    s_dialog_end = false;
    return value;
}
