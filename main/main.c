#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/idf_additions.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "wifi.h"
#include "audio_out.h"
#include "afe_wake_word.h"
#include "ws_client.h"
#include "opus.h"

#define WIFI_SSID  "qwe"
#define WIFI_PASS  "12345678"
#define WS_URI     "ws://39.106.190.124:8000/ws/esp32"

#define SAMPLE_RATE     16000
#define OPUS_FRAME_SAMPLES (SAMPLE_RATE * 60 / 1000)
#define OPUS_MAX_PACKET_BYTES 400
#define OPUS_UPLOAD_STACK_BYTES (64 * 1024)
#define OPUS_UPLOAD_QUEUE_LEN 1
#define OPUS_UPLOAD_PRIORITY 3
#define OPUS_UPLOAD_OK 1
#define OPUS_UPLOAD_FAILED 2
#define REC_MAX_DURATION_MS 6000

#define WAKE_TRIGGER_TEXT "__wake__"
#define WS_READY_TIMEOUT_MS 10000
#define WS_RESTART_INTERVAL_MS 5000
#define TURN_REPLY_TIMEOUT_MS 60000
#define NO_SPEECH_DELAY_MS 250
#define ENABLE_UART_TEXT_INPUT 0

#define SPEECH_AC_AVG_THRESHOLD 160
#define SPEECH_PEAK_THRESHOLD 1000
#define SPEECH_ACTIVE_LEVEL 500
#define SPEECH_ACTIVE_MIN_SAMPLES (SAMPLE_RATE / 20)

static const char *TAG = "app";

static volatile bool s_wake_event = false;
static volatile bool s_dialog_active = false;
static QueueHandle_t s_opus_upload_queue = NULL;
static TaskHandle_t s_opus_upload_task = NULL;

// 借鉴 xiaozhi：保护 AFE capture 状态的 spinlock，防止 fetch 任务
// 和主循环同时访问 capture buffer 造成 use-after-free
static portMUX_TYPE s_capture_spinlock = portMUX_INITIALIZER_UNLOCKED;

typedef enum {
    RECORD_SENT,
    RECORD_NO_SPEECH,
    RECORD_FAILED,
} record_result_t;

typedef enum {
    TURN_DONE,
    TURN_DIALOG_END,
    TURN_TIMEOUT,
    TURN_WS_LOST,
} turn_wait_result_t;

typedef struct {
    int16_t *pcm;
    int trim_start;
    int trim_samples;
    int ac_avg;
    int peak;
    int active;
    TaskHandle_t waiter;
} opus_upload_job_t;

static bool wait_for_ws_connected(int timeout_ms)
{
    int waited = 0;
    while (waited < timeout_ms) {
        if (ws_client_is_connected()) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        waited += 100;
    }
    return ws_client_is_connected();
}

static turn_wait_result_t wait_for_turn_result(int timeout_ms)
{
    int waited = 0;
    int last_ping = -1000;  // 负值确保首发立即 ping
    while (waited < timeout_ms) {
        if (ws_client_consume_dialog_end()) {
            return TURN_DIALOG_END;
        }
        if (ws_client_consume_turn_done()) {
            if (ws_client_consume_dialog_end()) {
                return TURN_DIALOG_END;
            }
            return TURN_DONE;
        }
        if (!ws_client_is_connected()) {
            return TURN_WS_LOST;
        }
        // ★ 借鉴 xiaozhi：每 2 秒发应用层 ping，强制 WebSocket 有数据流，防云 SLB 杀连接
        if (waited - last_ping >= 1000) {
            ws_client_send_raw("{\"type\":\"ping\"}");
            last_ping = waited;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        waited += 100;
    }
    return TURN_TIMEOUT;
}

static bool pcm_has_speech(const int16_t *pcm, int samples,
                           int *out_ac_avg, int *out_peak, int *out_active)
{
    if (!pcm || samples <= 0) {
        return false;
    }

    int64_t sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += pcm[i];
    }
    int dc = (int)(sum / samples);

    int64_t ac_sum = 0;
    int peak = 0;
    int active = 0;
    for (int i = 0; i < samples; i++) {
        int delta = (int)pcm[i] - dc;
        int a = delta >= 0 ? delta : -delta;
        ac_sum += a;
        if (a > peak) {
            peak = a;
        }
        if (a >= SPEECH_ACTIVE_LEVEL) {
            active++;
        }
    }

    int ac_avg = (int)(ac_sum / samples);
    if (out_ac_avg) {
        *out_ac_avg = ac_avg;
    }
    if (out_peak) {
        *out_peak = peak;
    }
    if (out_active) {
        *out_active = active;
    }

    return ac_avg >= SPEECH_AC_AVG_THRESHOLD
        || (peak >= SPEECH_PEAK_THRESHOLD && active >= SPEECH_ACTIVE_MIN_SAMPLES);
}

static bool send_opus_upload(const opus_upload_job_t *job)
{
    const int16_t *send_pcm = job->pcm + job->trim_start;

    int opus_err = OPUS_OK;
    OpusEncoder *encoder = opus_encoder_create(SAMPLE_RATE, 1, OPUS_APPLICATION_VOIP, &opus_err);
    if (!encoder || opus_err != OPUS_OK) {
        ESP_LOGE(TAG, "opus encoder create failed: %d", opus_err);
        return false;
    }
    opus_encoder_ctl(encoder, OPUS_SET_BITRATE(24000));
    opus_encoder_ctl(encoder, OPUS_SET_COMPLEXITY(0));
    opus_encoder_ctl(encoder, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));
    opus_encoder_ctl(encoder, OPUS_SET_VBR(1));
    opus_encoder_ctl(encoder, OPUS_SET_DTX(1));

    // ★ 借鉴 xiaozhi：先发 start，让服务器进入实时音频处理模式
    const char *start_json = "{\"type\":\"listen\",\"state\":\"start\",\"mode\":\"auto\"}";
    bool sent = ws_client_send_raw(start_json);
    if (!sent) {
        ESP_LOGE(TAG, "listen start send failed");
    }

    uint8_t opus_packet[OPUS_MAX_PACKET_BYTES];
    int16_t *pad_frame = heap_caps_malloc(OPUS_FRAME_SAMPLES * sizeof(int16_t),
                                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!pad_frame) {
        pad_frame = malloc(OPUS_FRAME_SAMPLES * sizeof(int16_t));
    }
    if (!pad_frame) {
        ESP_LOGE(TAG, "opus pad frame alloc failed");
        opus_encoder_destroy(encoder);
        return false;
    }

    int chunks = 0;
    int opus_bytes = 0;

    for (int offset = 0; sent && offset < job->trim_samples; offset += OPUS_FRAME_SAMPLES) {
        int frame_samples = job->trim_samples - offset;
        const int16_t *frame = send_pcm + offset;
        if (frame_samples < OPUS_FRAME_SAMPLES) {
            memcpy(pad_frame, frame, frame_samples * sizeof(int16_t));
            memset(pad_frame + frame_samples, 0, (OPUS_FRAME_SAMPLES - frame_samples) * sizeof(int16_t));
            frame = pad_frame;
        } else {
            frame_samples = OPUS_FRAME_SAMPLES;
        }

        int encoded = opus_encode(encoder, frame, OPUS_FRAME_SAMPLES, opus_packet, sizeof(opus_packet));
        if (encoded <= 0) {
            ESP_LOGE(TAG, "opus encode failed: %d", encoded);
            sent = false;
            break;
        }
        if (!ws_client_send_binary(opus_packet, encoded)) {
            ESP_LOGE(TAG, "opus frame send failed at chunk=%d len=%d", chunks + 1, encoded);
            sent = false;
            break;
        }
        chunks++;
        opus_bytes += encoded;
    }

    char end_json[96];
    snprintf(end_json, sizeof(end_json),
             "{\"type\":\"listen\",\"state\":\"stop\",\"chunks\":%d,\"bytes\":%d}",
             chunks, opus_bytes);
    if (sent) {
        sent = ws_client_send_raw(end_json);
        if (!sent) {
            ESP_LOGE(TAG, "listen stop send failed");
        }
    }

    free(pad_frame);
    opus_encoder_destroy(encoder);

    if (chunks > 0) {
        ESP_LOGI(TAG, "audio sent: %d frames, %d bytes", chunks, opus_bytes);
    }
    return sent;
}

static void opus_upload_task(void *arg)
{
    (void)arg;
    opus_upload_job_t job;

    while (1) {
        if (xQueueReceive(s_opus_upload_queue, &job, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        bool sent = false;
        if (job.pcm && ws_client_is_connected()) {
            sent = send_opus_upload(&job);
        }
        free(job.pcm);
        if (job.waiter) {
            xTaskNotify(job.waiter, sent ? OPUS_UPLOAD_OK : OPUS_UPLOAD_FAILED, eSetValueWithOverwrite);
        }
    }
}

static bool start_opus_upload_task(void)
{
    if (s_opus_upload_queue && s_opus_upload_task) {
        return true;
    }

    s_opus_upload_queue = xQueueCreate(OPUS_UPLOAD_QUEUE_LEN, sizeof(opus_upload_job_t));
    if (!s_opus_upload_queue) {
        ESP_LOGE(TAG, "opus upload queue create failed");
        return false;
    }

    BaseType_t ret = xTaskCreateWithCaps(opus_upload_task, "opus_upload",
                                         OPUS_UPLOAD_STACK_BYTES, NULL,
                                         OPUS_UPLOAD_PRIORITY, &s_opus_upload_task,
                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (ret != pdPASS) {
        ESP_LOGW(TAG, "opus upload PSRAM stack create failed, fallback internal");
        ret = xTaskCreate(opus_upload_task, "opus_upload", OPUS_UPLOAD_STACK_BYTES,
                          NULL, OPUS_UPLOAD_PRIORITY, &s_opus_upload_task);
    }
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "opus upload task create failed");
        vQueueDelete(s_opus_upload_queue);
        s_opus_upload_queue = NULL;
        return false;
    }

    return true;
}

static record_result_t record_and_send(void)
{
    int total = SAMPLE_RATE * REC_MAX_DURATION_MS / 1000;
    int waited_ms = 0;
    int last_keepalive_ms = -1000;  // 首发立即 ping

    afe_capture_start(total);
    while (!afe_capture_is_done()) {
        vTaskDelay(pdMS_TO_TICKS(50));
        waited_ms += 50;
        // ★ 录音期间每 2 秒发 ping，防止云 SLB 杀空闲连接
        if (waited_ms - last_keepalive_ms >= 2000) {
            ws_client_send_raw("{\"type\":\"ping\"}");
            last_keepalive_ms = waited_ms;
        }
    }

    bool vad_had_speech = afe_capture_had_speech();
    int samples = 0;
    int16_t *pcm = afe_capture_finish(&samples);

    if (!pcm || samples < SAMPLE_RATE / 2) {
        ESP_LOGW(TAG, "Record too short, skip");
        free(pcm);
        return RECORD_FAILED;
    }

    int ac_avg = 0;
    int peak = 0;
    int active = 0;
    bool has_energy = pcm_has_speech(pcm, samples, &ac_avg, &peak, &active);
    if (!vad_had_speech && !has_energy) {
        free(pcm);
        return RECORD_NO_SPEECH;
    }

    if (!s_opus_upload_queue) {
        ESP_LOGE(TAG, "opus upload task not ready");
        free(pcm);
        return RECORD_FAILED;
    }

    // 清除之前的通知
    uint32_t notify_value = 0;
    xTaskNotifyWait(0, UINT32_MAX, &notify_value, 0);

    opus_upload_job_t job = {
        .pcm = pcm,
        .trim_start = 0,
        .trim_samples = samples,
        .ac_avg = ac_avg,
        .peak = peak,
        .active = active,
        .waiter = xTaskGetCurrentTaskHandle(),
    };

    if (xQueueSend(s_opus_upload_queue, &job, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "opus upload queue send failed");
        free(pcm);
        return RECORD_FAILED;
    }

    if (xTaskNotifyWait(0, UINT32_MAX, &notify_value, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "opus upload wait failed");
        return RECORD_FAILED;
    }

    return notify_value == OPUS_UPLOAD_OK ? RECORD_SENT : RECORD_FAILED;
}

static void run_dialog(void)
{
    s_dialog_active = true;
    s_wake_event = false;

    printf("\n[唤醒] 你好小智 → 进入对话\n");
    if (!wait_for_ws_connected(WS_READY_TIMEOUT_MS)) {
        s_dialog_active = false;
        return;
    }

    ws_client_clear_events();
    ws_client_send_raw("{\"type\":\"listen\",\"state\":\"detect\",\"text\":\"你好小智\"}");

    // 等服务器问候语（tts start 已占住连接，音频帧随后就到）
    turn_wait_result_t wake_result = wait_for_turn_result(3000);
    if (wake_result == TURN_WS_LOST) {
        s_dialog_active = false;
        return;
    }
    // timeout 或 turn_done：继续录音

    while (s_dialog_active) {
        if (!wait_for_ws_connected(WS_READY_TIMEOUT_MS)) {
            break;
        }

        ws_client_clear_events();
        record_result_t rec = record_and_send();
        if (rec == RECORD_NO_SPEECH) {
            vTaskDelay(pdMS_TO_TICKS(NO_SPEECH_DELAY_MS));
            continue;
        }
        if (rec == RECORD_FAILED) {
            break;
        }

        turn_wait_result_t result = wait_for_turn_result(TURN_REPLY_TIMEOUT_MS);
        if (result == TURN_DIALOG_END) {
            printf("[对话结束]\n\n");
            break;
        }
        if (result == TURN_WS_LOST || result == TURN_TIMEOUT) {
            break;
        }
    }

    ws_client_clear_events();
    s_wake_event = false;
    s_dialog_active = false;
}

static void on_wake_word(void)
{
    if (!s_dialog_active) {
        s_wake_event = true;
    }
}

void app_main(void)
{
    printf("\n========== 智能枕头 v1.0 ==========\n\n");
    if (!start_opus_upload_task()) {
        ESP_LOGE(TAG, "Opus task init failed");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
    audio_out_init();

    uart_config_t uart_cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_driver_install(UART_NUM_0, 512, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_cfg);

    if (wifi_connect(WIFI_SSID, WIFI_PASS) != 0) {
        ESP_LOGE(TAG, "WiFi failed");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    ws_client_start(WS_URI);
    vTaskDelay(pdMS_TO_TICKS(2000));

    if (afe_wake_word_init(on_wake_word) != 0) {
        ESP_LOGE(TAG, "AFE init failed");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
    vTaskDelay(pdMS_TO_TICKS(1500));

    printf("[就绪] 说 '你好小智' 唤醒\n\n");

    uint8_t rx_buf[128];
    TickType_t last_ws_restart = xTaskGetTickCount();
    while (1) {
        if (s_wake_event) {
            run_dialog();
        }

        if (!s_dialog_active && !ws_client_is_connected()) {
            TickType_t now = xTaskGetTickCount();
            if ((now - last_ws_restart) >= pdMS_TO_TICKS(WS_RESTART_INTERVAL_MS)) {
                ws_client_restart();
                last_ws_restart = now;
            }
        } else {
            last_ws_restart = xTaskGetTickCount();
        }

        int len = uart_read_bytes(UART_NUM_0, rx_buf, sizeof(rx_buf) - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            rx_buf[len] = '\0';
            char *cmd = (char *)rx_buf;
            while (*cmd == '\r' || *cmd == '\n') {
                cmd++;
            }
            char *end = cmd + strlen(cmd) - 1;
            while (end > cmd && (*end == '\r' || *end == '\n')) {
                *end = '\0';
                end--;
            }

            if (strlen(cmd) > 0) {
#if ENABLE_UART_TEXT_INPUT
                if (strncmp(cmd, "text:", 5) == 0) {
                    char *payload = cmd + 5;
                    while (*payload == ' ') payload++;
                    if (*payload) ws_client_send_text(payload);
                }
#endif
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
