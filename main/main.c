#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "wifi.h"
#include "audio_out.h"
#include "afe_wake_word.h"
#include "ws_client.h"
#include "mbedtls/base64.h"

#define WIFI_SSID  "qwe"
#define WIFI_PASS  "12345678"
#define WS_URI     "ws://39.106.190.124:8000/ws/esp32"

#define SAMPLE_RATE     16000
#define REC_MAX_DURATION_MS 6000

#define WAKE_TRIGGER_TEXT "__wake__"
#define WS_READY_TIMEOUT_MS 10000
#define WS_RESTART_INTERVAL_MS 15000
#define WAKE_REPLY_TIMEOUT_MS 30000
#define TURN_REPLY_TIMEOUT_MS 60000
#define NO_SPEECH_DELAY_MS 250

#define SPEECH_AC_AVG_THRESHOLD 160
#define SPEECH_PEAK_THRESHOLD 1000
#define SPEECH_ACTIVE_LEVEL 500
#define SPEECH_ACTIVE_MIN_SAMPLES (SAMPLE_RATE / 20)

#define TRIM_FRAME_SAMPLES (SAMPLE_RATE * 30 / 1000)
#define TRIM_PRE_SAMPLES   (SAMPLE_RATE * 200 / 1000)
#define TRIM_POST_SAMPLES  (SAMPLE_RATE * 300 / 1000)
#define TRIM_AC_AVG_THRESHOLD 260
#define TRIM_PEAK_THRESHOLD   1800
#define TRIM_ACTIVE_LEVEL     700

static const char *TAG = "app";

static volatile bool s_wake_event = false;
static volatile bool s_dialog_active = false;

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

static bool pcm_frame_has_speech(const int16_t *pcm, int samples)
{
    if (!pcm || samples <= 0) {
        return false;
    }

    int64_t sum = 0;
    for (int index = 0; index < samples; index++) {
        sum += pcm[index];
    }
    int dc = (int)(sum / samples);

    int64_t ac_sum = 0;
    int peak = 0;
    int active = 0;
    for (int index = 0; index < samples; index++) {
        int delta = (int)pcm[index] - dc;
        int abs_delta = delta >= 0 ? delta : -delta;
        ac_sum += abs_delta;
        if (abs_delta > peak) {
            peak = abs_delta;
        }
        if (abs_delta >= TRIM_ACTIVE_LEVEL) {
            active++;
        }
    }

    int ac_avg = (int)(ac_sum / samples);
    int min_active = samples / 24;
    if (min_active < 8) {
        min_active = 8;
    }

    return ac_avg >= TRIM_AC_AVG_THRESHOLD
        || (peak >= TRIM_PEAK_THRESHOLD && active >= min_active);
}

static bool pcm_find_speech_bounds(const int16_t *pcm, int samples,
                                   int *out_start, int *out_count)
{
    int first = -1;
    int last = -1;
    for (int frame_start = 0; frame_start < samples; frame_start += TRIM_FRAME_SAMPLES) {
        int frame_samples = samples - frame_start;
        if (frame_samples > TRIM_FRAME_SAMPLES) {
            frame_samples = TRIM_FRAME_SAMPLES;
        }
        if (pcm_frame_has_speech(pcm + frame_start, frame_samples)) {
            if (first < 0) {
                first = frame_start;
            }
            last = frame_start + frame_samples;
        }
    }

    if (first < 0 || last <= first) {
        return false;
    }

    int start = first - TRIM_PRE_SAMPLES;
    if (start < 0) {
        start = 0;
    }
    int end = last + TRIM_POST_SAMPLES;
    if (end > samples) {
        end = samples;
    }
    if (end <= start) {
        return false;
    }

    *out_start = start;
    *out_count = end - start;
    return true;
}
static record_result_t record_and_send(void)
{
    int total = SAMPLE_RATE * REC_MAX_DURATION_MS / 1000;

    ESP_LOGI(TAG, "Recording (max %dms, auto-stop on silence)...", REC_MAX_DURATION_MS);
    afe_capture_start(total);
    while (!afe_capture_is_done()) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    bool vad_had_speech = afe_capture_had_speech();
    int samples = 0;
    int16_t *pcm = afe_capture_finish(&samples);
    ESP_LOGI(TAG, "Record done: %d samples vad_speech=%d", samples, vad_had_speech ? 1 : 0);

    if (!pcm || samples < SAMPLE_RATE / 2) {
        ESP_LOGW(TAG, "Record too short, skip");
        free(pcm);
        return RECORD_FAILED;
    }
    if (!vad_had_speech) {
        ESP_LOGI(TAG, "No VAD speech, skip upload");
        free(pcm);
        return RECORD_NO_SPEECH;
    }

    int trim_start = 0;
    int trim_samples = samples;
    if (pcm_find_speech_bounds(pcm, samples, &trim_start, &trim_samples)) {
        ESP_LOGI(TAG, "Trim audio: %d -> %d samples (start=%d)", samples, trim_samples, trim_start);
    } else {
        ESP_LOGW(TAG, "Speech bounds not found, keep full record");
    }

    const int16_t *send_pcm = pcm + trim_start;
    int ac_avg = 0;
    int peak = 0;
    int active = 0;
    if (!pcm_has_speech(send_pcm, trim_samples, &ac_avg, &peak, &active)) {
        ESP_LOGI(TAG, "No speech, skip upload: ac_avg=%d peak=%d active=%d",
                 ac_avg, peak, active);
        free(pcm);
        return RECORD_NO_SPEECH;
    }

    size_t pcm_bytes = trim_samples * sizeof(int16_t);
    size_t b64_len = ((pcm_bytes + 2) / 3) * 4 + 8;
    char *b64 = malloc(b64_len);
    if (!b64) {
        free(pcm);
        return RECORD_FAILED;
    }

    size_t out = 0;
    int enc_ret = mbedtls_base64_encode((unsigned char *)b64, b64_len, &out,
                                        (const unsigned char *)send_pcm, pcm_bytes);
    free(pcm);
    if (enc_ret != 0) {
        ESP_LOGE(TAG, "base64 encode failed: %d", enc_ret);
        free(b64);
        return RECORD_FAILED;
    }

    const char *prefix = "{\"type\":\"audio\",\"audio\":\"";
    const char *suffix = "\"}";
    size_t json_len = strlen(prefix) + out + strlen(suffix) + 1;
    char *json = malloc(json_len);
    if (!json) {
        free(b64);
        return RECORD_FAILED;
    }

    int pos = snprintf(json, json_len, "%s", prefix);
    memcpy(json + pos, b64, out);
    pos += (int)out;
    memcpy(json + pos, suffix, strlen(suffix));
    pos += (int)strlen(suffix);
    json[pos] = '\0';
    free(b64);

    ESP_LOGI(TAG, "Send audio: base64=%d bytes samples=%d ac_avg=%d peak=%d active=%d",
             (int)out, trim_samples, ac_avg, peak, active);
    bool sent = ws_client_send_raw(json);
    free(json);

    return sent ? RECORD_SENT : RECORD_FAILED;
}

static void run_dialog(void)
{
    s_dialog_active = true;
    s_wake_event = false;

    ESP_LOGI(TAG, "Wake word accepted, entering dialog");
    if (!wait_for_ws_connected(WS_READY_TIMEOUT_MS)) {
        ESP_LOGW(TAG, "WebSocket not ready, dialog canceled");
        s_dialog_active = false;
        return;
    }

    ws_client_clear_events();
    if (!ws_client_send_text(WAKE_TRIGGER_TEXT)) {
        ESP_LOGW(TAG, "Wake trigger send failed");
        s_dialog_active = false;
        return;
    }

    turn_wait_result_t wake_result = wait_for_turn_result(WAKE_REPLY_TIMEOUT_MS);
    if (wake_result == TURN_DIALOG_END || wake_result == TURN_WS_LOST) {
        ESP_LOGI(TAG, "Dialog ended during wake reply");
        s_dialog_active = false;
        return;
    }
    if (wake_result == TURN_TIMEOUT) {
        ESP_LOGW(TAG, "Wake reply timeout, continue listening");
    }

    while (s_dialog_active) {
        if (!wait_for_ws_connected(WS_READY_TIMEOUT_MS)) {
            ESP_LOGW(TAG, "WebSocket lost, exit dialog");
            break;
        }

        ws_client_clear_events();
        record_result_t rec = record_and_send();
        if (rec == RECORD_NO_SPEECH) {
            vTaskDelay(pdMS_TO_TICKS(NO_SPEECH_DELAY_MS));
            continue;
        }
        if (rec == RECORD_FAILED) {
            ESP_LOGW(TAG, "Record/send failed, exit dialog");
            break;
        }

        turn_wait_result_t result = wait_for_turn_result(TURN_REPLY_TIMEOUT_MS);
        if (result == TURN_DIALOG_END) {
            ESP_LOGI(TAG, "Exit dialog by cloud command");
            break;
        }
        if (result == TURN_WS_LOST) {
            ESP_LOGW(TAG, "WebSocket disconnected during turn");
            break;
        }
        if (result == TURN_TIMEOUT) {
            ESP_LOGW(TAG, "Cloud reply timeout, continue listening");
        }
    }

    ws_client_clear_events();
    s_wake_event = false;
    s_dialog_active = false;
    ESP_LOGI(TAG, "Dialog stopped, waiting for wake word");
}

static void on_wake_word(void)
{
    if (!s_dialog_active) {
        s_wake_event = true;
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== smart pillow firmware start ===");
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
        ESP_LOGE(TAG, "WiFi connect failed");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    ESP_LOGI(TAG, "WiFi connected");

    ws_client_start(WS_URI);
    vTaskDelay(pdMS_TO_TICKS(2000));

    if (afe_wake_word_init(on_wake_word) != 0) {
        ESP_LOGE(TAG, "AFE wake word init failed");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    vTaskDelay(pdMS_TO_TICKS(1500));

    ESP_LOGI(TAG, "Ready. Say wake word to start dialog");

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
                ESP_LOGI(TAG, "Send text: %s", cmd);
                ws_client_send_text(cmd);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
