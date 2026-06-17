#include "audio_in.h"
#include "audio_out.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

static const char *TAG = "audio_in";

int audio_in_read(int16_t *buf, int samples)
{
    i2s_chan_handle_t rx = audio_out_get_rx_chan();
    if (!rx) return 0;

    // 单次最多读 480 stereo samples（= 1个 DMA descriptor 大小，避免跨 descriptor 超时）
    int max_chunk = 480;
    if (samples > max_chunk) samples = max_chunk;

    // static 避免栈溢出，480 stereo samples = 3840 bytes
    static int16_t stereo[480 * 2];

    size_t bytes = 0;
    esp_err_t err = i2s_channel_read(rx, stereo, samples * 4, &bytes, pdMS_TO_TICKS(500));

    if (err != ESP_OK) {
        if (bytes == 0) {
            // 真的没数据（超时、DMA 为空）
            return 0;
        }
        // 有部分数据但发生了非致命错误（如 timeout 但 bytes>0）
        ESP_LOGW(TAG, "partial read: err=%d, bytes=%d", err, (int)bytes);
    }

    int frames = bytes / 4;         // stereo frames
    if (frames > samples) frames = samples;

    // 立体声 → 提取左声道（INMP441 L/R=GND，左声道有效）
    for (int i = 0; i < frames; i++) {
        buf[i] = stereo[i * 2];
    }

    return frames;
}
