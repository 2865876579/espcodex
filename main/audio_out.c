#include "audio_out.h"
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "audio_out";

// ═══════════════════════════════════════════════════
//  小智方案：两根独立 I2S 总线，喇叭+麦克风各有独立时钟
//  I2S0 = TX(喇叭 MAX98357A)
//  I2S1 = RX(麦克风 INMP441)
// ═══════════════════════════════════════════════════

// ── I2S0 TX: MAX98357A 喇叭 ──
#define I2S0_BCLK_GPIO   GPIO_NUM_5
#define I2S0_LRC_GPIO    GPIO_NUM_4
#define I2S0_DOUT_GPIO   GPIO_NUM_6

// ── I2S1 RX: INMP441 麦克风 ──
#define I2S1_SCK_GPIO    GPIO_NUM_11
#define I2S1_WS_GPIO     GPIO_NUM_10
#define I2S1_DIN_GPIO    GPIO_NUM_3

#define SAMPLE_RATE      16000
#define DMA_DESC_NUM     8
#define DMA_FRAME_NUM    511    // 对齐 AFE feed chunksize (512 超 DMA buffer，驱动强制截断)

static i2s_chan_handle_t s_tx_chan = NULL;  // I2S0 喇叭
static i2s_chan_handle_t s_rx_chan = NULL;  // I2S1 麦克风


void audio_out_init(void)
{
    // ═══════════════════════════════════════
    //  I2S0: TX only — MAX98357A 喇叭
    // ═══════════════════════════════════════
    i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    tx_chan_cfg.dma_desc_num = DMA_DESC_NUM;
    tx_chan_cfg.dma_frame_num = DMA_FRAME_NUM;
    ESP_ERROR_CHECK(i2s_new_channel(&tx_chan_cfg, &s_tx_chan, NULL));  // RX=NULL, TX only

    i2s_std_config_t tx_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S0_BCLK_GPIO,
            .ws   = I2S0_LRC_GPIO,
            .dout = I2S0_DOUT_GPIO,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false, .bclk_inv = false, .ws_inv = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_tx_chan, &tx_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_tx_chan));
    ESP_LOGI(TAG, "I2S0 TX 就绪 (BCLK=%d, LRC=%d, DOUT=%d)",
             I2S0_BCLK_GPIO, I2S0_LRC_GPIO, I2S0_DOUT_GPIO);

    // ═══════════════════════════════════════
    //  I2S1: RX only — INMP441 麦克风
    // ═══════════════════════════════════════
    i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    rx_chan_cfg.dma_desc_num = DMA_DESC_NUM;
    rx_chan_cfg.dma_frame_num = DMA_FRAME_NUM;
    ESP_ERROR_CHECK(i2s_new_channel(&rx_chan_cfg, NULL, &s_rx_chan));  // TX=NULL, RX only

    // INMP441 输出 24-bit 左对齐在 32-bit slot，且要求 BCLK ≥ 1MHz
    // BCLK = 16kHz × 32 × 2 = 1.024MHz
    i2s_std_config_t rx_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT,
            .slot_mode      = I2S_SLOT_MODE_STEREO,
            .slot_mask      = I2S_STD_SLOT_BOTH,
            .ws_width       = I2S_SLOT_BIT_WIDTH_32BIT,
            .ws_pol         = false,
            .bit_shift      = true,   // Philips: data 1 BCLK after WS
            .left_align     = false,
            .big_endian     = false,
            .bit_order_lsb  = false,
        },
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S1_SCK_GPIO,
            .ws   = I2S1_WS_GPIO,
            .dout = I2S_GPIO_UNUSED,
            .din  = I2S1_DIN_GPIO,
            .invert_flags = {
                .mclk_inv = false, .bclk_inv = false, .ws_inv = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_rx_chan, &rx_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_rx_chan));
    ESP_LOGI(TAG, "I2S1 RX 就绪 (SCK=%d, WS=%d, DIN=%d, BCLK=1.024MHz)",
             I2S1_SCK_GPIO, I2S1_WS_GPIO, I2S1_DIN_GPIO);

    ESP_LOGI(TAG, "双 I2S 独立总线初始化完成");
}


void audio_out_write(const uint8_t *data, size_t len)
{
    if (s_tx_chan == NULL || data == NULL || len == 0) return;
    size_t written = 0;
    i2s_channel_write(s_tx_chan, data, len, &written, portMAX_DELAY);
}


i2s_chan_handle_t audio_out_get_rx_chan(void)
{
    return s_rx_chan;
}


void audio_out_play_test_tone(void)
{
    const int freq = 440;
    const int duration_ms = 1000;
    const int total_samples = SAMPLE_RATE * duration_ms / 1000;
    const int chunk_samples = 256;

    int16_t buf[chunk_samples * 2];
    ESP_LOGI(TAG, "播放测试音 %dHz, %dms...", freq, duration_ms);

    int sample_idx = 0;
    while (sample_idx < total_samples) {
        int n = (total_samples - sample_idx > chunk_samples)
                    ? chunk_samples : (total_samples - sample_idx);
        for (int i = 0; i < n; i++) {
            double t = (double)(sample_idx + i) / SAMPLE_RATE;
            int16_t val = (int16_t)(0.3 * 32767.0 * sin(2.0 * M_PI * freq * t));
            buf[i * 2]     = val;
            buf[i * 2 + 1] = val;
        }
        audio_out_write((const uint8_t *)buf, n * 2 * sizeof(int16_t));
        sample_idx += n;
    }
    ESP_LOGI(TAG, "测试音播放完成");
}
