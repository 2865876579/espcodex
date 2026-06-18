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
//  借鉴 xiaozhi 项目：两根独立 I2S 总线，喇叭+麦克风各有独立时钟
//  I2S0 = TX(喇叭 MAX98357A)
//  I2S1 = RX(麦克风 INMP441)
//  ★ TX 默认不 enable，只在播放音频时才开，消除空闲底噪
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
static volatile bool s_tx_enabled = false;


void audio_out_init(void)
{
    // ═══════════════════════════════════════
    //  I2S0: TX only — MAX98357A 喇叭
    //  ★ 只初始化，不 enable — 播放时才开，避免空闲底噪
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

    // ★ 借鉴 xiaozhi：降低 DOUT GPIO 驱动强度，消除 MAX98357A 信号振铃导致的杂声
    gpio_set_drive_capability(I2S0_DOUT_GPIO, GPIO_DRIVE_CAP_0);

    // ★ TX 默认不 enable — audio_out_start() 时再开

    // ═══════════════════════════════════════
    //  I2S1: RX only — INMP441 麦克风
    //  ★ RX 始终 enable，因为需要持续监听唤醒词
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
    ESP_ERROR_CHECK(i2s_channel_enable(s_rx_chan));  // RX 始终开着，等唤醒词
}


void audio_out_start(void)
{
    if (s_tx_chan && !s_tx_enabled) {
        ESP_ERROR_CHECK(i2s_channel_enable(s_tx_chan));
        s_tx_enabled = true;
    }
}


void audio_out_stop(void)
{
    if (s_tx_chan && s_tx_enabled) {
        ESP_ERROR_CHECK(i2s_channel_disable(s_tx_chan));
        s_tx_enabled = false;
    }
}


void audio_out_write(const uint8_t *data, size_t len)
{
    if (s_tx_chan == NULL || data == NULL || len == 0) return;
    if (!s_tx_enabled) {
        // 自动开启 TX，兼容未显式调用 audio_out_start 的旧调用路径
        audio_out_start();
    }
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
    audio_out_start();
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
    audio_out_stop();
}
