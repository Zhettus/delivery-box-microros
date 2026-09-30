
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "wm8960.h"
#include "wm8960_clock.h"

static const char *TAG = "wm8960";

/* Registers used by this driver */
#define R_LOUT1_VOL        0x02
#define R_ROUT1_VOL        0x03
#define R_CLOCKING1        0x04
#define R_DAC_CTRL1        0x05
#define R_AUDIO_IFACE1     0x07
#define R_CLOCKING2        0x08
#define R_LDAC_VOL         0x0A
#define R_RDAC_VOL         0x0B
#define R_RESET            0x0F
#define R_POWER1           0x19
#define R_POWER2           0x1A
#define R_LOUT_MIX         0x22
#define R_ROUT_MIX         0x25
#define R_LSPK_VOL         0x28
#define R_RSPK_VOL         0x29
#define R_POWER3           0x2F
#define R_CLASSD_CTRL1     0x31
#define R_CLASSD_CTRL2     0x33
#define R_PLL_N            0x34
#define R_PLL_K1           0x35
#define R_PLL_K2           0x36
#define R_PLL_K3           0x37
#define NUM_REGS           0x38

/* Bit fields */
#define POWER1_VMID_50K    (1 << 7)    /* VMIDSEL = 01: normal playback      */
#define POWER1_VMID_5K     (3 << 7)    /* VMIDSEL = 11: fast start-up        */
#define POWER1_VREF        (1 << 6)
#define POWER2_DACL        (1 << 8)
#define POWER2_DACR        (1 << 7)
#define POWER2_LOUT1       (1 << 6)
#define POWER2_ROUT1       (1 << 5)
#define POWER2_SPKL        (1 << 4)
#define POWER2_SPKR        (1 << 3)
#define POWER2_PLL_EN      (1 << 0)
#define POWER3_LOMIX       (1 << 3)
#define POWER3_ROMIX       (1 << 2)
#define CLK1_CLKSEL_PLL    (1 << 0)
#define CLK1_SYSCLKDIV_2   (2 << 1)
#define DAC_CTRL1_DACMU    (1 << 3)
#define IFACE1_FORMAT_I2S  (2 << 0)
#define IFACE1_MASTER      (1 << 6)    /* MS: codec drives BCLK and LRCLK     */
#define CLK2_DCLKDIV_16    (7 << 6)    /* class-D clock divider (default)     */
#define MIX_DAC_TO_OUT     (1 << 8)    /* LD2LO / RD2RO                       */
#define VOL_UPDATE         (1 << 8)    /* OUT1VU / SPKVU / DACVU              */
#define VOL_ZERO_CROSS     (1 << 7)
#define DAC_VOL_0DB        0xFF
#define OUT_VOL_MIN        0x30        /* -73 dB; below this is mute          */
#define OUT_VOL_MAX        0x7F        /* +6 dB                                */
#define CLASSD_BOTH_ON     0x0F7       /* SPK_OP_EN = 11 + reserved defaults   */
#define CLASSD_GAIN_1_27   0x089       /* DC/AC boost 1.27x (5 V SPKVDD, 3.3 V AVDD) */
#define PLL_N_SDM          (1 << 5)
#define PLL_N_PRESCALE     (1 << 4)

static i2c_master_dev_handle_t s_dev;
static uint16_t                s_shadow[NUM_REGS];
static uint8_t                 s_volume;

static esp_err_t reg_write(uint8_t reg, uint16_t val)
{
    uint8_t buf[2] = {
        (uint8_t)((reg << 1) | ((val >> 8) & 0x01)),
        (uint8_t)(val & 0xFF),
    };
    esp_err_t err = i2c_master_transmit(s_dev, buf, sizeof(buf), 50);
    if (err == ESP_OK && reg < NUM_REGS) {
        s_shadow[reg] = val;
    }
    return err;
}

static uint8_t percent_to_out_vol(uint8_t pct)
{
    if (pct == 0) {
        return 0;   /* mute */
    }
    if (pct > 100) {
        pct = 100;
    }
    return (uint8_t)(OUT_VOL_MIN + ((OUT_VOL_MAX - OUT_VOL_MIN) * pct + 50) / 100);
}

esp_err_t wm8960_set_volume(uint8_t percent)
{
    uint16_t v = percent_to_out_vol(percent) | VOL_ZERO_CROSS;
    /* Write left without / right with the update bit: both change together. */
    ESP_RETURN_ON_ERROR(reg_write(R_LOUT1_VOL, v), TAG, "hp L");
    ESP_RETURN_ON_ERROR(reg_write(R_ROUT1_VOL, v | VOL_UPDATE), TAG, "hp R");
    ESP_RETURN_ON_ERROR(reg_write(R_LSPK_VOL, v), TAG, "spk L");
    ESP_RETURN_ON_ERROR(reg_write(R_RSPK_VOL, v | VOL_UPDATE), TAG, "spk R");
    s_volume = percent > 100 ? 100 : percent;
    return ESP_OK;
}

uint8_t wm8960_get_volume(void)
{
    return s_volume;
}

esp_err_t wm8960_set_mute(bool mute)
{
    uint16_t v = s_shadow[R_DAC_CTRL1];
    v = mute ? (v | DAC_CTRL1_DACMU) : (v & ~DAC_CTRL1_DACMU);
    return reg_write(R_DAC_CTRL1, v);
}

static uint16_t word_length_bits(uint8_t word_bits)
{
    switch (word_bits) {
    case 16: return 0 << 2;
    case 20: return 1 << 2;
    case 24: return 2 << 2;
    default: return 3 << 2;   /* 32 */
    }
}

esp_err_t wm8960_init(const wm8960_config_t *cfg)
{
    wm8960_clocking_t clk;
    ESP_RETURN_ON_FALSE(wm8960_clocking_calc(cfg->mclk_hz, cfg->sample_rate_hz, &clk),
                        ESP_ERR_INVALID_ARG, TAG,
                        "no PLL setting for MCLK %lu Hz / fs %lu Hz",
                        (unsigned long)cfg->mclk_hz, (unsigned long)cfg->sample_rate_hz);

    ESP_RETURN_ON_ERROR(i2c_master_probe(cfg->bus, cfg->address, 50), TAG,
                        "no answer at 0x%02X (HAT connected and powered?)", cfg->address);

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = cfg->address,
        .scl_speed_hz    = cfg->i2c_speed_hz,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(cfg->bus, &dev_cfg, &s_dev), TAG, "add");

    /* 1. Reset, then bring up VMID quickly and settle to the normal divider. */
    ESP_RETURN_ON_ERROR(reg_write(R_RESET, 0x000), TAG, "reset");
    vTaskDelay(pdMS_TO_TICKS(10));
    reg_write(R_DAC_CTRL1, DAC_CTRL1_DACMU);                 /* stay muted */
    reg_write(R_POWER1, POWER1_VMID_5K | POWER1_VREF);
    vTaskDelay(pdMS_TO_TICKS(100));
    reg_write(R_POWER1, POWER1_VMID_50K | POWER1_VREF);

    /* 2. Clocks: MCLK -> PLL -> /4 -> /2 = SYSCLK; DAC rate = SYSCLK / (256 * div). */
    reg_write(R_PLL_N, (clk.pll.fractional ? PLL_N_SDM : 0) |
                       (clk.pll.prescale ? PLL_N_PRESCALE : 0) | clk.pll.n);
    reg_write(R_PLL_K1, (clk.pll.k >> 16) & 0xFF);
    reg_write(R_PLL_K2, (clk.pll.k >> 8) & 0xFF);
    reg_write(R_PLL_K3, clk.pll.k & 0xFF);

    /* 3. Power DACs, headphone + speaker outputs, output mixers and the PLL. */
    reg_write(R_POWER2, POWER2_DACL | POWER2_DACR | POWER2_LOUT1 | POWER2_ROUT1 |
                        POWER2_SPKL | POWER2_SPKR | POWER2_PLL_EN);
    reg_write(R_POWER3, POWER3_LOMIX | POWER3_ROMIX);
    vTaskDelay(pdMS_TO_TICKS(5));                             /* PLL lock */
    reg_write(R_CLOCKING1, (uint16_t)(clk.dacdiv << 6) | (uint16_t)(clk.dacdiv << 3) |
                           CLK1_SYSCLKDIV_2 | CLK1_CLKSEL_PLL);

    /* 4. I2S master: BCLK = SYSCLK / BCLKDIV = 64 x fs, LRCLK = fs. */
    reg_write(R_CLOCKING2, CLK2_DCLKDIV_16 | clk.bclkdiv);
    reg_write(R_AUDIO_IFACE1, IFACE1_MASTER | IFACE1_FORMAT_I2S |
                              word_length_bits(cfg->word_bits));

    /* 5. Route DAC -> output mixers -> headphones + class-D speakers. */
    reg_write(R_LOUT_MIX, MIX_DAC_TO_OUT);
    reg_write(R_ROUT_MIX, MIX_DAC_TO_OUT);
    reg_write(R_CLASSD_CTRL1, CLASSD_BOTH_ON);
    reg_write(R_CLASSD_CTRL2, CLASSD_GAIN_1_27);

    /* 6. Levels: DAC at 0 dB, outputs at the requested volume, then unmute. */
    reg_write(R_LDAC_VOL, DAC_VOL_0DB);
    reg_write(R_RDAC_VOL, DAC_VOL_0DB | VOL_UPDATE);
    ESP_RETURN_ON_ERROR(wm8960_set_volume(cfg->volume_percent), TAG, "volume");
    ESP_RETURN_ON_ERROR(wm8960_set_mute(false), TAG, "unmute");

    ESP_LOGI(TAG, "ready: MCLK %lu Hz -> SYSCLK %lu Hz (PLL N=%u K=0x%06lX%s), "
                  "I2S master %lu Hz, %u-bit, volume %u%%",
             (unsigned long)cfg->mclk_hz, (unsigned long)clk.sysclk_hz, clk.pll.n,
             (unsigned long)clk.pll.k, clk.pll.prescale ? ", MCLK/2" : "",
             (unsigned long)cfg->sample_rate_hz, cfg->word_bits, s_volume);
    return ESP_OK;
}
