/*
 * ina228.c — TI INA228 driver. See ina228.h.
 * Register map: INA228 datasheet, section 7.6.
 */
#include <stdlib.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ina228.h"
#include "ina228_conv.h"

static const char *TAG = "ina228";

#define REG_CONFIG        0x00
#define REG_ADC_CONFIG    0x01
#define REG_SHUNT_CAL     0x02
#define REG_VSHUNT        0x04
#define REG_VBUS          0x05
#define REG_DIETEMP       0x06
#define REG_CURRENT       0x07
#define REG_POWER         0x08
#define REG_ENERGY        0x09
#define REG_CHARGE        0x0A
#define REG_MANUFACTURER  0x3E
#define REG_DEVICE_ID     0x3F

#define CONFIG_RST        (1u << 15)
#define CONFIG_RSTACC     (1u << 14)
#define CONFIG_ADCRANGE   (1u << 4)     /* 1 = +/-40.96 mV, 0 = +/-163.84 mV */

/*
 * ADC_CONFIG: continuous bus + shunt + temperature (MODE = 0xF),
 * 1052 us conversions (code 5) for each, 64-sample averaging (code 3).
 * One fresh averaged result about every 200 ms.
 */
#define ADC_CONFIG_VALUE  ((0xFu << 12) | (5u << 9) | (5u << 6) | (5u << 3) | 3u)

/*
 * Fast mode: continuous shunt + bus voltage (MODE = 0xB), 280 us conversions
 * (code 3), 16-sample averaging (code 2) -> new current AND bus voltage about
 * every 9 ms. Temperature is not updated.
 */
#define ADC_CONFIG_FAST   ((0xBu << 12) | (3u << 9) | (3u << 6) | (3u << 3) | 2u)

#define MANUFACTURER_TI   0x5449        /* "TI" */
#define DEVICE_ID_INA228  0x228

#define I2C_TIMEOUT_MS    50

struct ina228 {
    i2c_master_dev_handle_t dev;
    double                  current_lsb;
    bool                    range_40mv;
};

static esp_err_t read_reg(ina228_handle_t h, uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(h->dev, &reg, 1, buf, len, I2C_TIMEOUT_MS);
}

static esp_err_t read16(ina228_handle_t h, uint8_t reg, uint16_t *v)
{
    uint8_t b[2];
    ESP_RETURN_ON_ERROR(read_reg(h, reg, b, 2), TAG, "read 0x%02X", reg);
    *v = (uint16_t)((b[0] << 8) | b[1]);
    return ESP_OK;
}

static esp_err_t read24(ina228_handle_t h, uint8_t reg, uint32_t *v)
{
    uint8_t b[3];
    ESP_RETURN_ON_ERROR(read_reg(h, reg, b, 3), TAG, "read 0x%02X", reg);
    *v = ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2];
    return ESP_OK;
}

static esp_err_t read40(ina228_handle_t h, uint8_t reg, uint64_t *v)
{
    uint8_t b[5];
    ESP_RETURN_ON_ERROR(read_reg(h, reg, b, 5), TAG, "read 0x%02X", reg);
    *v = 0;
    for (int i = 0; i < 5; i++) {
        *v = (*v << 8) | b[i];
    }
    return ESP_OK;
}

static esp_err_t write16(ina228_handle_t h, uint8_t reg, uint16_t v)
{
    uint8_t b[3] = { reg, (uint8_t)(v >> 8), (uint8_t)(v & 0xFF) };
    return i2c_master_transmit(h->dev, b, sizeof(b), I2C_TIMEOUT_MS);
}

esp_err_t ina228_create(const ina228_config_t *cfg, ina228_handle_t *out)
{
    ESP_RETURN_ON_FALSE(cfg->shunt_ohms > 0.0f && cfg->max_current_a > 0.0f,
                        ESP_ERR_INVALID_ARG, TAG, "shunt / max current");
    ESP_RETURN_ON_ERROR(i2c_master_probe(cfg->bus, cfg->address, I2C_TIMEOUT_MS), TAG,
                        "no answer at 0x%02X", cfg->address);

    struct ina228 *h = calloc(1, sizeof(*h));
    ESP_RETURN_ON_FALSE(h, ESP_ERR_NO_MEM, TAG, "alloc");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = cfg->address,
        .scl_speed_hz    = cfg->i2c_speed_hz,
    };
    esp_err_t err = i2c_master_bus_add_device(cfg->bus, &dev_cfg, &h->dev);
    if (err != ESP_OK) {
        free(h);
        return err;
    }

    uint16_t manufacturer = 0, device = 0;
    read16(h, REG_MANUFACTURER, &manufacturer);
    read16(h, REG_DEVICE_ID, &device);
    if (manufacturer != MANUFACTURER_TI || (device >> 4) != DEVICE_ID_INA228) {
        ESP_LOGE(TAG, "0x%02X is not an INA228 (manufacturer 0x%04X, device 0x%04X)",
                 cfg->address, manufacturer, device);
        i2c_master_bus_rm_device(h->dev);
        free(h);
        return ESP_ERR_NOT_FOUND;
    }

    /* Pick the more precise 40.96 mV range if the full-scale shunt voltage fits. */
    h->range_40mv  = (cfg->max_current_a * cfg->shunt_ohms) <= 0.04096f;
    h->current_lsb = ina228_current_lsb(cfg->max_current_a);
    uint16_t cal   = ina228_shunt_cal(h->current_lsb, cfg->shunt_ohms, h->range_40mv);

    write16(h, REG_CONFIG, CONFIG_RST);
    vTaskDelay(pdMS_TO_TICKS(2));
    err = write16(h, REG_CONFIG, h->range_40mv ? CONFIG_ADCRANGE : 0);
    if (err == ESP_OK) err = write16(h, REG_ADC_CONFIG, cfg->mode == INA228_MODE_FAST_CURRENT
                                                       ? ADC_CONFIG_FAST : ADC_CONFIG_VALUE);
    if (err == ESP_OK) err = write16(h, REG_SHUNT_CAL, cal);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "configuration failed: %s", esp_err_to_name(err));
        i2c_master_bus_rm_device(h->dev);
        free(h);
        return err;
    }

    ESP_LOGI(TAG, "ready at 0x%02X: shunt %.4f ohm, max %.1f A, range %s, SHUNT_CAL %u, %s",
             cfg->address, cfg->shunt_ohms, cfg->max_current_a,
             h->range_40mv ? "40.96 mV" : "163.84 mV", cal,
             cfg->mode == INA228_MODE_FAST_CURRENT ? "fast current mode" : "monitor mode");
    *out = h;
    return ESP_OK;
}

esp_err_t ina228_read(ina228_handle_t h, ina228_reading_t *r)
{
    uint32_t vbus, vshunt, current, power;
    uint16_t temp;
    uint64_t energy, charge;

    ESP_RETURN_ON_ERROR(read24(h, REG_VBUS, &vbus), TAG, "vbus");
    ESP_RETURN_ON_ERROR(read24(h, REG_VSHUNT, &vshunt), TAG, "vshunt");
    ESP_RETURN_ON_ERROR(read24(h, REG_CURRENT, &current), TAG, "current");
    ESP_RETURN_ON_ERROR(read24(h, REG_POWER, &power), TAG, "power");
    ESP_RETURN_ON_ERROR(read16(h, REG_DIETEMP, &temp), TAG, "temp");
    ESP_RETURN_ON_ERROR(read40(h, REG_ENERGY, &energy), TAG, "energy");
    ESP_RETURN_ON_ERROR(read40(h, REG_CHARGE, &charge), TAG, "charge");

    r->bus_voltage_v    = (float)ina228_vbus_volts(vbus);
    r->shunt_voltage_mv = (float)(ina228_vshunt_volts(vshunt, h->range_40mv) * 1000.0);
    r->current_a        = (float)ina228_current_amps(current, h->current_lsb);
    r->power_w          = (float)ina228_power_watts(power, h->current_lsb);
    r->die_temp_c       = (float)ina228_die_temp_c(temp);
    r->energy_wh        = ina228_energy_joules(energy, h->current_lsb) / 3600.0;
    r->charge_ah        = ina228_charge_coulombs(charge, h->current_lsb) / 3600.0;
    return ESP_OK;
}

esp_err_t ina228_read_current(ina228_handle_t h, float *amps)
{
    uint32_t raw;
    ESP_RETURN_ON_ERROR(read24(h, REG_CURRENT, &raw), TAG, "current");
    *amps = (float)ina228_current_amps(raw, h->current_lsb);
    return ESP_OK;
}

esp_err_t ina228_read_bus_voltage(ina228_handle_t h, float *volts)
{
    uint32_t raw;
    ESP_RETURN_ON_ERROR(read24(h, REG_VBUS, &raw), TAG, "vbus");
    *volts = (float)ina228_vbus_volts(raw);
    return ESP_OK;
}

esp_err_t ina228_reset_accumulators(ina228_handle_t h)
{
    uint16_t cfg;
    ESP_RETURN_ON_ERROR(read16(h, REG_CONFIG, &cfg), TAG, "config");
    return write16(h, REG_CONFIG, (uint16_t)(cfg | CONFIG_RSTACC));
}
