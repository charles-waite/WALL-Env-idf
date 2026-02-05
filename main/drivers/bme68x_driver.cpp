/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstring>

#include <bme68x/bme68x.h>

#include "drivers/i2c_bus.h"
#include "drivers/bme68x_driver.h"

static const char *TAG = "bme68x_driver";

typedef struct {
    i2c_master_dev_handle_t dev;
    uint8_t i2c_addr;
} bme68x_i2c_ctx_t;

static bme68x_i2c_ctx_t s_i2c_ctx;
static bme68x_dev s_dev;
static bme68x_conf s_conf;
static bme68x_heatr_conf s_heatr_conf;
static bool s_initialized = false;

static void delay_us(uint32_t period_us, void *intf_ptr)
{
    (void) intf_ptr;
    const uint64_t start = esp_timer_get_time();
    const uint32_t lag_us = 5000;

    if (period_us > lag_us) {
        vTaskDelay(pdMS_TO_TICKS((period_us - lag_us) / 1000U));
        while ((esp_timer_get_time() - start) < (period_us - lag_us)) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    while ((esp_timer_get_time() - start) < period_us) {
    }
}

static int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    if (!intf_ptr || (!reg_data && length > 0)) {
        return BME68X_E_NULL_PTR;
    }

    const auto *ctx = static_cast<const bme68x_i2c_ctx_t *>(intf_ptr);
    if (!ctx->dev) {
        return BME68X_E_NULL_PTR;
    }

    if (length > 64) {
        return BME68X_E_INVALID_LENGTH;
    }

    uint8_t buffer[65];
    buffer[0] = reg_addr;
    if (length > 0) {
        memcpy(&buffer[1], reg_data, length);
    }

    esp_err_t err = i2c_master_transmit(ctx->dev, buffer, length + 1, 1000);
    return (err == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}

static int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    if (!intf_ptr || (!reg_data && length > 0)) {
        return BME68X_E_NULL_PTR;
    }

    const auto *ctx = static_cast<const bme68x_i2c_ctx_t *>(intf_ptr);
    if (!ctx->dev) {
        return BME68X_E_NULL_PTR;
    }

    const uint8_t reg = reg_addr;
    esp_err_t err = i2c_master_transmit_receive(ctx->dev, &reg, 1, reg_data, length, 1000);
    return (err == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}

esp_err_t bme68x_driver_init(const bme68x_driver_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_initialized) {
        return ESP_OK;
    }

    esp_err_t err = i2c_bus_add_device(config->i2c_addr, &s_i2c_ctx.dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_bus_add_device failed: %d", err);
        return err;
    }
    s_i2c_ctx.i2c_addr = config->i2c_addr;

    memset(&s_dev, 0, sizeof(s_dev));
    s_dev.intf = BME68X_I2C_INTF;
    s_dev.intf_ptr = &s_i2c_ctx;
    s_dev.read = i2c_read;
    s_dev.write = i2c_write;
    s_dev.delay_us = delay_us;

    int8_t rslt = bme68x_init(&s_dev);
    if (rslt != BME68X_OK) {
        ESP_LOGE(TAG, "bme68x_init failed: %d", rslt);
        return ESP_FAIL;
    }

    memset(&s_conf, 0, sizeof(s_conf));
    s_conf.os_hum = BME68X_OS_1X;
    s_conf.os_temp = BME68X_OS_2X;
    s_conf.os_pres = BME68X_OS_4X;
    s_conf.filter = BME68X_FILTER_OFF;
    s_conf.odr = BME68X_ODR_NONE;

    rslt = bme68x_set_conf(&s_conf, &s_dev);
    if (rslt != BME68X_OK) {
        ESP_LOGE(TAG, "bme68x_set_conf failed: %d", rslt);
        return ESP_FAIL;
    }

    memset(&s_heatr_conf, 0, sizeof(s_heatr_conf));
    s_heatr_conf.enable = BME68X_ENABLE;
    s_heatr_conf.heatr_temp = config->heater_temp_c;
    s_heatr_conf.heatr_dur = config->heater_dur_ms;

    rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &s_heatr_conf, &s_dev);
    if (rslt != BME68X_OK) {
        ESP_LOGE(TAG, "bme68x_set_heatr_conf failed: %d", rslt);
        return ESP_FAIL;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "BME68x initialized (addr=0x%02X)", config->i2c_addr);
    return ESP_OK;
}

esp_err_t bme68x_driver_read(struct bme68x_data *out)
{
    if (!s_initialized || !out) {
        return ESP_ERR_INVALID_STATE;
    }

    int8_t rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &s_dev);
    if (rslt != BME68X_OK) {
        return ESP_FAIL;
    }

    uint32_t meas_dur_us = bme68x_get_meas_dur(BME68X_FORCED_MODE, &s_conf, &s_dev);
    delay_us(meas_dur_us, nullptr);

    uint8_t n_fields = 0;
    rslt = bme68x_get_data(BME68X_FORCED_MODE, out, &n_fields, &s_dev);
    if (rslt != BME68X_OK || n_fields == 0) {
        return ESP_FAIL;
    }

    return ESP_OK;
}
