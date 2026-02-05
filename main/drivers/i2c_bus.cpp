/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_log.h>

#include "i2c_bus.h"

static const char * TAG = "i2c_bus";

static i2c_master_bus_handle_t s_bus = nullptr;

esp_err_t i2c_bus_init()
{
    if (s_bus) {
        return ESP_OK;
    }

    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = I2C_NUM_0;
    bus_cfg.sda_io_num = static_cast<gpio_num_t>(CONFIG_SHTC3_I2C_SDA_PIN);
    bus_cfg.scl_io_num = static_cast<gpio_num_t>(CONFIG_SHTC3_I2C_SCL_PIN);
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = true;

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %d", err);
        return err;
    }

    return ESP_OK;
}

i2c_master_bus_handle_t i2c_bus_get()
{
    return s_bus;
}

esp_err_t i2c_bus_add_device(uint8_t addr, i2c_master_dev_handle_t *out_dev)
{
    if (!out_dev) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = i2c_bus_init();
    if (err != ESP_OK) {
        return err;
    }

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = addr;
    dev_cfg.scl_speed_hz = CONFIG_I2C_MASTER_FREQ_HZ;

    err = i2c_master_bus_add_device(s_bus, &dev_cfg, out_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %d", err);
        return err;
    }

    return ESP_OK;
}
