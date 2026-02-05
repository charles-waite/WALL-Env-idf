/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#pragma once

#include <esp_err.h>

#include <bme68x/bme68x_defs.h>

typedef struct {
    uint8_t i2c_addr;
    uint16_t heater_temp_c;
    uint16_t heater_dur_ms;
} bme68x_driver_config_t;

esp_err_t bme68x_driver_init(const bme68x_driver_config_t *config);
esp_err_t bme68x_driver_read(struct bme68x_data *out);
