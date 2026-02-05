/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#pragma once

#include <driver/i2c_master.h>
#include <esp_err.h>

esp_err_t i2c_bus_init();
i2c_master_bus_handle_t i2c_bus_get();
esp_err_t i2c_bus_add_device(uint8_t addr, i2c_master_dev_handle_t *out_dev);
