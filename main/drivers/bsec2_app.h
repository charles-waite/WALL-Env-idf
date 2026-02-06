/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#pragma once

#include <esp_err.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint16_t temp_endpoint = 0;
    uint16_t humidity_endpoint = 0;
    uint16_t pressure_endpoint = 0;
    uint16_t air_quality_endpoint = 0;
    uint16_t co2_endpoint = 0;
} bsec2_app_config_t;

typedef struct {
    bool have_temp;
    bool have_humidity;
    bool have_pressure;
    bool have_iaq;
    bool have_co2;

    float temp_c;
    float humidity_pct;
    float pressure_hpa;
    float iaq;
    uint8_t iaq_accuracy;
    float co2_ppm;
} bsec2_app_latest_t;

esp_err_t bsec2_app_start(const bsec2_app_config_t *config);
bool bsec2_app_get_latest(bsec2_app_latest_t *out_latest);
