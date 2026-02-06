/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#pragma once

#include <esp_err.h>

// Start an OLED task (SH1106 over I2C) that shows a splash screen, then
// periodically updates with latest sensor values (if available).
//
// Safe to call even if OLED isn't present on the bus; errors are logged and
// the app continues.
esp_err_t oled_sh1106_start();
