#pragma once

// Project-specific CHIP/Matter configuration overrides can go here.
// This file is referenced by CONFIG_CHIP_PROJECT_CONFIG.

// Use LwIP for CHIP SystemLayer networking on ESP-IDF.
#ifndef CHIP_SYSTEM_CONFIG_USE_LWIP
#define CHIP_SYSTEM_CONFIG_USE_LWIP 1
#endif

#ifndef CHIP_SYSTEM_CONFIG_USE_SOCKETS
#define CHIP_SYSTEM_CONFIG_USE_SOCKETS 0
#endif

#ifndef CHIP_SYSTEM_CONFIG_USE_NETWORK_FRAMEWORK
#define CHIP_SYSTEM_CONFIG_USE_NETWORK_FRAMEWORK 0
#endif

#ifndef CHIP_SYSTEM_CONFIG_USE_OPENTHREAD_ENDPOINT
#define CHIP_SYSTEM_CONFIG_USE_OPENTHREAD_ENDPOINT 0
#endif

// Ensure the device layer target is ESP32 so platform impl headers are included.
#ifndef CHIP_DEVICE_LAYER_TARGET
#define CHIP_DEVICE_LAYER_TARGET ESP32
#endif

#ifndef CHIP_DEVICE_LAYER_TARGET_ESP32
#define CHIP_DEVICE_LAYER_TARGET_ESP32 1
#endif
