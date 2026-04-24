cmake_minimum_required(VERSION 3.16)

set(PROJECT_VER "1.2")
set(PROJECT_VER_NUMBER 1)

# This project is a Thread-only sensor (BLE commissioning + Thread transport).
# Exclude esp-matter's RainMaker integration to avoid pulling in Wi-Fi/cloud stack pieces.
set(EXCLUDE_COMPONENTS
    esp_matter_rainmaker
)

# This should be done before using the IDF_TARGET variable.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(wall_env_idf)

# Match esp-matter example build flags.
idf_build_set_property(CXX_COMPILE_OPTIONS "-std=gnu++17;-Os;-DCHIP_HAVE_CONFIG_H;-Wno-overloaded-virtual" APPEND)
idf_build_set_property(C_COMPILE_OPTIONS "-Os" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wno-format-nonliteral;-Wno-format-security;-Wno-error=array-bounds" APPEND)
