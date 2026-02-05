cmake_minimum_required(VERSION 3.16)

set(PROJECT_VER "1.0")
set(PROJECT_VER_NUMBER 1)

# Use the pinned esp-matter submodule for reproducible builds.
set(ESP_MATTER_PATH "${CMAKE_SOURCE_DIR}/esp-matter")
set(ENV{ESP_MATTER_PATH} "${ESP_MATTER_PATH}")
set(MATTER_SDK_PATH "${ESP_MATTER_PATH}/connectedhomeip/connectedhomeip")
set(ENV{MATTER_SDK_PATH} "${MATTER_SDK_PATH}")

# This should be done before using the IDF_TARGET variable.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS
    "${ESP_MATTER_PATH}/examples/common"
    "${MATTER_SDK_PATH}/config/esp32/components"
    "${ESP_MATTER_PATH}/components"
)

project(wall_env_idf)

# Match esp-matter example build flags.
idf_build_set_property(CXX_COMPILE_OPTIONS "-std=gnu++17;-Os;-DCHIP_HAVE_CONFIG_H;-Wno-overloaded-virtual" APPEND)
idf_build_set_property(C_COMPILE_OPTIONS "-Os" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wno-format-nonliteral;-Wno-format-security" APPEND)
