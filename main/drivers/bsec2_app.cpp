/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <cstring>
#include <cmath>

#include <esp_log.h>
#include <esp_rom_sys.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <nvs.h>
#include <nvs_flash.h>

#include <app/server/Server.h>
#include <esp_matter.h>

#include <bsec2.h>

#include "drivers/i2c_bus.h"
#include "drivers/bsec2_app.h"
#include "drivers/bsec2_configs.h"

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace chip::app::Clusters;

static const char *TAG = "bsec2_app";

static constexpr const char *kNvsNamespace = "bsec2";
static constexpr const char *kNvsStateKey = "state";
static constexpr const char *kNvsTempProfileKey = "temp_profile";
static constexpr int64_t kStateSaveIntervalMs = 60 * 60 * 1000; // 1 hour

typedef struct {
    i2c_master_dev_handle_t dev = nullptr;
} bsec2_i2c_ctx_t;

static Bsec2 s_bsec;
static bsec2_i2c_ctx_t s_i2c_ctx;
static bsec2_app_config_t s_cfg;
static TaskHandle_t s_task = nullptr;
static int64_t s_last_state_save_ms = 0;
static uint8_t s_last_iaq_accuracy = 0;
static int64_t s_last_log_ms = 0;

typedef struct {
    bool have_temp = false;
    bool have_humidity = false;
    bool have_pressure = false;
    bool have_iaq = false;
    bool have_co2 = false;
    float temp_c = 0;
    float humidity_pct = 0;
    float pressure_hpa = 0;
    float static_iaq = 0;
    float co2_ppm = 0;
    uint8_t iaq_accuracy = 0;
} bsec_latest_t;

static bsec_latest_t s_latest;
static portMUX_TYPE s_latest_mux = portMUX_INITIALIZER_UNLOCKED;

static constexpr int64_t kMinPublishIntervalMs = 30000;
static constexpr int16_t kTempDeltaCenti = 10;       // 0.10 C
static constexpr uint16_t kHumidityDeltaCenti = 50;  // 0.50 %
static constexpr int16_t kPressureDelta = 1;         // 1 hPa
static constexpr float kCo2DeltaPpm = 10.0f;         // 10 ppm
static constexpr float kDefaultTempOffsetC = CONFIG_WALL_ENV_BSEC_TEMP_OFFSET_CENTI / 100.0f;

typedef struct {
    const char *name;
    float temp_offset_c;
} temp_profile_t;

static constexpr temp_profile_t kTempProfiles[] = {
    {.name = "v1", .temp_offset_c = 6.5f},
    {.name = "v2", .temp_offset_c = 5.0f},
};

static const temp_profile_t *s_active_temp_profile = &kTempProfiles[1]; // v2 default
static float s_active_temp_offset_c = kDefaultTempOffsetC;
static bool s_pending_temp_offset_apply = false;
static bool s_temp_profile_loaded_from_nvs = false;

typedef struct {
    bool valid = false;
    int64_t last_ms = 0;
} publish_gate_t;

static publish_gate_t s_temp_gate;
static publish_gate_t s_humidity_gate;
static publish_gate_t s_pressure_gate;
static publish_gate_t s_air_quality_gate;
static publish_gate_t s_co2_gate;

static int16_t s_last_temp_c_x100 = 0;
static uint16_t s_last_humidity_x100 = 0;
static int16_t s_last_pressure_hpa = 0;
static uint8_t s_last_air_quality_enum = 0xFF;
static float s_last_co2_ppm = 0;

static const temp_profile_t *find_temp_profile(const char *name)
{
    if (!name || name[0] == '\0') {
        return nullptr;
    }
    for (size_t i = 0; i < ARRAY_LEN(kTempProfiles); ++i) {
        if (strcmp(name, kTempProfiles[i].name) == 0) {
            return &kTempProfiles[i];
        }
    }
    return nullptr;
}

static void set_active_temp_profile(const temp_profile_t *profile)
{
    if (!profile) {
        return;
    }
    s_active_temp_profile = profile;
    s_active_temp_offset_c = profile->temp_offset_c;
    s_pending_temp_offset_apply = true;
}

static void save_temp_profile_nvs()
{
    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(kNvsNamespace, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS open failed for temp profile save: %d", err);
        return;
    }
    err = nvs_set_str(nvs, kNvsTempProfileKey, s_active_temp_profile->name);
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS save temp profile failed: %d", err);
    } else {
        ESP_LOGI(TAG, "Temp profile saved: %s (offset %.2fC)",
                 s_active_temp_profile->name, s_active_temp_offset_c);
    }
    nvs_close(nvs);
}

static void load_temp_profile_nvs()
{
    s_temp_profile_loaded_from_nvs = false;
    const temp_profile_t *fallback = find_temp_profile("v2");
    if (fallback) {
        s_active_temp_profile = fallback;
        s_active_temp_offset_c = fallback->temp_offset_c;
    } else {
        s_active_temp_offset_c = kDefaultTempOffsetC;
    }

    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(kNvsNamespace, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS open failed for temp profile load: %d", err);
        return;
    }

    char profile_name[16] = {0};
    size_t len = sizeof(profile_name);
    err = nvs_get_str(nvs, kNvsTempProfileKey, profile_name, &len);
    nvs_close(nvs);

    if (err != ESP_OK) {
        ESP_LOGI(TAG, "Temp profile not set in NVS; using %s (%.2fC)",
                 s_active_temp_profile ? s_active_temp_profile->name : "default",
                 s_active_temp_offset_c);
        return;
    }

    const temp_profile_t *loaded = find_temp_profile(profile_name);
    if (!loaded) {
        ESP_LOGW(TAG, "Unknown temp profile '%s' in NVS; using %s (%.2fC)",
                 profile_name,
                 s_active_temp_profile ? s_active_temp_profile->name : "default",
                 s_active_temp_offset_c);
        return;
    }

    s_active_temp_profile = loaded;
    s_active_temp_offset_c = loaded->temp_offset_c;
    s_temp_profile_loaded_from_nvs = true;
    ESP_LOGI(TAG, "Loaded temp profile: %s (offset %.2fC)",
             s_active_temp_profile->name, s_active_temp_offset_c);
}

bool bsec2_app_get_latest(bsec2_app_latest_t *out_latest)
{
    if (!out_latest) {
        return false;
    }

    portENTER_CRITICAL(&s_latest_mux);
    const bsec_latest_t latest = s_latest;
    portEXIT_CRITICAL(&s_latest_mux);

    out_latest->have_temp = latest.have_temp;
    out_latest->have_humidity = latest.have_humidity;
    out_latest->have_pressure = latest.have_pressure;
    out_latest->have_iaq = latest.have_iaq;
    out_latest->have_co2 = latest.have_co2;
    out_latest->temp_c = latest.temp_c;
    out_latest->humidity_pct = latest.humidity_pct;
    out_latest->pressure_hpa = latest.pressure_hpa;
    out_latest->iaq = latest.static_iaq;
    out_latest->iaq_accuracy = latest.iaq_accuracy;
    out_latest->co2_ppm = latest.co2_ppm;
    return true;
}

static unsigned long bsec_millis()
{
    return static_cast<unsigned long>(esp_timer_get_time() / 1000ULL);
}

static bool publish_gate_allows(publish_gate_t &gate, bool changed)
{
    const int64_t now_ms = esp_timer_get_time() / 1000;
    if (!gate.valid) {
        gate.valid = true;
        gate.last_ms = now_ms;
        return true;
    }
    if ((now_ms - gate.last_ms) < kMinPublishIntervalMs) {
        return false;
    }
    gate.last_ms = now_ms;
    // After interval expires, publish latest value even if unchanged to keep subscriptions fresh.
    (void) changed;
    return true;
}

static void delay_us(uint32_t period_us, void *intf_ptr)
{
    (void) intf_ptr;
    // Avoid long busy-wait loops here; they can starve CHIP/OpenThread tasks.
    // Sleep coarse time via RTOS, then do a short ROM delay for the remainder.
    const TickType_t whole_ms = pdMS_TO_TICKS(period_us / 1000U);
    if (whole_ms > 0) {
        vTaskDelay(whole_ms);
    }
    const uint32_t rem_us = period_us % 1000U;
    if (rem_us > 0) {
        esp_rom_delay_us(rem_us);
    }
}

static int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    if (!intf_ptr || (!reg_data && length > 0)) {
        return BME68X_E_NULL_PTR;
    }

    const auto *ctx = static_cast<const bsec2_i2c_ctx_t *>(intf_ptr);
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

    const auto *ctx = static_cast<const bsec2_i2c_ctx_t *>(intf_ptr);
    if (!ctx->dev) {
        return BME68X_E_NULL_PTR;
    }

    const uint8_t reg = reg_addr;
    esp_err_t err = i2c_master_transmit_receive(ctx->dev, &reg, 1, reg_data, length, 1000);
    return (err == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}

static void update_temperature(float temp_c)
{
    const int16_t temp_c_x100 = static_cast<int16_t>(temp_c * 100.0f);
    const bool changed = !s_temp_gate.valid || (std::abs(temp_c_x100 - s_last_temp_c_x100) >= kTempDeltaCenti);
    if (!publish_gate_allows(s_temp_gate, changed)) {
        return;
    }
    s_last_temp_c_x100 = temp_c_x100;

    chip::DeviceLayer::SystemLayer().ScheduleLambda([temp_c_x100]() {
        esp_matter_attr_val_t val = esp_matter_nullable_int16(temp_c_x100);
        esp_err_t err = attribute::update(s_cfg.temp_endpoint, TemperatureMeasurement::Id,
                                          TemperatureMeasurement::Attributes::MeasuredValue::Id, &val);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Temperature update failed: %d", err);
        }
    });
}

static void update_humidity(float humidity_pct)
{
    const uint16_t humidity_x100 = static_cast<uint16_t>(humidity_pct * 100.0f);
    const bool changed = !s_humidity_gate.valid ||
                         (std::abs((int) humidity_x100 - (int) s_last_humidity_x100) >= kHumidityDeltaCenti);
    if (!publish_gate_allows(s_humidity_gate, changed)) {
        return;
    }
    s_last_humidity_x100 = humidity_x100;

    chip::DeviceLayer::SystemLayer().ScheduleLambda([humidity_x100]() {
        esp_matter_attr_val_t val = esp_matter_nullable_uint16(humidity_x100);
        esp_err_t err = attribute::update(s_cfg.humidity_endpoint, RelativeHumidityMeasurement::Id,
                                          RelativeHumidityMeasurement::Attributes::MeasuredValue::Id, &val);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Humidity update failed: %d", err);
        }
    });
}

static void update_pressure(float pressure_hpa)
{
    const int16_t pressure_kpa_x10 = static_cast<int16_t>(pressure_hpa);
    const bool changed = !s_pressure_gate.valid || (std::abs(pressure_kpa_x10 - s_last_pressure_hpa) >= kPressureDelta);
    if (!publish_gate_allows(s_pressure_gate, changed)) {
        return;
    }
    s_last_pressure_hpa = pressure_kpa_x10;

    chip::DeviceLayer::SystemLayer().ScheduleLambda([pressure_kpa_x10]() {
        esp_matter_attr_val_t val = esp_matter_nullable_int16(pressure_kpa_x10);
        esp_err_t err = attribute::update(s_cfg.pressure_endpoint, PressureMeasurement::Id,
                                          PressureMeasurement::Attributes::MeasuredValue::Id, &val);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Pressure update failed: %d", err);
        }
    });
}

static uint8_t map_iaq_to_enum(float iaq)
{
    using AirQualityEnum = AirQuality::AirQualityEnum;
    if (iaq <= 50.0f) return chip::to_underlying(AirQualityEnum::kGood);
    if (iaq <= 100.0f) return chip::to_underlying(AirQualityEnum::kFair);
    if (iaq <= 150.0f) return chip::to_underlying(AirQualityEnum::kModerate);
    if (iaq <= 200.0f) return chip::to_underlying(AirQualityEnum::kPoor);
    if (iaq <= 300.0f) return chip::to_underlying(AirQualityEnum::kVeryPoor);
    return chip::to_underlying(AirQualityEnum::kExtremelyPoor);
}

static void update_air_quality(float static_iaq, uint8_t accuracy)
{
    s_last_iaq_accuracy = accuracy;
    const uint8_t aq_enum = (accuracy >= 1) ? map_iaq_to_enum(static_iaq)
                                            : chip::to_underlying(AirQuality::AirQualityEnum::kUnknown);
    const bool changed = !s_air_quality_gate.valid || (aq_enum != s_last_air_quality_enum);
    if (!publish_gate_allows(s_air_quality_gate, changed)) {
        return;
    }
    s_last_air_quality_enum = aq_enum;

    chip::DeviceLayer::SystemLayer().ScheduleLambda([aq_enum]() {
        esp_matter_attr_val_t val = esp_matter_enum8(aq_enum);
        esp_err_t err = attribute::update(s_cfg.air_quality_endpoint, AirQuality::Id,
                                          AirQuality::Attributes::AirQuality::Id, &val);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "AirQuality update failed: %d", err);
        }
    });
}

static void update_co2(float co2_ppm)
{
    const bool changed = !s_co2_gate.valid || (std::fabs(co2_ppm - s_last_co2_ppm) >= kCo2DeltaPpm);
    if (!publish_gate_allows(s_co2_gate, changed)) {
        return;
    }
    s_last_co2_ppm = co2_ppm;

    chip::DeviceLayer::SystemLayer().ScheduleLambda([co2_ppm]() {
        esp_matter_attr_val_t val = esp_matter_nullable_float(co2_ppm);
        esp_err_t err = attribute::update(s_cfg.co2_endpoint, CarbonDioxideConcentrationMeasurement::Id,
                                          CarbonDioxideConcentrationMeasurement::Attributes::MeasuredValue::Id, &val);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "CO2 update failed: %d", err);
        }
    });
}

static void bsec_callback(const bme68xData data, const bsecOutputs outputs, const Bsec2 bsec)
{
    (void) data;
    (void) bsec;

    for (uint8_t i = 0; i < outputs.nOutputs; i++) {
        const bsecData &out = outputs.output[i];
        switch (out.sensor_id) {
        case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
            portENTER_CRITICAL(&s_latest_mux);
            s_latest.have_temp = true;
            s_latest.temp_c = out.signal;
            portEXIT_CRITICAL(&s_latest_mux);
            update_temperature(out.signal);
            break;
        case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
            portENTER_CRITICAL(&s_latest_mux);
            s_latest.have_humidity = true;
            s_latest.humidity_pct = out.signal;
            portEXIT_CRITICAL(&s_latest_mux);
            update_humidity(out.signal);
            break;
        case BSEC_OUTPUT_RAW_PRESSURE:
            portENTER_CRITICAL(&s_latest_mux);
            s_latest.have_pressure = true;
            s_latest.pressure_hpa = out.signal;
            portEXIT_CRITICAL(&s_latest_mux);
            update_pressure(out.signal);
            break;
        case BSEC_OUTPUT_STATIC_IAQ:
            portENTER_CRITICAL(&s_latest_mux);
            s_latest.have_iaq = true;
            s_latest.static_iaq = out.signal;
            s_latest.iaq_accuracy = out.accuracy;
            portEXIT_CRITICAL(&s_latest_mux);
            update_air_quality(out.signal, out.accuracy);
            break;
        case BSEC_OUTPUT_CO2_EQUIVALENT:
            portENTER_CRITICAL(&s_latest_mux);
            s_latest.have_co2 = true;
            s_latest.co2_ppm = out.signal;
            portEXIT_CRITICAL(&s_latest_mux);
            update_co2(out.signal);
            break;
        default:
            break;
        }
    }

    // Throttle logs to avoid spamming serial output.
    const int64_t now_ms = esp_timer_get_time() / 1000;
    if ((now_ms - s_last_log_ms) >= 5000) {
        s_last_log_ms = now_ms;
        portENTER_CRITICAL(&s_latest_mux);
        const bsec_latest_t latest = s_latest;
        portEXIT_CRITICAL(&s_latest_mux);
        const float temp_f = (latest.temp_c * 9.0f / 5.0f) + 32.0f;
        ESP_LOGI(TAG, "BSEC: T=%.2fF RH=%.2f%% P=%.2fhPa IAQ=%.1f(acc=%u) CO2=%.0fppm",
                 temp_f, latest.humidity_pct, latest.pressure_hpa,
                 latest.static_iaq, latest.iaq_accuracy, latest.co2_ppm);
    }
}

static void maybe_save_state()
{
    const int64_t now_ms = esp_timer_get_time() / 1000;
    if (s_last_iaq_accuracy < 2) {
        return;
    }
    if ((now_ms - s_last_state_save_ms) < kStateSaveIntervalMs) {
        return;
    }

    uint8_t state[BSEC_MAX_STATE_BLOB_SIZE] = {0};
    if (!s_bsec.getState(state)) {
        ESP_LOGW(TAG, "BSEC getState failed");
        return;
    }

    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(kNvsNamespace, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS open failed: %d", err);
        return;
    }
    err = nvs_set_blob(nvs, kNvsStateKey, state, BSEC_MAX_STATE_BLOB_SIZE);
    if (err == ESP_OK) {
        nvs_commit(nvs);
        s_last_state_save_ms = now_ms;
        ESP_LOGI(TAG, "[BSEC2] state saved");
    } else {
        ESP_LOGW(TAG, "NVS set blob failed: %d", err);
    }
    nvs_close(nvs);
}

static void restore_state()
{
    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(kNvsNamespace, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS open failed: %d", err);
        return;
    }
    uint8_t state[BSEC_MAX_STATE_BLOB_SIZE] = {0};
    size_t len = sizeof(state);
    err = nvs_get_blob(nvs, kNvsStateKey, state, &len);
    if (err == ESP_OK && len == sizeof(state)) {
        if (s_bsec.setState(state)) {
            ESP_LOGI(TAG, "[BSEC2] state restored");
        } else {
            ESP_LOGW(TAG, "BSEC setState failed");
        }
    }
    nvs_close(nvs);
}

static void bsec_task(void *arg)
{
    while (true) {
        if (s_pending_temp_offset_apply) {
            s_bsec.setTemperatureOffset(s_active_temp_offset_c);
            s_pending_temp_offset_apply = false;
            ESP_LOGI(TAG, "Applied temp profile %s (offset %.2fC)",
                     s_active_temp_profile ? s_active_temp_profile->name : "custom",
                     s_active_temp_offset_c);
        }
        s_bsec.run();
        maybe_save_state();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static bool set_bsec_config_from_blob(const uint8_t *blob, size_t blob_len)
{
    if (!blob || blob_len == 0) {
        return false;
    }

    // Bosch ".config" files in this repo include a 4-byte little-endian length prefix,
    // followed by the serialized settings blob.
    if (blob_len == (BSEC_MAX_PROPERTY_BLOB_SIZE + sizeof(uint32_t))) {
        uint32_t embedded_len = 0;
        memcpy(&embedded_len, blob, sizeof(embedded_len));
        if (embedded_len == BSEC_MAX_PROPERTY_BLOB_SIZE) {
            blob += sizeof(uint32_t);
            blob_len = embedded_len;
        }
    }

    return s_bsec.setConfig(blob, static_cast<uint32_t>(blob_len));
}

esp_err_t bsec2_app_start(const bsec2_app_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    s_cfg = *config;

    esp_err_t err = i2c_bus_add_device(CONFIG_BME68X_I2C_ADDR, &s_i2c_ctx.dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_bus_add_device failed: %d", err);
        return err;
    }

    if (!s_bsec.begin(BME68X_I2C_INTF, i2c_read, i2c_write, delay_us, &s_i2c_ctx, bsec_millis)) {
        ESP_LOGE(TAG, "BSEC begin failed");
        return ESP_FAIL;
    }

    load_temp_profile_nvs();
    s_bsec.setTemperatureOffset(s_active_temp_offset_c);
    s_pending_temp_offset_apply = false;
    if (s_temp_profile_loaded_from_nvs) {
        ESP_LOGI(TAG, "Device calibration set for %s.",
                 s_active_temp_profile ? s_active_temp_profile->name : "custom");
    } else {
        ESP_LOGW(TAG, "Device calibration not set. Set with 'profile <v1/v2>'. Using %s (%.2fC).",
                 s_active_temp_profile ? s_active_temp_profile->name : "default",
                 s_active_temp_offset_c);
    }

    // Load a Bosch-provided configuration blob before subscribing to virtual sensors.
    if (!set_bsec_config_from_blob(wall_env::bsec2_config::kIaq33v3s4d,
                                   wall_env::bsec2_config::kIaq33v3s4dLen)) {
        ESP_LOGE(TAG, "BSEC setConfig failed (status=%d)", static_cast<int>(s_bsec.status));
        return ESP_FAIL;
    }

    bsecSensor sensor_list[] = {
        BSEC_OUTPUT_STATIC_IAQ,
        BSEC_OUTPUT_CO2_EQUIVALENT,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
        BSEC_OUTPUT_RAW_PRESSURE,
    };
    if (!s_bsec.updateSubscription(sensor_list, ARRAY_LEN(sensor_list), BSEC_SAMPLE_RATE_LP)) {
        ESP_LOGE(TAG, "BSEC updateSubscription failed (status=%d)", static_cast<int>(s_bsec.status));
        return ESP_FAIL;
    }

    s_bsec.attachCallback(bsec_callback);
    restore_state();

    if (!s_task) {
        if (xTaskCreate(bsec_task, "bsec2_task", 8192, nullptr, 3, &s_task) != pdPASS) {
            ESP_LOGE(TAG, "Failed to start bsec2 task");
            return ESP_FAIL;
        }
    }

    return ESP_OK;
}

esp_err_t bsec2_app_set_temp_profile(const char *profile_name, bool persist)
{
    const temp_profile_t *profile = find_temp_profile(profile_name);
    if (!profile) {
        return ESP_ERR_NOT_FOUND;
    }

    set_active_temp_profile(profile);
    s_temp_profile_loaded_from_nvs = true;
    if (persist) {
        save_temp_profile_nvs();
    }
    return ESP_OK;
}

const char *bsec2_app_get_temp_profile()
{
    return s_active_temp_profile ? s_active_temp_profile->name : "unknown";
}

float bsec2_app_get_temp_offset_c()
{
    return s_active_temp_offset_c;
}
