/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <cstring>

#include <esp_log.h>
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
    chip::DeviceLayer::SystemLayer().ScheduleLambda([temp_c_x100]() {
        esp_matter_attr_val_t val = esp_matter_int16(temp_c_x100);
        attribute::update(s_cfg.temp_endpoint, TemperatureMeasurement::Id,
                          TemperatureMeasurement::Attributes::MeasuredValue::Id, &val);
    });
}

static void update_humidity(float humidity_pct)
{
    const uint16_t humidity_x100 = static_cast<uint16_t>(humidity_pct * 100.0f);
    chip::DeviceLayer::SystemLayer().ScheduleLambda([humidity_x100]() {
        esp_matter_attr_val_t val = esp_matter_uint16(humidity_x100);
        attribute::update(s_cfg.humidity_endpoint, RelativeHumidityMeasurement::Id,
                          RelativeHumidityMeasurement::Attributes::MeasuredValue::Id, &val);
    });
}

static void update_pressure(float pressure_hpa)
{
    const int16_t pressure_kpa_x10 = static_cast<int16_t>(pressure_hpa);
    chip::DeviceLayer::SystemLayer().ScheduleLambda([pressure_kpa_x10]() {
        esp_matter_attr_val_t val = esp_matter_int16(pressure_kpa_x10);
        attribute::update(s_cfg.pressure_endpoint, PressureMeasurement::Id,
                          PressureMeasurement::Attributes::MeasuredValue::Id, &val);
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
    const uint8_t aq_enum = (accuracy >= 2) ? map_iaq_to_enum(static_iaq)
                                            : chip::to_underlying(AirQuality::AirQualityEnum::kUnknown);

    chip::DeviceLayer::SystemLayer().ScheduleLambda([aq_enum]() {
        esp_matter_attr_val_t val = esp_matter_uint8(aq_enum);
        attribute::update(s_cfg.air_quality_endpoint, AirQuality::Id,
                          AirQuality::Attributes::AirQuality::Id, &val);
    });
}

static void update_co2(float co2_ppm)
{
    chip::DeviceLayer::SystemLayer().ScheduleLambda([co2_ppm]() {
        esp_matter_attr_val_t val = esp_matter_float(co2_ppm);
        attribute::update(s_cfg.co2_endpoint, CarbonDioxideConcentrationMeasurement::Id,
                          CarbonDioxideConcentrationMeasurement::Attributes::MeasuredValue::Id, &val);
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
        ESP_LOGI(TAG, "BSEC: T=%.2fC RH=%.2f%% P=%.2fhPa IAQ=%.1f(acc=%u) CO2=%.0fppm",
                 latest.temp_c, latest.humidity_pct, latest.pressure_hpa,
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

    s_bsec.setTemperatureOffset(0.0f);

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
        xTaskCreate(bsec_task, "bsec2_task", 8192, nullptr, 5, &s_task);
    }

    return ESP_OK;
}
