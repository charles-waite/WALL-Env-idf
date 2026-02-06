/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>
#include <platform/ConnectivityManager.h>
#include <bsp/esp-bsp.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nvs_flash.h>

#include <app_openthread_config.h>
#include <app_reset.h>
#include <common_macros.h>
#include <esp_matter.h>
#include <esp_matter_attribute.h>
#include <esp_matter_cluster.h>
#include <esp_matter_endpoint.h>
#include <setup_payload/OnboardingCodesUtil.h>
#include <cctype>
#include <cstring>

// drivers implemented by this example
#include <drivers/bsec2_app.h>
#include <drivers/oled_sh1106.h>

static const char *TAG = "app_main";

extern "C" void init_network_driver();

static void normalize_serial_command(char *cmd)
{
    if (!cmd) {
        return;
    }

    size_t len = strlen(cmd);
    if (len == 0) {
        return;
    }

    // Trim leading whitespace.
    size_t start = 0;
    while (start < len && std::isspace(static_cast<unsigned char>(cmd[start]))) {
        start++;
    }

    // Trim trailing whitespace.
    size_t end = len;
    while (end > start && std::isspace(static_cast<unsigned char>(cmd[end - 1]))) {
        end--;
    }

    size_t out = 0;
    for (size_t i = start; i < end; ++i) {
        cmd[out++] = static_cast<char>(std::tolower(static_cast<unsigned char>(cmd[i])));
    }
    cmd[out] = '\0';
}

static void schedule_decommission()
{
    chip::DeviceLayer::SystemLayer().ScheduleLambda([]() {
        auto &fabric_table = chip::Server::GetInstance().GetFabricTable();
        const uint8_t before = fabric_table.FabricCount();
        if (before == 0) {
            ESP_LOGI(TAG, "decom: no fabrics to remove");
            return;
        }

        ESP_LOGI(TAG, "decom: removing %u fabric(s)", before);
        fabric_table.DeleteAllFabrics();
        ESP_LOGI(TAG, "decom: removal requested, remaining fabrics now %u", fabric_table.FabricCount());
    });
}

static void schedule_factory_reset()
{
    chip::DeviceLayer::SystemLayer().ScheduleLambda([]() {
        ESP_LOGW(TAG, "factoryreset: scheduling full factory reset");
        chip::Server::GetInstance().ScheduleFactoryReset();
    });
}

static void execute_serial_command(const char *cmd)
{
    if (!cmd || cmd[0] == '\0') {
        return;
    }

    ESP_LOGI(TAG, "serial cmd: '%s'", cmd);

    if (strcmp(cmd, "decom") == 0 || strcmp(cmd, "decommission") == 0) {
        ESP_LOGI(TAG, "decom: command accepted");
        schedule_decommission();
        return;
    }

    if (strcmp(cmd, "factoryreset") == 0 || strcmp(cmd, "reset") == 0) {
        schedule_factory_reset();
        return;
    }

    if (strcmp(cmd, "help") == 0) {
        ESP_LOGI(TAG, "serial commands: decom, decommission, factoryreset, reset, help");
        return;
    }

    ESP_LOGW(TAG, "Unknown serial command: '%s' (try 'help')", cmd);
}

static void serial_command_task(void *arg)
{
    (void) arg;
    static constexpr size_t kCmdMax = 32;
    char cmd[kCmdMax] = {0};
    size_t idx = 0;

    ESP_LOGI(TAG, "Serial command task ready. Type 'decom' to decommission.");
    while (true) {
        int ch = getchar();
        if (ch == EOF) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (ch == '\r' || ch == '\n') {
            if (idx > 0) {
                cmd[idx] = '\0';
                normalize_serial_command(cmd);
                execute_serial_command(cmd);
                idx = 0;
            }
            continue;
        }

        if (ch == 0x08 || ch == 0x7F) { // Backspace / DEL
            if (idx > 0) {
                idx--;
            }
            continue;
        }

        if (idx < (kCmdMax - 1)) {
            cmd[idx++] = static_cast<char>(ch);
        }
    }
}

static esp_err_t factory_reset_button_register()
{
    button_handle_t push_button;
    esp_err_t err = bsp_iot_button_create(&push_button, NULL, BSP_BUTTON_NUM);
    VerifyOrReturnError(err == ESP_OK, err);
    return app_reset_button_register(push_button);
}

static void open_commissioning_window_if_necessary()
{
    VerifyOrReturn(chip::Server::GetInstance().GetFabricTable().FabricCount() == 0);

    chip::CommissioningWindowManager  &commissionMgr = chip::Server::GetInstance().GetCommissioningWindowManager();
    VerifyOrReturn(commissionMgr.IsCommissioningWindowOpen() == false);

    // After removing last fabric, this example does not remove the Wi-Fi credentials
    // and still has IP connectivity so, only advertising on DNS-SD.
    CHIP_ERROR err = commissionMgr.OpenBasicCommissioningWindow(chip::System::Clock::Seconds16(300),
                                                                chip::CommissioningWindowAdvertisement::kDnssdOnly);
    if (err != CHIP_NO_ERROR) {
        ESP_LOGE(TAG, "Failed to open commissioning window, err:%" CHIP_ERROR_FORMAT, err.Format());
    }
}

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        break;

    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(TAG, "Commissioning failed, fail safe timer expired");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
        ESP_LOGI(TAG, "Fabric removed successfully");
        open_commissioning_window_if_necessary();
        break;

    case chip::DeviceLayer::DeviceEventType::kBLEDeinitialized:
        ESP_LOGI(TAG, "BLE deinitialized and memory reclaimed");
        break;

    default:
        break;
    }
}

// This callback is called for every attribute update. Since this device is a sensor-only
// device, we don't expect any attribute writes from the network.
static esp_err_t app_attribute_update_cb(esp_matter::attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                        uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    return ESP_OK;
}

static esp_err_t app_identification_cb(esp_matter::identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                      uint8_t effect_variant, void *priv_data)
{
    return ESP_OK;
}

extern "C" void app_main()
{
    // Endpoint 0 is always the Root Node.
    uint16_t temp_endpoint_id = 0;
    uint16_t humidity_endpoint_id = 0;
    uint16_t pressure_endpoint_id = 0;
    uint16_t air_quality_endpoint_id = 0;

    /* Initialize the ESP NVS layer */
    nvs_flash_init();

    /* Initialize push button on the dev-kit to reset the device */
    esp_err_t err = factory_reset_button_register();
    ABORT_APP_ON_FAILURE(ESP_OK == err, ESP_LOGE(TAG, "Failed to initialize reset button, err:%d", err));

    // Optional OLED splash/status task (does not abort app on failure).
    (void) oled_sh1106_start();

    /* Create a Matter node (Root Node device type on endpoint 0 is added automatically) */
    esp_matter::node::config_t node_config;
    esp_matter::node_t *node = esp_matter::node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));

    // Split endpoints for Apple Home compatibility:
    // temp, humidity, pressure as dedicated endpoints + dedicated AQ endpoint (AQ + CO2).
    esp_matter::endpoint::temperature_sensor::config_t temp_sensor_cfg;
    esp_matter::endpoint_t *temp_ep =
        esp_matter::endpoint::temperature_sensor::create(node, &temp_sensor_cfg, esp_matter::ENDPOINT_FLAG_NONE, nullptr);
    ABORT_APP_ON_FAILURE(temp_ep != nullptr, ESP_LOGE(TAG, "Failed to create temperature_sensor endpoint"));
    temp_endpoint_id = esp_matter::endpoint::get_id(temp_ep);

    esp_matter::endpoint::humidity_sensor::config_t humidity_sensor_cfg;
    esp_matter::endpoint_t *humidity_ep =
        esp_matter::endpoint::humidity_sensor::create(node, &humidity_sensor_cfg, esp_matter::ENDPOINT_FLAG_NONE, nullptr);
    ABORT_APP_ON_FAILURE(humidity_ep != nullptr, ESP_LOGE(TAG, "Failed to create humidity_sensor endpoint"));
    humidity_endpoint_id = esp_matter::endpoint::get_id(humidity_ep);

    esp_matter::endpoint::pressure_sensor::config_t pressure_sensor_cfg;
    esp_matter::endpoint_t *pressure_ep =
        esp_matter::endpoint::pressure_sensor::create(node, &pressure_sensor_cfg, esp_matter::ENDPOINT_FLAG_NONE, nullptr);
    ABORT_APP_ON_FAILURE(pressure_ep != nullptr, ESP_LOGE(TAG, "Failed to create pressure_sensor endpoint"));
    pressure_endpoint_id = esp_matter::endpoint::get_id(pressure_ep);

    // Build AQ endpoint manually to avoid helper-created internal-only AirQuality attr.
    esp_matter::endpoint_t *aq_ep = esp_matter::endpoint::create(node, esp_matter::ENDPOINT_FLAG_NONE, nullptr);
    ABORT_APP_ON_FAILURE(aq_ep != nullptr, ESP_LOGE(TAG, "Failed to create air_quality_sensor endpoint"));
    ABORT_APP_ON_FAILURE(esp_matter::cluster::descriptor::create(aq_ep, nullptr, esp_matter::CLUSTER_FLAG_SERVER) != nullptr,
                         ESP_LOGE(TAG, "Failed to create AQ Descriptor cluster"));
    ABORT_APP_ON_FAILURE(esp_matter::endpoint::add_device_type(aq_ep,
                                                                ESP_MATTER_AIR_QUALITY_SENSOR_DEVICE_TYPE_ID,
                                                                ESP_MATTER_AIR_QUALITY_SENSOR_DEVICE_TYPE_VERSION) == ESP_OK,
                         ESP_LOGE(TAG, "Failed to add AirQualitySensor device type"));
    air_quality_endpoint_id = esp_matter::endpoint::get_id(aq_ep);
    esp_matter::cluster::identify::config_t aq_identify_cfg;
    ABORT_APP_ON_FAILURE(esp_matter::cluster::identify::create(aq_ep, &aq_identify_cfg, esp_matter::CLUSTER_FLAG_SERVER) != nullptr,
                         ESP_LOGE(TAG, "Failed to create AQ Identify cluster"));

    // Create AirQuality as esp-matter-managed attr so attribute::update() works.
    esp_matter::cluster_t *aq_cluster =
        esp_matter::cluster::create(aq_ep, chip::app::Clusters::AirQuality::Id, esp_matter::CLUSTER_FLAG_SERVER);
    ABORT_APP_ON_FAILURE(aq_cluster != nullptr, ESP_LOGE(TAG, "Failed to create AirQuality cluster"));
    ABORT_APP_ON_FAILURE(esp_matter::cluster::global::attribute::create_feature_map(aq_cluster, 0) != nullptr,
                         ESP_LOGE(TAG, "Failed to create AirQuality FeatureMap"));
    ABORT_APP_ON_FAILURE(esp_matter::attribute::create(aq_cluster, chip::app::Clusters::AirQuality::Attributes::AirQuality::Id,
                                                       esp_matter::ATTRIBUTE_FLAG_NONE,
                                                       esp_matter_enum8(chip::to_underlying(chip::app::Clusters::AirQuality::AirQualityEnum::kUnknown))) != nullptr,
                         ESP_LOGE(TAG, "Failed to create AirQuality attribute"));
    ABORT_APP_ON_FAILURE(esp_matter::cluster::global::attribute::create_cluster_revision(aq_cluster, 1) != nullptr,
                         ESP_LOGE(TAG, "Failed to create AirQuality ClusterRevision"));

    // Keep CO2 on the AirQuality endpoint.
    esp_matter::cluster::carbon_dioxide_concentration_measurement::config_t co2_cfg;
    co2_cfg.measurement_medium = 0x00; // Air
    co2_cfg.feature_flags = esp_matter::cluster::concentration_measurement::feature::numeric_measurement::get_id();
    co2_cfg.features.numeric_measurement.measurement_unit = 0x00; // ppm
    ABORT_APP_ON_FAILURE(
        esp_matter::cluster::carbon_dioxide_concentration_measurement::create(aq_ep, &co2_cfg, esp_matter::CLUSTER_FLAG_SERVER) != nullptr,
        ESP_LOGE(TAG, "Failed to create CO2 Measurement cluster"));

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    /* Set OpenThread platform config */
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&config);
#endif

    // Initialize network glue early so that when CHIP brings up mDNS/DNS-SD during
    // esp_matter::start(), it's already able to bind to the OpenThread netif.
    init_network_driver();

    /* Matter start */
    err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter, err:%d", err));

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    // esp_matter core defaults FTD devices to Router role.
    // Override to Full End Device so OpenThread can promote to router only if needed.
    if (chip::DeviceLayer::ConnectivityMgr().SetThreadDeviceType(
            chip::DeviceLayer::ConnectivityManager::kThreadDeviceType_FullEndDevice) != CHIP_NO_ERROR) {
        ESP_LOGW(TAG, "Failed to set Thread device type to FullEndDevice");
    } else {
        ESP_LOGI(TAG, "Thread device type set to FullEndDevice");
    }
#endif

    if (xTaskCreate(serial_command_task, "serial_cmd", 4096, nullptr, 4, nullptr) != pdPASS) {
        ESP_LOGW(TAG, "Failed to start serial command task");
    }

    // Print commissioning codes on every boot to simplify bring-up/testing.
    ESP_LOGI(TAG, "Matter commissioning codes:");
    PrintOnboardingCodes(chip::RendezvousInformationFlags(chip::RendezvousInformationFlag::kBLE));

    // Start BSEC2 processing loop (updates split sensor endpoints).
    bsec2_app_config_t bsec_cfg = {
        .temp_endpoint = temp_endpoint_id,
        .humidity_endpoint = humidity_endpoint_id,
        .pressure_endpoint = pressure_endpoint_id,
        .air_quality_endpoint = air_quality_endpoint_id,
        .co2_endpoint = air_quality_endpoint_id,
    };
    err = bsec2_app_start(&bsec_cfg);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "BSEC2 init failed"));
}
