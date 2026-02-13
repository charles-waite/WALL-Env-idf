/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <app/server/CommissioningWindowManager.h>
#include <app/server/Dnssd.h>
#include <app/server/Server.h>
#include <app-common/zap-generated/ids/Attributes.h>
#include <app-common/zap-generated/ids/Clusters.h>
#include <platform/ConnectivityManager.h>
#include <button_gpio.h>
#include <iot_button.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_system.h>
#include <driver/usb_serial_jtag.h>
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
#include <inttypes.h>

// drivers implemented by this example
#include <drivers/bsec2_app.h>
#include <drivers/oled_sh1106.h>

static const char *TAG = "app_main";
static constexpr uint32_t kTimeSyncClusterId = chip::app::Clusters::TimeSynchronization::Id;
static constexpr int kBootButtonGpio = 9;
static bool s_reboot_after_decom = false;
static bool s_decom_in_progress = false;

enum class decom_state_t : uint8_t {
    kIdle = 0,
    kAccepted,
    kDeleteRequested,
    kFabricEvent,
    kServicesReset,
    kRebootScheduled,
};

static decom_state_t s_decom_state = decom_state_t::kIdle;

static const char *decom_state_str(decom_state_t state)
{
    switch (state) {
    case decom_state_t::kIdle:
        return "idle";
    case decom_state_t::kAccepted:
        return "accepted";
    case decom_state_t::kDeleteRequested:
        return "delete_requested";
    case decom_state_t::kFabricEvent:
        return "fabric_event";
    case decom_state_t::kServicesReset:
        return "services_reset";
    case decom_state_t::kRebootScheduled:
        return "reboot_scheduled";
    default:
        return "unknown";
    }
}

static void set_decom_state(decom_state_t next, const char *reason)
{
    s_decom_state = next;
    ESP_LOGI(TAG, "decom: state=%s (%s)", decom_state_str(next), reason ? reason : "n/a");
}

static const char *attribute_cb_type_str(esp_matter::attribute::callback_type_t type)
{
    switch (type) {
    case esp_matter::attribute::PRE_UPDATE:
        return "PRE_UPDATE";
    case esp_matter::attribute::POST_UPDATE:
        return "POST_UPDATE";
    case esp_matter::attribute::READ:
        return "READ";
    case esp_matter::attribute::WRITE:
        return "WRITE";
    default:
        return "UNKNOWN";
    }
}

extern "C" void init_network_driver();

static bool init_time_sync_cluster(esp_matter::node_t *node)
{
    if (!node) {
        return false;
    }
    esp_matter::endpoint_t *root = esp_matter::endpoint::get(node, 0);
    if (!root) {
        return false;
    }

    esp_matter::cluster::time_synchronization::config_t cfg;
    esp_matter::cluster_t *cluster =
        esp_matter::cluster::time_synchronization::create(root, &cfg, esp_matter::CLUSTER_FLAG_SERVER);
    return cluster != nullptr;
}

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
        set_decom_state(decom_state_t::kAccepted, "command accepted on CHIP thread");
        if (before == 0) {
            ESP_LOGI(TAG, "decom: no fabrics to remove");
            s_decom_in_progress = false;
            s_reboot_after_decom = false;
            set_decom_state(decom_state_t::kIdle, "no fabrics");
            return;
        }

        ESP_LOGI(TAG, "decom: removing %u fabric(s)", before);
        s_decom_in_progress = true;
        s_reboot_after_decom = true;
        fabric_table.DeleteAllFabrics();
        set_decom_state(decom_state_t::kDeleteRequested, "DeleteAllFabrics issued");
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

    if (strcmp(cmd, "profile") == 0) {
        ESP_LOGI(TAG, "Temp profile: %s (offset %.2fC)",
                 bsec2_app_get_temp_profile(), bsec2_app_get_temp_offset_c());
        ESP_LOGI(TAG, "Use: profile v1 | profile v2");
        return;
    }

    if (strncmp(cmd, "profile ", 8) == 0) {
        const char *name = cmd + 8;
        esp_err_t err = bsec2_app_set_temp_profile(name, true);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Temp profile set: %s (offset %.2fC)",
                     bsec2_app_get_temp_profile(), bsec2_app_get_temp_offset_c());
        } else {
            ESP_LOGW(TAG, "Unknown profile '%s'. Valid: v1, v2", name);
        }
        return;
    }

    if (strcmp(cmd, "help") == 0) {
        ESP_LOGI(TAG, "serial commands: decom, decommission, factoryreset, reset, profile, profile v1, profile v2, help");
        return;
    }

    ESP_LOGW(TAG, "Unknown serial command: '%s' (try 'help')", cmd);
}

static bool ensure_usb_serial_jtag_driver()
{
    if (usb_serial_jtag_is_driver_installed()) {
        return true;
    }
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "USB Serial JTAG install failed, err=%d", err);
        return false;
    }
    return true;
}

static int read_console_char()
{
    uint8_t ch = 0;

    // Try USB-Serial-JTAG first (cu.usbmodem* monitor path).
    if (ensure_usb_serial_jtag_driver()) {
        int n = usb_serial_jtag_read_bytes(&ch, 1, 0);
        if (n == 1) {
            return ch;
        }
    }

#if CONFIG_ESP_CONSOLE_UART
    // Fallback to UART0 console only when UART console is enabled.
    int n = uart_read_bytes(UART_NUM_0, &ch, 1, 0);
    if (n == 1) {
        return ch;
    }
#endif

    return EOF;
}

static void serial_command_task(void *arg)
{
    (void) arg;
    static constexpr size_t kCmdMax = 32;
    char cmd[kCmdMax] = {0};
    size_t idx = 0;

    ESP_LOGI(TAG, "Serial command task ready. Type 'decom' to decommission.");
    while (true) {
        int ch = read_console_char();
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
    button_handle_t push_button = nullptr;
    const button_config_t btn_cfg = {
        .long_press_time = 5000,
        .short_press_time = 180,
    };
    const button_gpio_config_t btn_gpio_cfg = {
        .gpio_num = kBootButtonGpio,
        .active_level = 0,
        .enable_power_save = false,
        .disable_pull = false,
    };
    esp_err_t err = iot_button_new_gpio_device(&btn_cfg, &btn_gpio_cfg, &push_button);
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

static void reset_discovery_after_decommission()
{
    auto &commissionMgr = chip::Server::GetInstance().GetCommissioningWindowManager();
    if (commissionMgr.IsCommissioningWindowOpen()) {
        ESP_LOGI(TAG, "decom: closing existing commissioning window before re-open");
        commissionMgr.CloseCommissioningWindow();
    }

    open_commissioning_window_if_necessary();

    // Force DNS-SD service refresh after fabric removal to avoid stale operational advertisements.
    chip::app::DnssdServer::Instance().StartServer(chip::Dnssd::CommissioningMode::kEnabledBasic);
    ESP_LOGI(TAG, "decom: discovery services restart requested");
}

static void decommission_reboot_timer_cb(chip::System::Layer *systemLayer, void *appState)
{
    (void) systemLayer;
    (void) appState;
    ESP_LOGW(TAG, "decom: rebooting to clear stale sessions");
    esp_restart();
}

static void update_oled_commissioning_state()
{
    const bool is_uncommissioned = (chip::Server::GetInstance().GetFabricTable().FabricCount() == 0);
    oled_sh1106_set_commissioning_active(is_uncommissioned);
}

static void push_commissioning_codes_to_oled()
{
    char qr_buf[128] = {0};
    char manual_buf[32] = {0};
    chip::MutableCharSpan qr_span(qr_buf);
    chip::MutableCharSpan manual_span(manual_buf);

    CHIP_ERROR qr_err = GetQRCode(qr_span, chip::RendezvousInformationFlags(chip::RendezvousInformationFlag::kBLE));
    CHIP_ERROR man_err =
        GetManualPairingCode(manual_span, chip::RendezvousInformationFlags(chip::RendezvousInformationFlag::kBLE));

    if (qr_err != CHIP_NO_ERROR) {
        ESP_LOGW(TAG, "Failed to generate QR payload for OLED, err=%" CHIP_ERROR_FORMAT, qr_err.Format());
        qr_buf[0] = '\0';
    }
    if (man_err != CHIP_NO_ERROR) {
        ESP_LOGW(TAG, "Failed to generate manual code for OLED, err=%" CHIP_ERROR_FORMAT, man_err.Format());
        manual_buf[0] = '\0';
    }

    oled_sh1106_set_commissioning_codes(qr_buf, manual_buf);
}

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        update_oled_commissioning_state();
        break;

    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(TAG, "Commissioning failed, fail safe timer expired");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
        ESP_LOGI(TAG, "Fabric removed successfully");
        if (s_decom_in_progress) {
            auto &fabric_table = chip::Server::GetInstance().GetFabricTable();
            ESP_LOGI(TAG, "decom: fabric removed event, remaining fabrics=%u", fabric_table.FabricCount());
            set_decom_state(decom_state_t::kFabricEvent, "fabric removed event");
        }
        open_commissioning_window_if_necessary();
        push_commissioning_codes_to_oled();
        update_oled_commissioning_state();
        if (s_reboot_after_decom && chip::Server::GetInstance().GetFabricTable().FabricCount() == 0) {
            set_decom_state(decom_state_t::kServicesReset, "all fabrics removed");
            reset_discovery_after_decommission();
            s_decom_in_progress = false;
            s_reboot_after_decom = false;
            set_decom_state(decom_state_t::kRebootScheduled, "reboot in 1200 ms");
            chip::DeviceLayer::SystemLayer().StartTimer(chip::System::Clock::Milliseconds32(1200),
                                                        decommission_reboot_timer_cb, nullptr);
        }
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
    if (cluster_id == kTimeSyncClusterId) {
        ESP_LOGI(TAG, "TimeSync attr event: type=%s ep=%u attr=0x%08" PRIX32 " val_type=%d",
                 attribute_cb_type_str(type), endpoint_id, attribute_id, val ? static_cast<int>(val->type) : -1);

        if (!val) {
            return ESP_OK;
        }

        if (attribute_id == chip::app::Clusters::TimeSynchronization::Attributes::UTCTime::Id) {
            if (val->type == ESP_MATTER_VAL_TYPE_UINT64 || val->type == ESP_MATTER_VAL_TYPE_NULLABLE_UINT64) {
                ESP_LOGI(TAG, "TimeSync UTCTime value=%" PRIu64, val->val.u64);
            }
        } else if (attribute_id == chip::app::Clusters::TimeSynchronization::Attributes::Granularity::Id) {
            if (val->type == ESP_MATTER_VAL_TYPE_ENUM8 || val->type == ESP_MATTER_VAL_TYPE_NULLABLE_ENUM8) {
                ESP_LOGI(TAG, "TimeSync Granularity=%u", static_cast<unsigned>(val->val.u8));
            }
        } else if (attribute_id == chip::app::Clusters::TimeSynchronization::Attributes::TimeSource::Id) {
            if (val->type == ESP_MATTER_VAL_TYPE_ENUM8 || val->type == ESP_MATTER_VAL_TYPE_NULLABLE_ENUM8) {
                ESP_LOGI(TAG, "TimeSync TimeSource=%u", static_cast<unsigned>(val->val.u8));
            }
        } else if (attribute_id == chip::app::Clusters::TimeSynchronization::Attributes::LocalTime::Id) {
            if (val->type == ESP_MATTER_VAL_TYPE_UINT64 || val->type == ESP_MATTER_VAL_TYPE_NULLABLE_UINT64) {
                ESP_LOGI(TAG, "TimeSync LocalTime=%" PRIu64, val->val.u64);
            }
        }
    }

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
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS init returned %d; erasing NVS partition and retrying", nvs_err);
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ABORT_APP_ON_FAILURE(nvs_err == ESP_OK, ESP_LOGE(TAG, "Failed to initialize NVS, err:%d", nvs_err));

    // Keep monitor output readable: suppress per-attribute value dumps from esp-matter.
    esp_log_level_set("esp_matter_attribute", ESP_LOG_WARN);

    /* Initialize push button on the dev-kit to reset the device */
    esp_err_t err = factory_reset_button_register();
    ABORT_APP_ON_FAILURE(ESP_OK == err, ESP_LOGE(TAG, "Failed to initialize reset button, err:%d", err));

    // Optional OLED splash/status task (does not abort app on failure).
    (void) oled_sh1106_start();

    /* Create a Matter node (Root Node device type on endpoint 0 is added automatically) */
    esp_matter::node::config_t node_config;
    esp_matter::node_t *node = esp_matter::node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));

    ABORT_APP_ON_FAILURE(init_time_sync_cluster(node), ESP_LOGE(TAG, "Failed to create Time Synchronization cluster on endpoint 0"));

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
    push_commissioning_codes_to_oled();
    update_oled_commissioning_state();

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
