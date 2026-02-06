/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <sdkconfig.h>

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <mdns.h>

#include <app/clusters/network-commissioning/network-commissioning.h>
#include <esp_openthread_netif_glue.h>
#include <platform/OpenThread/GenericNetworkCommissioningThreadDriver.h>

// This mirrors the pattern used in esp-matter ZAP examples: initialize the
// network commissioning driver instance(s) after Matter starts.

using namespace chip;
using namespace chip::DeviceLayer;

static const char *TAG = "netif";

namespace {

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD && CONFIG_THREAD_NETWORK_COMMISSIONING_DRIVER
app::Clusters::NetworkCommissioning::InstanceAndDriver<NetworkCommissioning::GenericThreadDriver>
sThreadNetworkDriver(CONFIG_THREAD_NETWORK_ENDPOINT_ID);
#endif

static void mdns_thread_netif_register_task(void *arg)
{
    (void) arg;

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD && !CONFIG_USE_MINIMAL_MDNS
    // CHIP initializes IDF mDNS internally. However, that init may be slightly
    // delayed relative to app_main, so retry a few times.
    for (int i = 0; i < 20; i++) {
        esp_netif_t *ot_netif = esp_openthread_get_netif();
        if (!ot_netif) {
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }

        esp_err_t err = mdns_register_netif(ot_netif);
        if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
            // ESP_ERR_INVALID_STATE can mean "already registered" or "mdns not running".
            // Attempt enabling anyway; if mdns isn't running yet we'll retry.
            err = mdns_netif_action(ot_netif, static_cast<mdns_event_actions_t>(MDNS_EVENT_ENABLE_IP6 | MDNS_EVENT_ANNOUNCE_IP6));
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "mDNS enabled on OpenThread netif");
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(250));
    }
#endif

    vTaskDelete(nullptr);
}

} // namespace

extern "C" void init_network_driver()
{
#if CHIP_DEVICE_CONFIG_ENABLE_THREAD && CONFIG_THREAD_NETWORK_COMMISSIONING_DRIVER
    sThreadNetworkDriver.Init();
#endif

    // With Wi‑Fi disabled, ensure mDNS is bound to the OpenThread netif so DNS-SD
    // advertising works during commissioning.
    (void) xTaskCreate(mdns_thread_netif_register_task, "mdns_ot", 3072, nullptr, 2, nullptr);
}
