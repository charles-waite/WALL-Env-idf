/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <sdkconfig.h>

#include <app/clusters/network-commissioning/network-commissioning.h>
#include <platform/OpenThread/GenericNetworkCommissioningThreadDriver.h>

// This mirrors the pattern used in esp-matter ZAP examples: initialize the
// network commissioning driver instance(s) after Matter starts.

using namespace chip;
using namespace chip::DeviceLayer;

namespace {

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD && CONFIG_THREAD_NETWORK_COMMISSIONING_DRIVER
app::Clusters::NetworkCommissioning::InstanceAndDriver<NetworkCommissioning::GenericThreadDriver>
sThreadNetworkDriver(CONFIG_THREAD_NETWORK_ENDPOINT_ID);
#endif

} // namespace

extern "C" void init_network_driver()
{
#if CHIP_DEVICE_CONFIG_ENABLE_THREAD && CONFIG_THREAD_NETWORK_COMMISSIONING_DRIVER
    sThreadNetworkDriver.Init();
#endif
}

