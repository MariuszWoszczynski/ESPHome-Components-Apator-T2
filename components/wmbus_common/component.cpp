#include "component.h"

#include "meters.h"

#include "esphome/core/defines.h"
#include "esphome/core/log.h"

namespace esphome {
namespace wmbus_common {
static const char *TAG = "wmbus_common";

std::vector<std::string> sorted_driver_names() {
  std::vector<std::string> driver_names;
  driver_names.reserve(allDrivers().size());
  for (auto &driver : allDrivers())
    driver_names.push_back(driver->name().str());
  std::sort(driver_names.begin(), driver_names.end());
  return driver_names;
}

void WMBusCommon::dump_config() {
  // Driver registration is performed by static initializers. Never touch the
  // registry from another global initializer because cross-TU initialization
  // order is unspecified and changes when new source files (such as T2) are
  // linked into the firmware.
  const auto driver_names = sorted_driver_names();

  ESP_LOGCONFIG(TAG, "wM-Bus Component:");
  ESP_LOGCONFIG(TAG, "  wmbusmeters version: %s", WMBUSMETERS_TAG);
  ESP_LOGCONFIG(TAG, "  Loaded drivers:");
  for (auto &driver : driver_names)
    ESP_LOGCONFIG(TAG, ("    - " + driver).c_str());
}

}  // namespace wmbus_common
}  // namespace esphome