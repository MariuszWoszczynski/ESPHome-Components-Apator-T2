#pragma once
#include "transceiver.h"

namespace esphome {
namespace wmbus_radio {
class SX1276 : public RadioTransceiver {
 public:
  void setup() override;
  bool read(uint8_t *buffer, size_t length) override;
  bool transmit_t2(const std::vector<uint8_t> &payload, uint8_t power_dbm) override;
  void restart_rx() override;
  int8_t get_rssi() override;
  const char *get_name() override;

 protected:
  void spi_write_burst_(uint8_t address, const uint8_t *data, size_t length);
};
}  // namespace wmbus_radio
}  // namespace esphome
