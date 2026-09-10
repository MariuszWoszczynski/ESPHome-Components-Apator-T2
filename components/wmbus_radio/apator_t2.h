#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace esphome {
namespace wmbus_radio {

struct ApatorT2Frame {
  std::array<uint8_t, 4> meter_id_bcd{};
  std::vector<uint8_t> radio_payload;
};

// Reproduces the AT-WMBUS-16-1 period-write telegram prepared by inkaSOID.
// The returned payload already contains format-A DLL CRCs, Manchester coding
// and an alternating postamble.  The radio still has to add the T2 O2M preamble
// and sync word.
bool build_apator_period_frame(const std::string &meter_id, uint16_t period_seconds, uint8_t version,
                               uint8_t device_type, const std::string &aes_key_hex, ApatorT2Frame *result);

}  // namespace wmbus_radio
}  // namespace esphome
