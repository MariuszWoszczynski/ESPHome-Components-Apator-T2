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

enum class ApatorReplyType : uint8_t { INVALID, NOT_FOR_US, WRITE_ACK, PERIOD_READ };

struct ApatorT2Reply {
  ApatorReplyType type{ApatorReplyType::INVALID};
  uint8_t error_code{0xFF};
  std::array<uint16_t, 5> periods_seconds{};
};

// Reproduces the AT-WMBUS-16-1 period-write telegram prepared by inkaSOID.
// The returned payload already contains format-A DLL CRCs, Manchester coding
// and an alternating postamble.  The radio still has to add the T2 O2M preamble
// and sync word.
bool build_apator_period_frame(const std::string &meter_id, uint16_t period_seconds, uint8_t version,
                               uint8_t device_type, const std::string &aes_key_hex, ApatorT2Frame *result);

// Builds the matching register 0xB0 read request used to verify a write.
bool build_apator_period_read_frame(const std::string &meter_id, uint8_t version, uint8_t device_type,
                                    const std::string &aes_key_hex, ApatorT2Frame *result);

// Parses a CRC-checked, decoded format-A link frame from an AT-WMBUS-16-1.
ApatorT2Reply parse_apator_t2_reply(const std::vector<uint8_t> &frame, const std::array<uint8_t, 4> &meter_id_bcd,
                                    const std::string &aes_key_hex);

const char *apator_error_to_string(uint8_t error_code);

}  // namespace wmbus_radio
}  // namespace esphome
