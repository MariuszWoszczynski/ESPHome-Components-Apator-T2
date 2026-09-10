#include "apator_t2.h"

#include <algorithm>
#include <cstdlib>

#include "../wmbus_common/aes.h"

namespace esphome {
namespace wmbus_radio {

static int hex_value_(char value) {
  if (value >= '0' && value <= '9')
    return value - '0';
  if (value >= 'A' && value <= 'F')
    return value - 'A' + 10;
  if (value >= 'a' && value <= 'f')
    return value - 'a' + 10;
  return -1;
}

static uint16_t crc16_en13757_(const uint8_t *data, size_t length) {
  uint16_t crc = 0;
  for (size_t i = 0; i < length; i++) {
    uint8_t value = data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (((crc & 0x8000) >> 8) ^ (value & 0x80))
        crc = (crc << 1) ^ 0x3D65;
      else
        crc <<= 1;
      value <<= 1;
    }
  }
  return ~crc;
}

static bool parse_meter_id_(const std::string &text, std::array<uint8_t, 4> *bcd) {
  if (text.empty() || text.size() > 8)
    return false;
  for (char c : text)
    if (c < '0' || c > '9')
      return false;

  uint32_t value = strtoul(text.c_str(), nullptr, 10);
  for (size_t i = 0; i < bcd->size(); i++) {
    const uint8_t low = value % 10;
    value /= 10;
    const uint8_t high = value % 10;
    value /= 10;
    (*bcd)[i] = (high << 4) | low;
  }
  return value == 0;
}

static bool parse_key_(const std::string &text, std::array<uint8_t, 16> *key) {
  if (text.size() != 32)
    return false;
  for (size_t i = 0; i < key->size(); i++) {
    int high = hex_value_(text[2 * i]);
    int low = hex_value_(text[2 * i + 1]);
    if (high < 0 || low < 0)
      return false;
    (*key)[i] = (high << 4) | low;
  }
  return true;
}

static void append_crc_(std::vector<uint8_t> *out, const uint8_t *data, size_t length) {
  out->insert(out->end(), data, data + length);
  const uint16_t crc = crc16_en13757_(data, length);
  out->push_back(crc >> 8);
  out->push_back(crc & 0xFF);
}

static std::vector<uint8_t> add_format_a_crcs_(const std::vector<uint8_t> &frame) {
  std::vector<uint8_t> result;
  result.reserve(frame.size() + 8);
  append_crc_(&result, frame.data(), std::min((size_t) 10, frame.size()));
  for (size_t offset = 10; offset < frame.size(); offset += 16)
    append_crc_(&result, frame.data() + offset, std::min((size_t) 16, frame.size() - offset));
  return result;
}

static std::vector<uint8_t> manchester_encode_(const std::vector<uint8_t> &data) {
  std::vector<uint8_t> result;
  result.reserve(data.size() * 2 + 1);
  uint8_t output = 0;
  uint8_t bits = 0;
  uint8_t last_chip = 0;
  for (uint8_t value : data) {
    for (int bit = 7; bit >= 0; bit--) {
      // EN 13757 T2 O2M: zero -> 10, one -> 01.
      const uint8_t pair = (value & (1 << bit)) ? 0b01 : 0b10;
      for (int pair_bit = 1; pair_bit >= 0; pair_bit--) {
        last_chip = (pair >> pair_bit) & 1;
        output = (output << 1) | last_chip;
        if (++bits == 8) {
          result.push_back(output);
          output = 0;
          bits = 0;
        }
      }
    }
  }
  // Eight alternating postamble chips, continuing from the final data chip.
  result.push_back(last_chip ? 0x55 : 0xAA);
  return result;
}

static bool build_apator_frame_(const std::string &meter_id, uint8_t version, uint8_t device_type,
                                const std::string &aes_key_hex, uint8_t overlay_instruction,
                                const std::vector<uint8_t> &register_data, ApatorT2Frame *result) {
  if (result == nullptr)
    return false;

  std::array<uint8_t, 16> key{};
  if (!parse_meter_id_(meter_id, &result->meter_id_bcd) || !parse_key_(aes_key_hex, &key))
    return false;

  std::vector<uint8_t> command = {0x0F, 0x00, 0x00, 0x00, 0x00, overlay_instruction};
  command.insert(command.end(), register_data.begin(), register_data.end());
  const size_t padding = (16 - ((register_data.size() + 10) % 16)) % 16;
  command.insert(command.end(), padding, 0xFF);
  const uint16_t command_crc = crc16_en13757_(command.data(), command.size());
  command.push_back(command_crc >> 8);
  command.push_back(command_crc & 0xFF);

  std::vector<uint8_t> cleartext = {0x2F, 0x2F};
  cleartext.insert(cleartext.end(), command.begin(), command.end());

  const uint8_t manufacturer_low = 0x01;
  const uint8_t manufacturer_high = 0x06;
  uint8_t iv[16] = {manufacturer_low, manufacturer_high, 0, 0, 0, 0, version, device_type, 1, 1, 1, 1, 1, 1, 1, 1};
  std::copy(result->meter_id_bcd.begin(), result->meter_id_bcd.end(), iv + 2);

  std::vector<uint8_t> encrypted(cleartext.size());
  AES_CBC_encrypt_buffer(encrypted.data(), cleartext.data(), cleartext.size(), key.data(), iv);

  std::vector<uint8_t> frame(23 + encrypted.size());
  frame[0] = frame.size() - 1;
  frame[1] = 0x5B;
  frame[2] = manufacturer_low;
  frame[3] = manufacturer_high;
  frame[4] = 0x46;
  frame[5] = frame[6] = frame[7] = 0x00;
  frame[8] = 0x02;
  frame[9] = 0x03;
  frame[10] = 0x5B;
  std::copy(result->meter_id_bcd.begin(), result->meter_id_bcd.end(), frame.begin() + 11);
  frame[15] = manufacturer_low;
  frame[16] = manufacturer_high;
  frame[17] = version;
  frame[18] = device_type;
  frame[19] = 0x01;
  frame[20] = 0x00;
  frame[21] = (cleartext.size() / 16) << 4;
  frame[22] = 0x05;
  std::copy(encrypted.begin(), encrypted.end(), frame.begin() + 23);

  result->radio_payload = manchester_encode_(add_format_a_crcs_(frame));
  return true;
}

bool build_apator_period_frame(const std::string &meter_id, uint16_t period_seconds, uint8_t version,
                               uint8_t device_type, const std::string &aes_key_hex, ApatorT2Frame *result) {
  if (result == nullptr || period_seconds < 10 || period_seconds > 2550 || period_seconds % 10 != 0)
    return false;

  const uint8_t period = period_seconds / 10;
  // AT-WMBUS-16-1 register 0xB0: normal, economy-hour, economy-weekday,
  // economy-month-day and economy-month periods, each in units of 10 seconds.
  const std::vector<uint8_t> register_data = {0x00,   0xFF,   0xFF,   0x00,   0xB0,  0x05,
                                              period, period, period, period, period};

  return build_apator_frame_(meter_id, version, device_type, aes_key_hex, 0x02, register_data, result);
}

bool build_apator_period_read_frame(const std::string &meter_id, uint8_t version, uint8_t device_type,
                                    const std::string &aes_key_hex, ApatorT2Frame *result) {
  return build_apator_frame_(meter_id, version, device_type, aes_key_hex, 0x01, {0xB0}, result);
}

ApatorT2Reply parse_apator_t2_reply(const std::vector<uint8_t> &frame,
                                    const std::array<uint8_t, 4> &meter_id_bcd, const std::string &aes_key_hex) {
  ApatorT2Reply result;
  if (frame.size() < 10 || static_cast<size_t>(frame[0]) + 1 != frame.size())
    return result;
  if (frame[2] != 0x01 || frame[3] != 0x06 ||
      !std::equal(meter_id_bcd.begin(), meter_id_bcd.end(), frame.begin() + 4)) {
    result.type = ApatorReplyType::NOT_FOR_US;
    return result;
  }

  if (frame[1] == 0x00) {
    if (frame.size() <= 20)
      return result;
    const uint8_t status = frame[20];
    if ((status & 0x0F) != 0x02)
      return result;
    result.type = ApatorReplyType::WRITE_ACK;
    result.error_code = (status >> 4) & 0x07;
    return result;
  }

  if (frame[1] != 0x08 || frame.size() < 31 || (frame.size() - 15) % 16 != 0)
    return result;

  std::array<uint8_t, 16> key{};
  if (!parse_key_(aes_key_hex, &key))
    return result;
  uint8_t iv[16];
  std::copy(frame.begin() + 2, frame.begin() + 10, iv);
  std::fill(iv + 8, iv + 16, frame[11]);
  std::vector<uint8_t> cleartext(frame.size() - 15);
  AES_CBC_decrypt_buffer(cleartext.data(), const_cast<uint8_t *>(frame.data() + 15), cleartext.size(), key.data(), iv);
  if (cleartext.size() < 16 || cleartext[0] != 0x2F || cleartext[1] != 0x2F || cleartext[2] != 0x0F)
    return result;

  // AT-WMBUS-16-1 responses contain seven fixed bytes after DIF 0x0F,
  // followed by register id and its value (without a length byte).
  for (size_t offset = 10; offset + 5 < cleartext.size(); offset++) {
    if (cleartext[offset] != 0xB0)
      continue;
    result.type = ApatorReplyType::PERIOD_READ;
    result.error_code = 0;
    for (size_t i = 0; i < result.periods_seconds.size(); i++)
      result.periods_seconds[i] = cleartext[offset + 1 + i] * 10;
    return result;
  }
  return result;
}

const char *apator_error_to_string(uint8_t error_code) {
  static const char *const ERRORS[] = {"OK",          "wrong PIN",      "wrong instruction",
                                      "wrong register", "wrong data size", "wrong CRC",
                                      "too many parameters"};
  return error_code < sizeof(ERRORS) / sizeof(ERRORS[0]) ? ERRORS[error_code] : "unknown error";
}

}  // namespace wmbus_radio
}  // namespace esphome
