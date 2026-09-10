#include <cassert>
#include <iomanip>
#include <sstream>

#include "../components/wmbus_radio/apator_t2.h"
#include "../components/wmbus_common/aes.h"

using esphome::wmbus_radio::ApatorT2Frame;
using esphome::wmbus_radio::build_apator_period_frame;
using esphome::wmbus_radio::build_apator_period_read_frame;
using esphome::wmbus_radio::parse_apator_t2_reply;
using esphome::wmbus_radio::ApatorReplyType;

static std::string hex(const std::vector<uint8_t> &data) {
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (uint8_t value : data)
    output << std::setw(2) << static_cast<unsigned>(value);
  return output.str();
}

int main() {
  ApatorT2Frame frame;
  assert(build_apator_period_frame("12345678", 60, 5, 7,
                                   "00000000000000000000000000000000", &frame));
  assert((frame.meter_id_bcd == std::array<uint8_t, 4>{0x78, 0x56, 0x34, 0x12}));
  assert(hex(frame.radio_payload) ==
         "a5969965aaa9aa969a96aaaaaaaaaaaaaaa6aaa55a9aa6559965956a9996a59aa9a6aaa9aa96aa99aa95aaa9aaaaa6aaaa"
         "995a655a96565956a9a96959a6995665aaa9a5a66555a5a959a66aa66a6aa59565966a59969559a69a96556a99595566a"
         "96595aa665669596a6665a59a56aa5aa996a5a5656965999a595565a6aa");

  assert(!build_apator_period_frame("12AB", 60, 5, 7,
                                    "00000000000000000000000000000000", &frame));
  assert(!build_apator_period_frame("12345678", 61, 5, 7,
                                    "00000000000000000000000000000000", &frame));

  ApatorT2Frame read_frame;
  assert(build_apator_period_read_frame("12345678", 5, 7,
                                        "00000000000000000000000000000000", &read_frame));
  assert(read_frame.radio_payload.size() < frame.radio_payload.size());

  std::vector<uint8_t> ack(21, 0);
  ack[0] = ack.size() - 1;
  ack[1] = 0x00;
  ack[2] = 0x01;
  ack[3] = 0x06;
  std::copy(read_frame.meter_id_bcd.begin(), read_frame.meter_id_bcd.end(), ack.begin() + 4);
  ack[20] = 0x02;
  auto reply = parse_apator_t2_reply(ack, read_frame.meter_id_bcd,
                                     "00000000000000000000000000000000");
  assert(reply.type == ApatorReplyType::WRITE_ACK);
  assert(reply.error_code == 0);
  ack[20] = 0x52;
  reply = parse_apator_t2_reply(ack, read_frame.meter_id_bcd,
                                "00000000000000000000000000000000");
  assert(reply.type == ApatorReplyType::WRITE_ACK);
  assert(reply.error_code == 5);

  std::vector<uint8_t> read_reply(31, 0);
  read_reply[0] = read_reply.size() - 1;
  read_reply[1] = 0x08;
  read_reply[2] = 0x01;
  read_reply[3] = 0x06;
  std::copy(read_frame.meter_id_bcd.begin(), read_frame.meter_id_bcd.end(), read_reply.begin() + 4);
  read_reply[8] = 5;
  read_reply[9] = 7;
  read_reply[10] = 0x7A;
  read_reply[11] = 3;
  std::vector<uint8_t> cleartext = {0x2F, 0x2F, 0x0F, 0x00, 0xFF, 0xFF, 0x00, 0x00,
                                    0x00, 0x00, 0xB0, 0x06, 0x06, 0x06, 0x06, 0x06};
  uint8_t iv[16];
  std::copy(read_reply.begin() + 2, read_reply.begin() + 10, iv);
  std::fill(iv + 8, iv + 16, read_reply[11]);
  std::array<uint8_t, 16> zero_key{};
  AES_CBC_encrypt_buffer(read_reply.data() + 15, cleartext.data(), cleartext.size(), zero_key.data(), iv);
  reply = parse_apator_t2_reply(read_reply, read_frame.meter_id_bcd,
                                "00000000000000000000000000000000");
  assert(reply.type == ApatorReplyType::PERIOD_READ);
  assert((reply.periods_seconds == std::array<uint16_t, 5>{60, 60, 60, 60, 60}));
}
