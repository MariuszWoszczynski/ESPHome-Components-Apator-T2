#include <cassert>
#include <iomanip>
#include <sstream>

#include "../components/wmbus_radio/apator_t2.h"

using esphome::wmbus_radio::ApatorT2Frame;
using esphome::wmbus_radio::build_apator_period_frame;

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
}
