#pragma once

#include "esphome/core/automation.h"
#include "component.h"
#include "packet.h"

namespace esphome {
namespace wmbus_radio {
class FrameTrigger : public Trigger<Frame *> {
 public:
  explicit FrameTrigger(wmbus_radio::Radio *radio, bool mark_handled) {
    radio->add_frame_handler([this, mark_handled](Frame *frame) {
      this->trigger(frame);
      if (mark_handled)
        frame->mark_as_handled();
    });
  }
};

class PacketTrigger : public Trigger<Packet *> {
 public:
  explicit PacketTrigger(wmbus_radio::Radio *radio) {
    radio->on_packet([this](Packet *packet) { this->trigger(packet); });
  }
};

template<typename... Ts> class ApatorSetPeriodAction : public Action<Ts...> {
 public:
  explicit ApatorSetPeriodAction(Radio *parent) : parent_(parent) {}

  TEMPLATABLE_VALUE(std::string, meter_id)
  TEMPLATABLE_VALUE(uint16_t, period_seconds)
  TEMPLATABLE_VALUE(uint8_t, version)
  TEMPLATABLE_VALUE(uint8_t, device_type)
  TEMPLATABLE_VALUE(std::string, aes_key)
  TEMPLATABLE_VALUE(uint8_t, attempts)
  TEMPLATABLE_VALUE(uint8_t, power_dbm)

  void play(const Ts &...x) override {
    this->parent_->arm_apator_period(this->meter_id_.value(x...), this->period_seconds_.value(x...),
                                     this->version_.value(x...), this->device_type_.value(x...),
                                     this->aes_key_.value(x...), this->attempts_.value(x...),
                                     this->power_dbm_.value(x...));
  }

 protected:
  Radio *parent_;
};

}  // namespace wmbus_radio
}  // namespace esphome
