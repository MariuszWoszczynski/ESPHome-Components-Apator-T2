#pragma once

#include <functional>

#include "freertos/FreeRTOS.h"

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"

#include "esphome/components/spi/spi.h"

#include "packet.h"
#include "transceiver.h"
#include "apator_t2.h"

namespace esphome {
namespace wmbus_radio {

class Radio : public Component {
 public:
  void set_radio(RadioTransceiver *radio) { this->radio = radio; };

  void setup() override;
  void loop() override;
  void receive_frame();

  void add_frame_handler(std::function<void(Frame *)> &&callback);
  void on_packet(std::function<void(Packet *)> &&callback);
  bool arm_apator_period(const std::string &meter_id, uint16_t period_seconds, uint8_t version, uint8_t device_type,
                         const std::string &aes_key_hex, uint8_t attempts, uint8_t power_dbm);

 protected:
  static void wakeup_receiver_task_from_isr(TaskHandle_t *arg);
  static void receiver_task(Radio *arg);

  RadioTransceiver *radio{nullptr};
  TaskHandle_t receiver_task_handle_{nullptr};
  QueueHandle_t packet_queue_{nullptr};
  QueueHandle_t command_queue_{nullptr};

  struct PendingCommand {
    ApatorT2Frame frame;
    uint8_t attempts_left;
    uint8_t power_dbm;
  };
  PendingCommand *pending_command_{nullptr};

  bool accept_armed_command_();
  void transmit_pending_command_();

  std::vector<std::function<void(Frame *)>> frame_handlers_;

  CallbackManager<void(Packet *)> on_packet_callback_manager;
};
}  // namespace wmbus_radio
}  // namespace esphome
