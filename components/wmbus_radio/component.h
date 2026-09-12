#pragma once

#include <functional>
#include <memory>

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
  void on_apator_result(std::function<void(std::string, uint16_t, uint16_t)> &&callback);
  void on_apator_read_result(
      std::function<void(std::string, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t)> &&callback);
  bool arm_apator_period(const std::string &meter_id, uint16_t period_seconds, uint8_t version, uint8_t device_type,
                         const std::string &aes_key_hex, uint8_t attempts, uint8_t power_dbm);
  bool arm_apator_period_read(const std::string &meter_id, uint8_t version, uint8_t device_type,
                              const std::string &aes_key_hex, uint8_t attempts, uint8_t power_dbm);

 protected:
  static void wakeup_receiver_task_from_isr(TaskHandle_t *arg);
  static void receiver_task(Radio *arg);

  RadioTransceiver *radio{nullptr};
  TaskHandle_t receiver_task_handle_{nullptr};
  QueueHandle_t packet_queue_{nullptr};
  QueueHandle_t command_queue_{nullptr};
  QueueHandle_t result_queue_{nullptr};
  QueueHandle_t read_result_queue_{nullptr};

  struct PendingCommand {
    enum class Stage : uint8_t { WRITE, VERIFY, READ_ONLY } stage{Stage::WRITE};
    ApatorT2Frame write_frame;
    ApatorT2Frame read_frame;
    std::string aes_key_hex;
    uint16_t desired_period_seconds;
    uint8_t attempts_left;
    uint8_t power_dbm;
  };
  PendingCommand *pending_command_{nullptr};

  struct ProgrammingResult {
    std::string result;
    uint16_t desired_period;
    uint16_t actual_period;
  };

  struct ReadResult {
    std::string result;
    std::array<uint16_t, 5> periods;
  };

  bool accept_armed_command_();
  void transmit_pending_command_();
  std::unique_ptr<Packet> receive_response_packet_(TickType_t timeout);
  void command_failed_(const char *reason);
  void finish_command_(const std::string &result, uint16_t actual_period = 0);
  void finish_read_(const std::string &result, const std::array<uint16_t, 5> &periods = {});

  std::vector<std::function<void(Frame *)>> frame_handlers_;

  CallbackManager<void(Packet *)> on_packet_callback_manager;
  CallbackManager<void(std::string, uint16_t, uint16_t)> on_apator_result_callback_manager_;
  CallbackManager<void(std::string, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t)>
      on_apator_read_result_callback_manager_;
};
}  // namespace wmbus_radio
}  // namespace esphome
