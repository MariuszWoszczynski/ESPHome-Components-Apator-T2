#include "component.h"

#include <algorithm>

#include "freertos/task.h"
#include "freertos/queue.h"

#include "esphome/core/helpers.h"

#define ASSERT(expr, expected, before_exit) \
  { \
    auto result = (expr); \
    if (!!result != expected) { \
      ESP_LOGE(TAG, "Assertion failed: %s -> %d", #expr, result); \
      before_exit; \
      return; \
    } \
  }

#define ASSERT_SETUP(expr) ASSERT(expr, 1, this->mark_failed())

namespace esphome {
namespace wmbus_radio {
static const char *TAG = "wmbus";

void Radio::setup() {
  ASSERT_SETUP(this->packet_queue_ = xQueueCreate(3, sizeof(Packet *)));
  ASSERT_SETUP(this->command_queue_ = xQueueCreate(3, sizeof(PendingCommand *)));
  ASSERT_SETUP(this->result_queue_ = xQueueCreate(3, sizeof(ProgrammingResult *)));

  ASSERT_SETUP(xTaskCreate((TaskFunction_t) this->receiver_task, "radio_recv", 3 * 1024, this, 2,
                           &(this->receiver_task_handle_)));

  ESP_LOGI(TAG, "Receiver task created [%p]", this->receiver_task_handle_);

  this->radio->attach_data_interrupt(Radio::wakeup_receiver_task_from_isr, &(this->receiver_task_handle_));
}

void Radio::loop() {
  ProgrammingResult *result;
  while (xQueueReceive(this->result_queue_, &result, 0) == pdPASS) {
    this->on_apator_result_callback_manager_.call(result->result, result->desired_period, result->actual_period);
    delete result;
  }

  Packet *p;
  if (xQueueReceive(this->packet_queue_, &p, 0) != pdPASS)
    return;

  this->on_packet_callback_manager(p);

  auto frame = p->convert_to_frame();

  if (!frame)
    return;

  ESP_LOGI(TAG, "Frame created (%zu bytes) [RSSI: %d, mode:%s%s]", frame->data().size(), frame->rssi(),
           toString(frame->link_mode()), toString(frame->block_type()));

  uint8_t packet_handled = 0;
  for (auto &handler : this->frame_handlers_)
    handler(&frame.value());

  ESP_LOGI(TAG, "Telegram handled by %d handlers", frame->frame_handlers_count());
}

void IRAM_ATTR Radio::wakeup_receiver_task_from_isr(TaskHandle_t *arg) {
  BaseType_t xHigherPriorityTaskWoken;
  vTaskNotifyGiveFromISR(*arg, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void Radio::receive_frame() {
  this->accept_armed_command_();

  this->radio->restart_rx();

  auto packet = std::make_unique<Packet>();

  if (!ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(60000))) {
    ESP_LOGD(TAG, "Radio interrupt timeout");
    return;
  }

  if (this->accept_armed_command_())
    return;

  size_t rx_length;
  uint8_t *rx_buffer = packet->prepare_rx_buffer(&rx_length);
  if (!this->radio->read(rx_buffer, rx_length)) {
    ESP_LOGV(TAG, "Failed to read preamble");
    return;
  }
  if (!packet->validate_preamble()) {
    ESP_LOGV(TAG, "Received invalid preamble: [%s]", format_hex_pretty(packet->get_raw_data()).c_str());
    return;
  }

  if (!packet->calculate_payload_size()) {
    ESP_LOGV(TAG, "Cannot calculate payload size");
    return;
  }

  packet->set_rssi(this->radio->get_rssi());

  rx_buffer = packet->prepare_rx_buffer(&rx_length);
  if (!this->radio->read(rx_buffer, rx_length)) {
    ESP_LOGW(TAG, "Failed to read data");
    return;
  }

  if (this->pending_command_ != nullptr &&
      packet->matches_meter_id(this->pending_command_->write_frame.meter_id_bcd)) {
    ESP_LOGI(TAG, "Target Apator telegram received; replying in the T2 window");
    // T2 specifies a 2 ms minimum acknowledgement delay after the uplink.
    delay_microseconds_safe(2000);
    this->transmit_pending_command_();
  }

  auto packet_ptr = packet.get();

  if (xQueueSend(this->packet_queue_, &packet_ptr, 0) == pdTRUE) {
    ESP_LOGV(TAG, "Queue items: %zu", uxQueueMessagesWaiting(this->packet_queue_));
    ESP_LOGV(TAG, "Queue send success");
    packet.release();
  } else
    ESP_LOGW(TAG, "Queue send failed");
}

bool Radio::accept_armed_command_() {
  PendingCommand *command = nullptr;
  bool accepted = false;
  while (xQueueReceive(this->command_queue_, &command, 0) == pdTRUE) {
    delete this->pending_command_;
    this->pending_command_ = command;
    ESP_LOGI(TAG, "Apator T2 command armed; waiting for the target meter telegram");
    accepted = true;
  }
  return accepted;
}

void Radio::transmit_pending_command_() {
  if (this->pending_command_ == nullptr)
    return;
  const bool is_write = this->pending_command_->stage == PendingCommand::Stage::WRITE;
  const auto &tx_frame = is_write ? this->pending_command_->write_frame : this->pending_command_->read_frame;
  if (!this->radio->transmit_t2(tx_frame.radio_payload, this->pending_command_->power_dbm)) {
    this->command_failed_("radio_error");
    return;
  }

  const uint32_t started = millis();
  while (millis() - started < 500) {
    const TickType_t remaining = pdMS_TO_TICKS(500 - (millis() - started));
    auto response = this->receive_response_packet_(remaining);
    if (!response)
      break;
    std::vector<uint8_t> logical_frame;
    if (!response->decode_t1_format_a(&logical_frame)) {
      ESP_LOGW(TAG, "Ignoring Apator response with invalid 3-of-6 coding or DLL CRC");
      continue;
    }
    const auto reply = parse_apator_t2_reply(logical_frame, this->pending_command_->write_frame.meter_id_bcd,
                                             this->pending_command_->aes_key_hex);
    if (reply.type == ApatorReplyType::NOT_FOR_US)
      continue;
    if (is_write && reply.type == ApatorReplyType::WRITE_ACK) {
      if (reply.error_code != 0) {
        ESP_LOGE(TAG, "Apator rejected write: %s (%u)", apator_error_to_string(reply.error_code), reply.error_code);
        this->finish_command_(std::string("overlay_error_") + apator_error_to_string(reply.error_code));
        return;
      }
      ESP_LOGI(TAG, "Apator accepted register 0xB0 write; readback armed for the next meter telegram");
      this->pending_command_->stage = PendingCommand::Stage::VERIFY;
      return;
    }
    if (!is_write && reply.type == ApatorReplyType::PERIOD_READ) {
      const uint16_t actual = reply.periods_seconds[0];
      const bool all_match = std::all_of(reply.periods_seconds.begin(), reply.periods_seconds.end(),
                                         [this](uint16_t value) {
                                           return value == this->pending_command_->desired_period_seconds;
                                         });
      if (all_match) {
        ESP_LOGI(TAG, "Apator register 0xB0 verified: %u seconds", actual);
        this->finish_command_("verified", actual);
      } else {
        ESP_LOGE(TAG, "Apator register 0xB0 readback differs (normal period: %u seconds)", actual);
        this->finish_command_("readback_mismatch", actual);
      }
      return;
    }
    ESP_LOGW(TAG, "Received an unexpected Apator response while waiting for %s", is_write ? "write ACK" : "readback");
  }

  this->command_failed_(is_write ? "write_ack_timeout" : "readback_timeout");
}

std::unique_ptr<Packet> Radio::receive_response_packet_(TickType_t timeout) {
  if (!ulTaskNotifyTake(pdTRUE, timeout))
    return nullptr;
  auto packet = std::make_unique<Packet>();
  size_t rx_length;
  uint8_t *rx_buffer = packet->prepare_rx_buffer(&rx_length);
  if (!this->radio->read(rx_buffer, rx_length) || !packet->validate_preamble() || !packet->calculate_payload_size())
    return nullptr;
  packet->set_rssi(this->radio->get_rssi());
  rx_buffer = packet->prepare_rx_buffer(&rx_length);
  if (!this->radio->read(rx_buffer, rx_length))
    return nullptr;
  return packet;
}

void Radio::command_failed_(const char *reason) {
  if (this->pending_command_ == nullptr)
    return;
  if (this->pending_command_->attempts_left > 0)
    this->pending_command_->attempts_left--;
  if (this->pending_command_->attempts_left == 0) {
    ESP_LOGE(TAG, "Apator transaction failed: %s", reason);
    this->finish_command_(reason);
  } else {
    ESP_LOGW(TAG, "Apator %s; retry armed for %u more target telegram(s)", reason,
             this->pending_command_->attempts_left);
  }
}

void Radio::finish_command_(const std::string &result, uint16_t actual_period) {
  if (this->pending_command_ == nullptr)
    return;
  const uint16_t desired = this->pending_command_->desired_period_seconds;
  auto *queued_result = new ProgrammingResult{result, desired, actual_period};
  if (xQueueSend(this->result_queue_, &queued_result, 0) != pdTRUE) {
    ESP_LOGW(TAG, "Apator result queue is full");
    delete queued_result;
  }
  delete this->pending_command_;
  this->pending_command_ = nullptr;
}

bool Radio::arm_apator_period(const std::string &meter_id, uint16_t period_seconds, uint8_t version,
                              uint8_t device_type, const std::string &aes_key_hex, uint8_t attempts,
                              uint8_t power_dbm) {
  auto *command = new PendingCommand();
  command->desired_period_seconds = period_seconds;
  command->aes_key_hex = aes_key_hex;
  command->attempts_left = attempts;
  command->power_dbm = power_dbm;
  if (attempts == 0 ||
      !build_apator_period_frame(meter_id, period_seconds, version, device_type, aes_key_hex,
                                 &command->write_frame) ||
      !build_apator_period_read_frame(meter_id, version, device_type, aes_key_hex, &command->read_frame)) {
    delete command;
    ESP_LOGE(TAG, "Invalid Apator T2 command parameters");
    return false;
  }
  if (xQueueSend(this->command_queue_, &command, 0) != pdTRUE) {
    delete command;
    ESP_LOGE(TAG, "Apator T2 command queue is full");
    return false;
  }
  xTaskNotifyGive(this->receiver_task_handle_);
  return true;
}

void Radio::receiver_task(Radio *arg) {
  ESP_LOGE(TAG, "Hello from radio task!");
  int counter = 0;
  while (true)
    arg->receive_frame();
}

void Radio::add_frame_handler(std::function<void(Frame *)> &&callback) {
  this->frame_handlers_.push_back(std::move(callback));
}

void Radio::on_packet(std::function<void(Packet *)> &&callback) {
  this->on_packet_callback_manager.add(std::move(callback));
}

void Radio::on_apator_result(std::function<void(std::string, uint16_t, uint16_t)> &&callback) {
  this->on_apator_result_callback_manager_.add(std::move(callback));
}

}  // namespace wmbus_radio
}  // namespace esphome
