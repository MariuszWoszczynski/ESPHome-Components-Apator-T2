#include "component.h"

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

  ASSERT_SETUP(xTaskCreate((TaskFunction_t) this->receiver_task, "radio_recv", 3 * 1024, this, 2,
                           &(this->receiver_task_handle_)));

  ESP_LOGI(TAG, "Receiver task created [%p]", this->receiver_task_handle_);

  this->radio->attach_data_interrupt(Radio::wakeup_receiver_task_from_isr, &(this->receiver_task_handle_));
}

void Radio::loop() {
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

  if (!this->radio->read(packet->rx_data_ptr(), packet->rx_capacity())) {
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

  if (!this->radio->read(packet->rx_data_ptr(), packet->rx_capacity())) {
    ESP_LOGW(TAG, "Failed to read data");
    return;
  }

  if (this->pending_command_ != nullptr && packet->matches_meter_id(this->pending_command_->frame.meter_id_bcd)) {
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
  const bool sent =
      this->radio->transmit_t2(this->pending_command_->frame.radio_payload, this->pending_command_->power_dbm);
  if (sent && this->pending_command_->attempts_left > 0)
    this->pending_command_->attempts_left--;
  if (!sent || this->pending_command_->attempts_left == 0) {
    ESP_LOGI(TAG, "Apator T2 command finished (%s)", sent ? "attempt limit reached" : "radio error");
    delete this->pending_command_;
    this->pending_command_ = nullptr;
  } else {
    ESP_LOGI(TAG, "Apator T2 command remains armed for %u more matching telegram(s)",
             this->pending_command_->attempts_left);
  }
}

bool Radio::arm_apator_period(const std::string &meter_id, uint16_t period_seconds, uint8_t version,
                              uint8_t device_type, const std::string &aes_key_hex, uint8_t attempts,
                              uint8_t power_dbm) {
  auto *command = new PendingCommand();
  command->attempts_left = attempts;
  command->power_dbm = power_dbm;
  if (attempts == 0 ||
      !build_apator_period_frame(meter_id, period_seconds, version, device_type, aes_key_hex, &command->frame)) {
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

}  // namespace wmbus_radio
}  // namespace esphome
