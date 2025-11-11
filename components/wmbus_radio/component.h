#pragma once

#include <functional>

#include "freertos/FreeRTOS.h"

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"

#include "esphome/components/spi/spi.h"
#include "esphome/components/wmbus_common/wmbus.h"
#include "esphome/components/text_sensor/text_sensor.h"

#include "packet.h"
#include "transceiver.h"

namespace esphome {
namespace wmbus_radio {

class Radio : public Component {
public:
  void set_radio(RadioTransceiver *radio) { this->radio = radio; };

  void setup() override;
  void loop() override;
  void receive_frame();

  void add_frame_handler(std::function<void(Frame *)> &&callback);
  void set_status_sensor(text_sensor::TextSensor *sensor) {
    this->status_sensor_ = sensor;
  }
  void publish_status(const std::string &status);

protected:
  static void wakeup_receiver_task_from_isr(TaskHandle_t *arg);
  static void receiver_task(Radio *arg);

  RadioTransceiver *radio{nullptr};
  TaskHandle_t receiver_task_handle_{nullptr};
  QueueHandle_t packet_queue_{nullptr};

  std::vector<std::function<void(Frame *)>> handlers_;
  text_sensor::TextSensor *status_sensor_{nullptr};
};
} // namespace wmbus_radio
} // namespace esphome
