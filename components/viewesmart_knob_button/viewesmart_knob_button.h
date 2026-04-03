#pragma once

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace viewesmart_knob_button {

class VieweSmartKnobButton : public binary_sensor::BinarySensor, public Component {
 public:
  void set_pin(GPIOPin *pin) { this->pin_ = pin; }
  void set_debounce_time(uint32_t debounce_time) { this->debounce_time_ms_ = debounce_time; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;

 protected:
  GPIOPin *pin_{nullptr};
  uint32_t debounce_time_ms_{10};
  bool raw_state_{false};
  bool stable_state_{false};
  uint32_t last_state_change_ms_{0};
  bool initialized_{false};
};

}  // namespace viewesmart_knob_button
}  // namespace esphome

