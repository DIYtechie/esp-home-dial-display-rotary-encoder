#include "viewesmart_knob_button.h"

#include "esphome/core/log.h"

namespace esphome {
namespace viewesmart_knob_button {

static const char *const TAG = "viewesmart_button";

void VieweSmartKnobButton::setup() {
  this->pin_->setup();
  this->raw_state_ = this->pin_->digital_read();
  this->stable_state_ = this->raw_state_;
  this->last_state_change_ms_ = millis();
  this->initialized_ = true;
  this->publish_initial_state(this->stable_state_);
}

void VieweSmartKnobButton::loop() {
  const bool current_raw = this->pin_->digital_read();
  const uint32_t now = millis();

  if (current_raw != this->raw_state_) {
    this->raw_state_ = current_raw;
    this->last_state_change_ms_ = now;
  }

  if (this->initialized_ && current_raw != this->stable_state_ && (now - this->last_state_change_ms_) >= this->debounce_time_ms_) {
    this->stable_state_ = current_raw;
    this->publish_state(this->stable_state_);
  }
}

void VieweSmartKnobButton::dump_config() {
  LOG_BINARY_SENSOR("", "VieweSmart Knob Button", this);
  LOG_PIN("  Pin: ", this->pin_);
  ESP_LOGCONFIG(TAG, "  Debounce: %u ms", this->debounce_time_ms_);
}

float VieweSmartKnobButton::get_setup_priority() const { return setup_priority::IO; }

}  // namespace viewesmart_knob_button
}  // namespace esphome

