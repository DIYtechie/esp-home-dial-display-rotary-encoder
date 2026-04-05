#pragma once

#include <cinttypes>
#include <cstdint>

#ifdef USE_ESP_IDF
#include <driver/pcnt.h>
#endif

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace viewesmart_rotary_encoder {

enum VieweSmartRotaryEncoderRestoreMode {
  VIEWESMART_ROTARY_ENCODER_RESTORE_DEFAULT_ZERO,
  VIEWESMART_ROTARY_ENCODER_ALWAYS_ZERO,
};

enum VieweSmartRotaryEncoderResolution {
  VIEWESMART_ROTARY_ENCODER_1_PULSE_PER_CYCLE = 0x4400,
  VIEWESMART_ROTARY_ENCODER_2_PULSES_PER_CYCLE = 0x2200,
  VIEWESMART_ROTARY_ENCODER_4_PULSES_PER_CYCLE = 0x1100,
};

class VieweSmartRotaryEncoderSensor : public sensor::Sensor, public Component {
 public:
  void set_pin_a(InternalGPIOPin *pin_a) { this->pin_a_ = pin_a; }
  void set_pin_b(InternalGPIOPin *pin_b) { this->pin_b_ = pin_b; }
  void set_reset_pin(InternalGPIOPin *pin_reset) { this->pin_reset_ = pin_reset; }
  void set_restore_mode(VieweSmartRotaryEncoderRestoreMode restore_mode) { this->restore_mode_ = restore_mode; }
  void set_resolution(VieweSmartRotaryEncoderResolution resolution) { this->resolution_ = resolution; }
  void set_min_value(int32_t min_value) { this->min_value_ = min_value; }
  void set_max_value(int32_t max_value) { this->max_value_ = max_value; }
  void set_publish_initial_value(bool publish_initial_value) { this->publish_initial_value_ = publish_initial_value; }
  void set_value(int value);

  void setup() override;
  void dump_config() override;
  void loop() override;
  float get_setup_priority() const override;

  template<typename F> void add_on_clockwise_callback(F &&callback) {
    this->on_clockwise_callback_.add(std::forward<F>(callback));
  }

  template<typename F> void add_on_anticlockwise_callback(F &&callback) {
    this->on_anticlockwise_callback_.add(std::forward<F>(callback));
  }

  template<typename F> void register_listener(F &&listener) { this->listeners_.add(std::forward<F>(listener)); }

 protected:
  int resolution_divider_() const;
  void publish_value_(bool force = false);
  int32_t poll_encoder_delta_();

  InternalGPIOPin *pin_a_{nullptr};
  InternalGPIOPin *pin_b_{nullptr};
  InternalGPIOPin *pin_reset_{nullptr};

  bool publish_initial_value_{false};
  bool pending_publish_{false};
  ESPPreferenceObject rtc_;
  VieweSmartRotaryEncoderRestoreMode restore_mode_{VIEWESMART_ROTARY_ENCODER_RESTORE_DEFAULT_ZERO};

  int32_t value_{0};
  int32_t last_published_{0};
  int32_t min_value_{INT32_MIN};
  int32_t max_value_{INT32_MAX};
  int32_t raw_count_total_{0};
  int32_t last_reported_step_count_{0};
  int32_t pending_step_delta_{0};
  uint32_t last_step_publish_ms_{0};
  uint32_t last_value_change_ms_{0};
  int8_t last_emitted_direction_{0};
  int8_t pending_direction_confirmation_{0};
  uint8_t pending_direction_confirmation_count_{0};
  int32_t pending_direction_confirmation_magnitude_{0};
  uint32_t pending_direction_confirmation_ms_{0};
  uint32_t last_poll_ms_{0};
  uint32_t last_raw_transition_ms_{0};
  uint8_t debounce_a_count_{0};
  uint8_t debounce_b_count_{0};
  bool encoder_a_change_{false};
  bool encoder_b_change_{false};
  bool encoder_a_level_{false};
  bool encoder_b_level_{false};
  uint8_t poll_state_{0};

  VieweSmartRotaryEncoderResolution resolution_{VIEWESMART_ROTARY_ENCODER_1_PULSE_PER_CYCLE};

#ifdef USE_ESP_IDF
  pcnt_unit_t pcnt_unit_{PCNT_UNIT_0};
  bool pcnt_initialized_{false};
#endif

  CallbackManager<void()> on_clockwise_callback_{};
  CallbackManager<void()> on_anticlockwise_callback_{};
  CallbackManager<void(int32_t)> listeners_{};
};

template<typename... Ts> class VieweSmartRotaryEncoderSetValueAction : public Action<Ts...> {
 public:
  explicit VieweSmartRotaryEncoderSetValueAction(VieweSmartRotaryEncoderSensor *encoder) : encoder_(encoder) {}
  TEMPLATABLE_VALUE(int, value)

  void play(const Ts &...x) override { this->encoder_->set_value(this->value_.value(x...)); }

 protected:
  VieweSmartRotaryEncoderSensor *encoder_;
};

}  // namespace viewesmart_rotary_encoder
}  // namespace esphome
