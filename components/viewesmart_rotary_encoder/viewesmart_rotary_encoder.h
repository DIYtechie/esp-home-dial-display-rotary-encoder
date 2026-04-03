#pragma once

#include <array>
#include <cinttypes>
#include <cstdint>

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

struct VieweSmartRotaryEncoderStore {
  static constexpr size_t EVENT_QUEUE_SIZE = 128;

  ISRInternalGPIOPin pin_a;
  ISRInternalGPIOPin pin_b;

  VieweSmartRotaryEncoderResolution resolution{VIEWESMART_ROTARY_ENCODER_1_PULSE_PER_CYCLE};
  uint8_t state{0};
  bool first_read{true};

  std::array<int8_t, EVENT_QUEUE_SIZE> events{};
  volatile uint16_t head{0};
  volatile uint16_t tail{0};
  volatile uint32_t overflow_count{0};
  volatile uint32_t invalid_transition_count{0};

  static void gpio_intr(VieweSmartRotaryEncoderStore *arg);
};

class VieweSmartRotaryEncoderSensor : public sensor::Sensor, public Component {
 public:
  void set_pin_a(InternalGPIOPin *pin_a) { this->pin_a_ = pin_a; }
  void set_pin_b(InternalGPIOPin *pin_b) { this->pin_b_ = pin_b; }
  void set_reset_pin(InternalGPIOPin *pin_reset) { this->pin_reset_ = pin_reset; }
  void set_restore_mode(VieweSmartRotaryEncoderRestoreMode restore_mode) { this->restore_mode_ = restore_mode; }
  void set_resolution(VieweSmartRotaryEncoderResolution resolution) { this->store_.resolution = resolution; }
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
  bool pop_event_(int8_t *event);
  void clear_pending_events_();
  void publish_value_(bool force = false);

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

  uint32_t last_reported_overflow_count_{0};
  uint32_t last_reported_invalid_transition_count_{0};

  VieweSmartRotaryEncoderStore store_{};

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
