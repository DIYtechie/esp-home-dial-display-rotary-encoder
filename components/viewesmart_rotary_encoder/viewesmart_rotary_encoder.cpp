#include "viewesmart_rotary_encoder.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace viewesmart_rotary_encoder {

static const char *const TAG = "viewesmart_rotary";

#ifdef USE_ESP_IDF
static pcnt_unit_t next_pcnt_unit() {
  static int unit_index = 0;
  const pcnt_unit_t unit = static_cast<pcnt_unit_t>(unit_index);
  unit_index = (unit_index + 1) % SOC_PCNT_UNITS_PER_GROUP;
  return unit;
}

static bool configure_pcnt_channel(pcnt_unit_t unit, pcnt_channel_t channel, int pulse_gpio, int control_gpio,
                                   pcnt_count_mode_t positive_mode, pcnt_count_mode_t negative_mode) {
  pcnt_config_t config{};
  config.pulse_gpio_num = pulse_gpio;
  config.ctrl_gpio_num = control_gpio;
  config.lctrl_mode = PCNT_MODE_REVERSE;
  config.hctrl_mode = PCNT_MODE_KEEP;
  config.pos_mode = positive_mode;
  config.neg_mode = negative_mode;
  config.counter_h_lim = INT16_MAX;
  config.counter_l_lim = INT16_MIN;
  config.unit = unit;
  config.channel = channel;
  return pcnt_unit_config(&config) == ESP_OK;
}
#endif

void VieweSmartRotaryEncoderSensor::setup() {
  int32_t initial_value = 0;
  if (this->restore_mode_ == VIEWESMART_ROTARY_ENCODER_RESTORE_DEFAULT_ZERO) {
    this->rtc_ = this->make_entity_preference<int32_t>();
    this->rtc_.load(&initial_value);
  }

  this->value_ = clamp(initial_value, this->min_value_, this->max_value_);
  this->last_published_ = this->value_;
  this->pin_a_->setup();
  this->pin_b_->setup();
  if (this->pin_reset_ != nullptr) {
    this->pin_reset_->setup();
  }

#ifdef USE_ESP_IDF
  this->pcnt_unit_ = next_pcnt_unit();
  const bool channel_a_ok =
      configure_pcnt_channel(this->pcnt_unit_, PCNT_CHANNEL_0, this->pin_a_->get_pin(), this->pin_b_->get_pin(),
                             PCNT_COUNT_INC, PCNT_COUNT_DEC);
  const bool channel_b_ok =
      configure_pcnt_channel(this->pcnt_unit_, PCNT_CHANNEL_1, this->pin_b_->get_pin(), this->pin_a_->get_pin(),
                             PCNT_COUNT_DEC, PCNT_COUNT_INC);
  if (!channel_a_ok || !channel_b_ok) {
    ESP_LOGE(TAG, "Failed to configure PCNT quadrature channels");
    this->mark_failed();
    return;
  }

  // The maximum legacy PCNT filter rejects sub-millisecond electrical spikes
  // without imposing a software debounce delay on valid quadrature edges.
  pcnt_set_filter_value(this->pcnt_unit_, 1023);
  pcnt_filter_enable(this->pcnt_unit_);
  pcnt_counter_pause(this->pcnt_unit_);
  pcnt_counter_clear(this->pcnt_unit_);
  pcnt_counter_resume(this->pcnt_unit_);
  this->pcnt_initialized_ = true;
#else
  ESP_LOGE(TAG, "The PCNT rotary decoder requires ESP-IDF");
  this->mark_failed();
  return;
#endif

  if (this->publish_initial_value_) {
    this->publish_value_(true);
    this->publish_initial_value_ = false;
  }
}

void VieweSmartRotaryEncoderSensor::dump_config() {
  LOG_SENSOR("", "VieweSmart Rotary Encoder", this);
  LOG_PIN("  Pin A: ", this->pin_a_);
  LOG_PIN("  Pin B: ", this->pin_b_);
  LOG_PIN("  Reset Pin: ", this->pin_reset_);
  ESP_LOGCONFIG(TAG, "  Decoder: ESP32 PCNT quadrature");
  ESP_LOGCONFIG(TAG, "  Output: every logical step, no decoder acceleration");
  ESP_LOGCONFIG(TAG, "  PCNT Glitch Filter: 1023 APB cycles");
  ESP_LOGCONFIG(TAG, "  Min Value: %" PRId32, this->min_value_);
  ESP_LOGCONFIG(TAG, "  Max Value: %" PRId32, this->max_value_);

  switch (this->resolution_) {
    case VIEWESMART_ROTARY_ENCODER_1_PULSE_PER_CYCLE:
      ESP_LOGCONFIG(TAG, "  Resolution: 1 pulse per cycle");
      break;
    case VIEWESMART_ROTARY_ENCODER_2_PULSES_PER_CYCLE:
      ESP_LOGCONFIG(TAG, "  Resolution: 2 pulses per cycle");
      break;
    case VIEWESMART_ROTARY_ENCODER_4_PULSES_PER_CYCLE:
      ESP_LOGCONFIG(TAG, "  Resolution: 4 pulses per cycle");
      break;
  }
}

void VieweSmartRotaryEncoderSensor::loop() {
  if (this->pin_reset_ != nullptr && this->pin_reset_->digital_read()) {
    this->set_value(0);
  }

#ifdef USE_ESP_IDF
  if (!this->pcnt_initialized_) {
    return;
  }

  int16_t hardware_delta = 0;
  if (pcnt_get_counter_value(this->pcnt_unit_, &hardware_delta) != ESP_OK) {
    ESP_LOGW(TAG, "Failed to read PCNT counter");
    return;
  }
  if (hardware_delta != 0) {
    pcnt_counter_clear(this->pcnt_unit_);
    this->raw_count_total_ += hardware_delta;
  }
#endif

  const int32_t current_step_count = this->raw_count_total_ / this->resolution_divider_();
  int32_t delta_steps = current_step_count - this->last_reported_step_count_;
  this->last_reported_step_count_ = current_step_count;

  // Deliver each hardware-counted logical step separately. This gives local
  // UI code a lossless event stream even when several steps arrive in one loop.
  while (delta_steps != 0) {
    const int32_t direction = delta_steps > 0 ? 1 : -1;
    const int32_t previous_value = this->value_;
    this->value_ = clamp(this->value_ + direction, this->min_value_, this->max_value_);
    if (this->value_ != previous_value) {
      if (direction > 0) {
        this->on_clockwise_callback_.call();
      } else {
        this->on_anticlockwise_callback_.call();
      }
      ESP_LOGD(TAG, "step dir=%" PRId32 " value=%" PRId32 "->%" PRId32, direction, previous_value, this->value_);
      this->publish_value_();
    }
    delta_steps -= direction;
  }

  if (this->pending_publish_ || this->publish_initial_value_) {
    this->publish_value_(true);
    this->pending_publish_ = false;
    this->publish_initial_value_ = false;
  }
}

float VieweSmartRotaryEncoderSensor::get_setup_priority() const { return setup_priority::IO; }

void VieweSmartRotaryEncoderSensor::set_value(int value) {
  // A page change changes only the logical value. The hardware counter and any
  // partial quadrature cycle continue uninterrupted.
  this->value_ = clamp<int32_t>(value, this->min_value_, this->max_value_);
  this->pending_publish_ = true;
}

int VieweSmartRotaryEncoderSensor::resolution_divider_() const {
  switch (this->resolution_) {
    case VIEWESMART_ROTARY_ENCODER_1_PULSE_PER_CYCLE:
      return 4;
    case VIEWESMART_ROTARY_ENCODER_2_PULSES_PER_CYCLE:
      return 2;
    case VIEWESMART_ROTARY_ENCODER_4_PULSES_PER_CYCLE:
      return 1;
  }
  return 4;
}

int VieweSmartRotaryEncoderSensor::logical_steps_per_cycle_() const { return 4 / this->resolution_divider_(); }

int VieweSmartRotaryEncoderSensor::step_size_from_phase_deltas_(uint32_t entry_delta_ms,
                                                                uint32_t exit_delta_ms) const {
  return 1;
}

int32_t VieweSmartRotaryEncoderSensor::poll_encoder_delta_() { return 0; }

void VieweSmartRotaryEncoderSensor::publish_value_(bool force) {
  if (!force && this->value_ == this->last_published_) {
    return;
  }
  if (this->restore_mode_ == VIEWESMART_ROTARY_ENCODER_RESTORE_DEFAULT_ZERO) {
    this->rtc_.save(&this->value_);
  }
  this->last_published_ = this->value_;
  this->publish_state(this->value_);
  this->listeners_.call(this->value_);
}

}  // namespace viewesmart_rotary_encoder
}  // namespace esphome
