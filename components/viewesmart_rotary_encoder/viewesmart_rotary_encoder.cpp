#include "viewesmart_rotary_encoder.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace viewesmart_rotary_encoder {

static const char *const TAG = "viewesmart_rotary";
static const bool DIAGNOSTIC_DECODER_MODE = true;
static const uint32_t STEP_PUBLISH_INTERVAL_MS = 10;
static const uint32_t FAST_STEP_PUBLISH_INTERVAL_MS = 5;
static const uint32_t DIRECTION_CONFIRMATION_WINDOW_MS = 180;
static const uint32_t FASTEST_SPEED_THRESHOLD_MS = 25;
static const uint32_t FAST_SPEED_THRESHOLD_MS = 45;
static const uint32_t MEDIUM_SPEED_THRESHOLD_MS = 70;
static const uint32_t SLOW_MEDIUM_SPEED_THRESHOLD_MS = 120;
static const int32_t FASTEST_VALUE_STEP = 1;
static const int32_t FAST_VALUE_STEP = 1;
static const int32_t MEDIUM_VALUE_STEP = 1;
static const int32_t SLOW_MEDIUM_VALUE_STEP = 1;
static const int32_t SLOW_VALUE_STEP = 1;
static const uint8_t NORMAL_REVERSE_CONFIRMATION_COUNT = 2;
static const uint8_t HIGH_SPEED_REVERSE_CONFIRMATION_COUNT = 3;
static const int32_t NORMAL_REVERSE_CONFIRMATION_MAGNITUDE = 3;
static const int32_t HIGH_SPEED_REVERSE_CONFIRMATION_MAGNITUDE = 4;

#ifdef USE_ESP_IDF
static pcnt_unit_t next_pcnt_unit() {
  static int unit_index = 0;
  const pcnt_unit_t unit = static_cast<pcnt_unit_t>(unit_index);
  unit_index = (unit_index + 1) % SOC_PCNT_UNITS_PER_GROUP;
  return unit;
}

static bool configure_pcnt_channel(pcnt_unit_t unit, pcnt_channel_t channel, int pulse_gpio, int ctrl_gpio,
                                   pcnt_count_mode_t pos_mode, pcnt_count_mode_t neg_mode) {
  pcnt_config_t config{};
  config.pulse_gpio_num = pulse_gpio;
  config.ctrl_gpio_num = ctrl_gpio;
  config.lctrl_mode = PCNT_MODE_REVERSE;
  config.hctrl_mode = PCNT_MODE_KEEP;
  config.pos_mode = pos_mode;
  config.neg_mode = neg_mode;
  config.counter_h_lim = INT16_MAX;
  config.counter_l_lim = INT16_MIN;
  config.unit = unit;
  config.channel = channel;
  return pcnt_unit_config(&config) == ESP_OK;
}
#endif

void VieweSmartRotaryEncoderSensor::setup() {
  int32_t initial_value = 0;
  switch (this->restore_mode_) {
    case VIEWESMART_ROTARY_ENCODER_RESTORE_DEFAULT_ZERO:
      this->rtc_ = this->make_entity_preference<int32_t>();
      if (!this->rtc_.load(&initial_value)) {
        initial_value = 0;
      }
      break;
    case VIEWESMART_ROTARY_ENCODER_ALWAYS_ZERO:
      initial_value = 0;
      break;
  }

  this->value_ = clamp(initial_value, this->min_value_, this->max_value_);
  this->last_published_ = this->value_;
  this->last_step_publish_ms_ = millis();
  this->last_value_change_ms_ = 0;

  this->pin_a_->setup();
  this->pin_b_->setup();

  if (this->pin_reset_ != nullptr) {
    this->pin_reset_->setup();
  }

#ifdef USE_ESP_IDF
  this->pcnt_unit_ = next_pcnt_unit();

  const bool first_channel_ok =
      configure_pcnt_channel(this->pcnt_unit_, PCNT_CHANNEL_0, this->pin_a_->get_pin(), this->pin_b_->get_pin(),
                             PCNT_COUNT_INC, PCNT_COUNT_DEC);
  const bool second_channel_ok =
      configure_pcnt_channel(this->pcnt_unit_, PCNT_CHANNEL_1, this->pin_b_->get_pin(), this->pin_a_->get_pin(),
                             PCNT_COUNT_DEC, PCNT_COUNT_INC);

  if (!first_channel_ok || !second_channel_ok) {
    ESP_LOGE(TAG, "Failed to configure PCNT unit for the rotary encoder");
    this->mark_failed();
    return;
  }

  pcnt_set_filter_value(this->pcnt_unit_, 1023);
  pcnt_filter_enable(this->pcnt_unit_);
  pcnt_counter_pause(this->pcnt_unit_);
  pcnt_counter_clear(this->pcnt_unit_);
  pcnt_counter_resume(this->pcnt_unit_);
  this->pcnt_initialized_ = true;
#else
  ESP_LOGE(TAG, "This rotary encoder component requires ESP-IDF");
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
  ESP_LOGCONFIG(TAG, "  Decoder: PCNT quadrature");
  ESP_LOGCONFIG(TAG, "  Diagnostic Decoder Mode: %s", YESNO(DIAGNOSTIC_DECODER_MODE));
  ESP_LOGCONFIG(TAG, "  Glitch Filter: 1023 APB cycles");
  ESP_LOGCONFIG(TAG, "  Step Publish Interval: %" PRIu32 " ms", STEP_PUBLISH_INTERVAL_MS);
  ESP_LOGCONFIG(TAG, "  Fast Step Publish Interval: %" PRIu32 " ms", FAST_STEP_PUBLISH_INTERVAL_MS);
  ESP_LOGCONFIG(TAG, "  Reverse Direction Confirmation: %" PRIu32 " ms", DIRECTION_CONFIRMATION_WINDOW_MS);
  ESP_LOGCONFIG(TAG, "  Slow Reverse Accept: disabled");
  ESP_LOGCONFIG(TAG, "  Speed Thresholds: slow_medium=%" PRIu32 ", medium=%" PRIu32 ", fast=%" PRIu32
                     ", fastest=%" PRIu32 " ms",
                SLOW_MEDIUM_SPEED_THRESHOLD_MS, MEDIUM_SPEED_THRESHOLD_MS, FAST_SPEED_THRESHOLD_MS,
                FASTEST_SPEED_THRESHOLD_MS);
  ESP_LOGCONFIG(TAG, "  Step Sizes: slow=%" PRId32 ", slow_medium=%" PRId32 ", medium=%" PRId32 ", fast=%" PRId32
                     ", fastest=%" PRId32,
                SLOW_VALUE_STEP, SLOW_MEDIUM_VALUE_STEP, MEDIUM_VALUE_STEP, FAST_VALUE_STEP, FASTEST_VALUE_STEP);
  ESP_LOGCONFIG(TAG, "  Reverse Confirmation Count: normal=%u, high_speed=%u",
                NORMAL_REVERSE_CONFIRMATION_COUNT, HIGH_SPEED_REVERSE_CONFIRMATION_COUNT);
  ESP_LOGCONFIG(TAG, "  Reverse Confirmation Magnitude: normal=%" PRId32 ", high_speed=%" PRId32,
                NORMAL_REVERSE_CONFIRMATION_MAGNITUDE, HIGH_SPEED_REVERSE_CONFIRMATION_MAGNITUDE);
  ESP_LOGCONFIG(TAG, "  Min Value: %" PRId32, this->min_value_);
  ESP_LOGCONFIG(TAG, "  Max Value: %" PRId32, this->max_value_);

  switch (this->restore_mode_) {
    case VIEWESMART_ROTARY_ENCODER_RESTORE_DEFAULT_ZERO:
      ESP_LOGCONFIG(TAG, "  Restore Mode: Restore (defaults to zero)");
      break;
    case VIEWESMART_ROTARY_ENCODER_ALWAYS_ZERO:
      ESP_LOGCONFIG(TAG, "  Restore Mode: Always zero");
      break;
  }

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

  int16_t raw_delta = 0;
  if (pcnt_get_counter_value(this->pcnt_unit_, &raw_delta) != ESP_OK) {
    ESP_LOGW(TAG, "Failed to read PCNT counter");
    return;
  }
  pcnt_counter_clear(this->pcnt_unit_);

  if (raw_delta != 0) {
    this->raw_count_total_ += raw_delta;
    if (DIAGNOSTIC_DECODER_MODE) {
      ESP_LOGD(TAG, "raw_delta=%d raw_total=%" PRId32 " divider=%d", raw_delta, this->raw_count_total_,
               this->resolution_divider_());
    }
  }
#endif

  const int32_t current_step_count = this->raw_count_total_ / this->resolution_divider_();
  const int32_t delta_steps = current_step_count - this->last_reported_step_count_;
  bool changed = false;

  if (delta_steps != 0) {
    this->pending_step_delta_ = delta_steps;
    this->last_reported_step_count_ = current_step_count;
    if (DIAGNOSTIC_DECODER_MODE) {
      ESP_LOGD(TAG, "step_update current=%" PRId32 " delta=%" PRId32 " pending=%" PRId32, current_step_count,
               delta_steps, this->pending_step_delta_);
    }
  }

  const uint32_t now = millis();
  const int32_t pending_step_magnitude = abs(this->pending_step_delta_);
  const uint32_t effective_step_interval =
      pending_step_magnitude >= 2 ? FAST_STEP_PUBLISH_INTERVAL_MS : STEP_PUBLISH_INTERVAL_MS;

  if (this->pending_step_delta_ != 0 && (now - this->last_step_publish_ms_) >= effective_step_interval) {
    const int32_t direction = this->pending_step_delta_ > 0 ? 1 : -1;
    const bool direction_changed = this->last_emitted_direction_ != 0 && direction != this->last_emitted_direction_;
    const bool leaving_lower_bound = this->value_ == this->min_value_ && direction > 0;
    const bool leaving_upper_bound = this->value_ == this->max_value_ && direction < 0;
    const bool leaving_bound = leaving_lower_bound || leaving_upper_bound;
    uint32_t time_since_value_change = UINT32_MAX;
    if (this->last_value_change_ms_ != 0) {
      time_since_value_change = now - this->last_value_change_ms_;
    }

    const bool high_speed_context =
        time_since_value_change <= FAST_SPEED_THRESHOLD_MS || pending_step_magnitude >= 4;
    const bool bypass_direction_confirmation = leaving_bound;

    if (direction_changed && !bypass_direction_confirmation) {
      const uint8_t required_confirmation_count =
          high_speed_context ? HIGH_SPEED_REVERSE_CONFIRMATION_COUNT : NORMAL_REVERSE_CONFIRMATION_COUNT;
      const int32_t required_confirmation_magnitude =
          high_speed_context ? HIGH_SPEED_REVERSE_CONFIRMATION_MAGNITUDE : NORMAL_REVERSE_CONFIRMATION_MAGNITUDE;
      const bool confirmation_window_open =
          this->pending_direction_confirmation_ == direction &&
          (now - this->pending_direction_confirmation_ms_) <= DIRECTION_CONFIRMATION_WINDOW_MS;
      if (confirmation_window_open) {
        this->pending_direction_confirmation_count_++;
        this->pending_direction_confirmation_magnitude_ += pending_step_magnitude;
      } else {
        this->pending_direction_confirmation_ = direction;
        this->pending_direction_confirmation_ms_ = now;
        this->pending_direction_confirmation_count_ = 1;
        this->pending_direction_confirmation_magnitude_ = pending_step_magnitude;
      }

      if (this->pending_direction_confirmation_count_ < required_confirmation_count ||
          this->pending_direction_confirmation_magnitude_ < required_confirmation_magnitude ||
          pending_step_magnitude < 2) {
        if (DIAGNOSTIC_DECODER_MODE) {
          ESP_LOGD(TAG,
                   "reject reverse pending=%" PRId32 " dir=%" PRId32 " last_dir=%" PRId8
                   " confirm=%u/%u magnitude=%" PRId32 "/%" PRId32 " high_speed=%s dt=%" PRIu32,
                   this->pending_step_delta_, direction, this->last_emitted_direction_,
                   this->pending_direction_confirmation_count_, required_confirmation_count,
                   this->pending_direction_confirmation_magnitude_, required_confirmation_magnitude,
                   YESNO(high_speed_context), time_since_value_change);
        }
        this->pending_step_delta_ = 0;
        this->last_step_publish_ms_ = now;
        return;
      }
    }

    int32_t value_step_size = SLOW_VALUE_STEP;
    if (!DIAGNOSTIC_DECODER_MODE && !direction_changed) {
      if (pending_step_magnitude >= 5 || time_since_value_change <= FASTEST_SPEED_THRESHOLD_MS) {
        value_step_size = FASTEST_VALUE_STEP;
      } else if (pending_step_magnitude >= 4 || time_since_value_change <= FAST_SPEED_THRESHOLD_MS) {
        value_step_size = FAST_VALUE_STEP;
      } else if (pending_step_magnitude >= 3 || time_since_value_change <= MEDIUM_SPEED_THRESHOLD_MS) {
        value_step_size = MEDIUM_VALUE_STEP;
      } else if (pending_step_magnitude >= 2 || time_since_value_change <= SLOW_MEDIUM_SPEED_THRESHOLD_MS) {
        value_step_size = SLOW_MEDIUM_VALUE_STEP;
      }
    }

    const int32_t previous_value = this->value_;
    this->value_ = clamp(this->value_ + direction * value_step_size, this->min_value_, this->max_value_);

    if (this->value_ != previous_value) {
      if (DIAGNOSTIC_DECODER_MODE) {
        ESP_LOGD(TAG,
                 "accept pending=%" PRId32 " dir=%" PRId32 " step=%" PRId32 " value=%" PRId32 "->%" PRId32
                 " changed_dir=%s leaving_bound=%s dt=%" PRIu32,
                 this->pending_step_delta_, direction, value_step_size, previous_value, this->value_,
                 YESNO(direction_changed), YESNO(leaving_bound), time_since_value_change);
      }
      if (direction > 0) {
        this->on_clockwise_callback_.call();
      } else {
        this->on_anticlockwise_callback_.call();
      }
      changed = true;
      this->last_emitted_direction_ = direction;
      this->last_value_change_ms_ = now;
    } else if (DIAGNOSTIC_DECODER_MODE) {
      ESP_LOGD(TAG, "clamped pending=%" PRId32 " dir=%" PRId32 " step=%" PRId32 " value=%" PRId32,
               this->pending_step_delta_, direction, value_step_size, previous_value);
    }

    this->pending_direction_confirmation_ = 0;
    this->pending_direction_confirmation_count_ = 0;
    this->pending_direction_confirmation_magnitude_ = 0;
    this->pending_direction_confirmation_ms_ = 0;
    this->pending_step_delta_ = 0;
    this->last_step_publish_ms_ = now;
  }

  if (changed || this->pending_publish_ || this->publish_initial_value_) {
    this->publish_value_(this->pending_publish_ || this->publish_initial_value_);
    this->pending_publish_ = false;
    this->publish_initial_value_ = false;
  }
}

float VieweSmartRotaryEncoderSensor::get_setup_priority() const { return setup_priority::IO; }

void VieweSmartRotaryEncoderSensor::set_value(int value) {
#ifdef USE_ESP_IDF
  if (this->pcnt_initialized_) {
    pcnt_counter_pause(this->pcnt_unit_);
    pcnt_counter_clear(this->pcnt_unit_);
    pcnt_counter_resume(this->pcnt_unit_);
  }
#endif
  this->raw_count_total_ = 0;
  this->last_reported_step_count_ = 0;
  this->pending_step_delta_ = 0;
  this->last_step_publish_ms_ = millis();
  this->last_value_change_ms_ = 0;
  this->last_emitted_direction_ = 0;
  this->pending_direction_confirmation_ = 0;
  this->pending_direction_confirmation_count_ = 0;
  this->pending_direction_confirmation_magnitude_ = 0;
  this->pending_direction_confirmation_ms_ = 0;
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
