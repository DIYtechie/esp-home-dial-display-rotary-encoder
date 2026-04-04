#include "viewesmart_rotary_encoder.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace viewesmart_rotary_encoder {

static const char *const TAG = "viewesmart_rotary";
static const bool DIAGNOSTIC_DECODER_MODE = false;
static const uint32_t POLL_INTERVAL_MS = 3;
static const uint8_t POLL_DEBOUNCE_TICKS = 2;
static const uint32_t STEP_PUBLISH_INTERVAL_MS = 10;
static const uint32_t FAST_STEP_PUBLISH_INTERVAL_MS = 5;
static const uint32_t DIRECTION_CONFIRMATION_WINDOW_MS = 180;
static const uint32_t DIRECTION_MEMORY_TIMEOUT_MS = 700;
static const uint32_t FASTEST_SPEED_THRESHOLD_MS = 160;
static const uint32_t FAST_SPEED_THRESHOLD_MS = 190;
static const uint32_t MEDIUM_SPEED_THRESHOLD_MS = 220;
static const uint32_t SLOW_MEDIUM_SPEED_THRESHOLD_MS = 300;
static const int32_t FASTEST_VALUE_STEP = 10;
static const int32_t FAST_VALUE_STEP = 7;
static const int32_t MEDIUM_VALUE_STEP = 5;
static const int32_t SLOW_MEDIUM_VALUE_STEP = 3;
static const int32_t SLOW_VALUE_STEP = 1;
static const uint8_t NORMAL_REVERSE_CONFIRMATION_COUNT = 2;
static const uint8_t HIGH_SPEED_REVERSE_CONFIRMATION_COUNT = 3;
static const int32_t NORMAL_REVERSE_CONFIRMATION_MAGNITUDE = 2;
static const int32_t HIGH_SPEED_REVERSE_CONFIRMATION_MAGNITUDE = 4;

enum PollState : uint8_t {
  POLL_STATE_CHECK = 0,
  POLL_STATE_READY = 1,
  POLL_STATE_PHASE_A = 2,
  POLL_STATE_PHASE_B = 3,
};

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
  this->last_poll_ms_ = millis();

  this->pin_a_->setup();
  this->pin_b_->setup();

  if (this->pin_reset_ != nullptr) {
    this->pin_reset_->setup();
  }

  this->encoder_a_level_ = this->pin_a_->digital_read();
  this->encoder_b_level_ = this->pin_b_->digital_read();
  this->poll_state_ = this->encoder_a_level_ == this->encoder_b_level_ ? POLL_STATE_READY : POLL_STATE_CHECK;
  this->pcnt_initialized_ = true;

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
  ESP_LOGCONFIG(TAG, "  Decoder: polled phase state machine");
  ESP_LOGCONFIG(TAG, "  Diagnostic Decoder Mode: %s", YESNO(DIAGNOSTIC_DECODER_MODE));
  ESP_LOGCONFIG(TAG, "  Poll Interval: %" PRIu32 " ms", POLL_INTERVAL_MS);
  ESP_LOGCONFIG(TAG, "  Debounce: %u ticks", POLL_DEBOUNCE_TICKS);
  ESP_LOGCONFIG(TAG, "  Step Publish Interval: %" PRIu32 " ms", STEP_PUBLISH_INTERVAL_MS);
  ESP_LOGCONFIG(TAG, "  Fast Step Publish Interval: %" PRIu32 " ms", FAST_STEP_PUBLISH_INTERVAL_MS);
  ESP_LOGCONFIG(TAG, "  Reverse Direction Confirmation: %" PRIu32 " ms", DIRECTION_CONFIRMATION_WINDOW_MS);
  ESP_LOGCONFIG(TAG, "  Direction Memory Timeout: %" PRIu32 " ms", DIRECTION_MEMORY_TIMEOUT_MS);
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
  const int32_t raw_delta = this->poll_encoder_delta_();
  if (raw_delta != 0) {
    this->raw_count_total_ += raw_delta;
    if (DIAGNOSTIC_DECODER_MODE) {
      ESP_LOGD(TAG, "raw_delta=%" PRId32 " raw_total=%" PRId32 " divider=%d", raw_delta, this->raw_count_total_,
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
    const bool direction_memory_expired =
        this->last_value_change_ms_ == 0 || (now - this->last_value_change_ms_) >= DIRECTION_MEMORY_TIMEOUT_MS;
    const int8_t effective_last_direction = direction_memory_expired ? 0 : this->last_emitted_direction_;
    const bool direction_changed = effective_last_direction != 0 && direction != effective_last_direction;
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
          this->pending_direction_confirmation_magnitude_ < required_confirmation_magnitude) {
        if (DIAGNOSTIC_DECODER_MODE) {
          ESP_LOGD(TAG,
                   "reject reverse pending=%" PRId32 " dir=%" PRId32 " last_dir=%" PRId8
                   " confirm=%u/%u magnitude=%" PRId32 "/%" PRId32 " high_speed=%s dt=%" PRIu32,
                   this->pending_step_delta_, direction, effective_last_direction,
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
      if (time_since_value_change <= FASTEST_SPEED_THRESHOLD_MS) {
        value_step_size = FASTEST_VALUE_STEP;
      } else if (time_since_value_change <= FAST_SPEED_THRESHOLD_MS) {
        value_step_size = FAST_VALUE_STEP;
      } else if (time_since_value_change <= MEDIUM_SPEED_THRESHOLD_MS) {
        value_step_size = MEDIUM_VALUE_STEP;
      } else if (time_since_value_change <= SLOW_MEDIUM_SPEED_THRESHOLD_MS) {
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
  this->debounce_a_count_ = 0;
  this->debounce_b_count_ = 0;
  this->encoder_a_change_ = false;
  this->encoder_b_change_ = false;
  this->encoder_a_level_ = this->pin_a_->digital_read();
  this->encoder_b_level_ = this->pin_b_->digital_read();
  this->poll_state_ = this->encoder_a_level_ == this->encoder_b_level_ ? POLL_STATE_READY : POLL_STATE_CHECK;
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

int32_t VieweSmartRotaryEncoderSensor::poll_encoder_delta_() {
  const uint32_t now = millis();
  if ((now - this->last_poll_ms_) < POLL_INTERVAL_MS) {
    return 0;
  }
  this->last_poll_ms_ = now;

  const bool phase_a = this->pin_a_->digital_read();
  const bool phase_b = this->pin_b_->digital_read();

  if (phase_a != this->encoder_a_level_) {
    this->debounce_a_count_++;
    if (this->debounce_a_count_ >= POLL_DEBOUNCE_TICKS) {
      this->encoder_a_level_ = phase_a;
      this->encoder_a_change_ = true;
      this->debounce_a_count_ = 0;
    }
  } else {
    this->debounce_a_count_ = 0;
  }

  if (phase_b != this->encoder_b_level_) {
    this->debounce_b_count_++;
    if (this->debounce_b_count_ >= POLL_DEBOUNCE_TICKS) {
      this->encoder_b_level_ = phase_b;
      this->encoder_b_change_ = true;
      this->debounce_b_count_ = 0;
    }
  } else {
    this->debounce_b_count_ = 0;
  }

  const int32_t step_delta = this->resolution_divider_();
  switch (this->poll_state_) {
    case POLL_STATE_READY:
      if (this->encoder_a_change_) {
        this->encoder_a_change_ = false;
        this->poll_state_ = POLL_STATE_PHASE_A;
      } else if (this->encoder_b_change_) {
        this->encoder_b_change_ = false;
        this->poll_state_ = POLL_STATE_PHASE_B;
      }
      break;

    case POLL_STATE_PHASE_A:
      if (this->encoder_b_change_) {
        this->encoder_b_change_ = false;
        this->poll_state_ = POLL_STATE_READY;
        return -step_delta;
      }
      if (this->encoder_a_change_) {
        this->encoder_a_change_ = false;
        this->poll_state_ = POLL_STATE_READY;
      }
      break;

    case POLL_STATE_PHASE_B:
      if (this->encoder_a_change_) {
        this->encoder_a_change_ = false;
        this->poll_state_ = POLL_STATE_READY;
        return step_delta;
      }
      if (this->encoder_b_change_) {
        this->encoder_b_change_ = false;
        this->poll_state_ = POLL_STATE_READY;
      }
      break;

    case POLL_STATE_CHECK:
    default:
      if (this->encoder_a_level_ == this->encoder_b_level_) {
        this->poll_state_ = POLL_STATE_READY;
        this->encoder_a_change_ = false;
        this->encoder_b_change_ = false;
      }
      break;
  }

  return 0;
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
