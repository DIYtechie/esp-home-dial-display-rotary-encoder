#include "viewesmart_rotary_encoder.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace viewesmart_rotary_encoder {

static const char *const TAG = "viewesmart_rotary";

static const uint8_t STATE_LUT_MASK = 0x1C;
static const uint16_t STATE_PIN_A_HIGH = 0x01;
static const uint16_t STATE_PIN_B_HIGH = 0x02;
static const uint16_t STATE_S0 = 0x00;
static const uint16_t STATE_S1 = 0x04;
static const uint16_t STATE_S2 = 0x08;
static const uint16_t STATE_S3 = 0x0C;
static const uint16_t STATE_CCW = 0x00;
static const uint16_t STATE_CW = 0x10;
static const uint16_t STATE_HAS_INCREMENTED = 0x0700;
static const uint16_t STATE_INCREMENT_COUNTER_4 = 0x0700;
static const uint16_t STATE_INCREMENT_COUNTER_2 = 0x0300;
static const uint16_t STATE_INCREMENT_COUNTER_1 = 0x0100;
static const uint16_t STATE_HAS_DECREMENTED = 0x7000;
static const uint16_t STATE_DECREMENT_COUNTER_4 = 0x7000;
static const uint16_t STATE_DECREMENT_COUNTER_2 = 0x3000;
static const uint16_t STATE_DECREMENT_COUNTER_1 = 0x1000;

#ifndef DRAM_ATTR
#define DRAM_ATTR
#endif

static const uint16_t DRAM_ATTR STATE_LOOKUP_TABLE[32] = {
    STATE_CCW | STATE_S0, STATE_CW | STATE_S1 | STATE_INCREMENT_COUNTER_1, STATE_CCW | STATE_S0,
    STATE_CCW | STATE_S3 | STATE_DECREMENT_COUNTER_4, STATE_CCW | STATE_S1, STATE_CCW | STATE_S1,
    STATE_CCW | STATE_S0 | STATE_DECREMENT_COUNTER_1, STATE_CW | STATE_S2 | STATE_INCREMENT_COUNTER_4,
    STATE_CCW | STATE_S1 | STATE_DECREMENT_COUNTER_2, STATE_CCW | STATE_S2,
    STATE_CW | STATE_S3 | STATE_INCREMENT_COUNTER_1, STATE_CCW | STATE_S2,
    STATE_CW | STATE_S0 | STATE_INCREMENT_COUNTER_2, STATE_CCW | STATE_S2 | STATE_DECREMENT_COUNTER_1,
    STATE_CCW | STATE_S3, STATE_CCW | STATE_S3, STATE_CW | STATE_S0,
    STATE_CW | STATE_S1 | STATE_INCREMENT_COUNTER_1, STATE_CW | STATE_S0,
    STATE_CCW | STATE_S3 | STATE_DECREMENT_COUNTER_4, STATE_CW | STATE_S1, STATE_CW | STATE_S1,
    STATE_CCW | STATE_S0 | STATE_DECREMENT_COUNTER_1, STATE_CW | STATE_S2 | STATE_INCREMENT_COUNTER_4,
    STATE_CCW | STATE_S1 | STATE_DECREMENT_COUNTER_2, STATE_CW | STATE_S2,
    STATE_CW | STATE_S3 | STATE_INCREMENT_COUNTER_1, STATE_CW | STATE_S2,
    STATE_CW | STATE_S0 | STATE_INCREMENT_COUNTER_2, STATE_CCW | STATE_S2 | STATE_DECREMENT_COUNTER_1,
    STATE_CW | STATE_S3, STATE_CW | STATE_S3,
};

void IRAM_ATTR HOT VieweSmartRotaryEncoderStore::gpio_intr(VieweSmartRotaryEncoderStore *arg) {
  uint8_t input_state = arg->state & STATE_LUT_MASK;
  if (arg->pin_a.digital_read()) {
    input_state |= STATE_PIN_A_HIGH;
  }
  if (arg->pin_b.digital_read()) {
    input_state |= STATE_PIN_B_HIGH;
  }

  const uint8_t previous_pins = arg->state & (STATE_PIN_A_HIGH | STATE_PIN_B_HIGH);
  const uint8_t changed_pins = previous_pins ^ (input_state & (STATE_PIN_A_HIGH | STATE_PIN_B_HIGH));
  if (changed_pins == (STATE_PIN_A_HIGH | STATE_PIN_B_HIGH)) {
    arg->invalid_transition_count++;
  }

  const uint16_t new_state = STATE_LOOKUP_TABLE[input_state];
  int8_t rotation_dir = 0;
  if ((new_state & arg->resolution & STATE_HAS_INCREMENTED) != 0) {
    rotation_dir = 1;
  }
  if ((new_state & arg->resolution & STATE_HAS_DECREMENTED) != 0) {
    rotation_dir = -1;
  }

  if (rotation_dir != 0 && !arg->first_read) {
    const uint16_t next_head = (arg->head + 1U) % VieweSmartRotaryEncoderStore::EVENT_QUEUE_SIZE;
    if (next_head == arg->tail) {
      arg->overflow_count++;
    } else {
      arg->events[arg->head] = rotation_dir;
      arg->head = next_head;
    }
  }

  arg->first_read = false;
  arg->state = new_state;
}

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

  this->pin_a_->setup();
  this->pin_b_->setup();
  this->store_.pin_a = this->pin_a_->to_isr();
  this->store_.pin_b = this->pin_b_->to_isr();

  if (this->pin_reset_ != nullptr) {
    this->pin_reset_->setup();
  }

  this->pin_a_->attach_interrupt(VieweSmartRotaryEncoderStore::gpio_intr, &this->store_, gpio::INTERRUPT_ANY_EDGE);
  this->pin_b_->attach_interrupt(VieweSmartRotaryEncoderStore::gpio_intr, &this->store_, gpio::INTERRUPT_ANY_EDGE);

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
  ESP_LOGCONFIG(TAG, "  Event Queue Size: %u", VieweSmartRotaryEncoderStore::EVENT_QUEUE_SIZE);
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

  switch (this->store_.resolution) {
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

  int8_t event = 0;
  bool changed = false;
  while (this->pop_event_(&event)) {
    changed = true;
    if (event > 0) {
      this->value_ = clamp(this->value_ + 1, this->min_value_, this->max_value_);
      this->on_clockwise_callback_.call();
    } else {
      this->value_ = clamp(this->value_ - 1, this->min_value_, this->max_value_);
      this->on_anticlockwise_callback_.call();
    }
  }

  if (changed || this->pending_publish_ || this->publish_initial_value_) {
    this->publish_value_(this->pending_publish_ || this->publish_initial_value_);
    this->pending_publish_ = false;
    this->publish_initial_value_ = false;
  }

  if (this->last_reported_overflow_count_ != this->store_.overflow_count) {
    const uint32_t delta = this->store_.overflow_count - this->last_reported_overflow_count_;
    this->last_reported_overflow_count_ = this->store_.overflow_count;
    ESP_LOGW(TAG, "Dropped %u queued rotary events because the ISR queue filled up", delta);
  }
  if (this->last_reported_invalid_transition_count_ != this->store_.invalid_transition_count) {
    const uint32_t delta = this->store_.invalid_transition_count - this->last_reported_invalid_transition_count_;
    this->last_reported_invalid_transition_count_ = this->store_.invalid_transition_count;
    ESP_LOGV(TAG, "Observed %u invalid quadrature transitions", delta);
  }
}

float VieweSmartRotaryEncoderSensor::get_setup_priority() const { return setup_priority::IO; }

void VieweSmartRotaryEncoderSensor::set_value(int value) {
  this->clear_pending_events_();
  this->value_ = clamp<int32_t>(value, this->min_value_, this->max_value_);
  this->pending_publish_ = true;
}

bool VieweSmartRotaryEncoderSensor::pop_event_(int8_t *event) {
  InterruptLock lock;
  if (this->store_.tail == this->store_.head) {
    return false;
  }
  *event = this->store_.events[this->store_.tail];
  this->store_.tail = (this->store_.tail + 1U) % VieweSmartRotaryEncoderStore::EVENT_QUEUE_SIZE;
  return true;
}

void VieweSmartRotaryEncoderSensor::clear_pending_events_() {
  InterruptLock lock;
  this->store_.head = 0;
  this->store_.tail = 0;
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

