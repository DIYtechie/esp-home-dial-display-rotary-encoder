#include "viewesmart_touchscreen.h"

#include <cmath>

#include "esphome/core/helpers.h"

namespace esphome {
namespace viewesmart_touchscreen {

void VieweSmartTouchscreen::start_swipe_(const touchscreen::TouchPoint &tp) {
  this->swipe_.active = true;
  this->swipe_.start_x = tp.x;
  this->swipe_.start_y = tp.y;
  this->swipe_.stable_x = tp.x;
  this->swipe_.stable_y = tp.y;
  this->swipe_.peak_x = tp.x;
  this->swipe_.peak_y = tp.y;
}

void VieweSmartTouchscreen::update_swipe_(const touchscreen::TouchPoint &tp) {
  if (!this->swipe_.active) {
    this->start_swipe_(tp);
    return;
  }

  const int dx_stable = static_cast<int>(tp.x) - static_cast<int>(this->swipe_.stable_x);
  const int dy_stable = static_cast<int>(tp.y) - static_cast<int>(this->swipe_.stable_y);
  const int settle_distance_sq =
      static_cast<int>(this->swipe_settle_distance_px_) * static_cast<int>(this->swipe_settle_distance_px_);
  if (dx_stable * dx_stable + dy_stable * dy_stable >= settle_distance_sq) {
    this->swipe_.stable_x = tp.x;
    this->swipe_.stable_y = tp.y;
  }

  const int dx_peak = static_cast<int>(this->swipe_.stable_x) - static_cast<int>(this->swipe_.start_x);
  const int dy_peak = static_cast<int>(this->swipe_.stable_y) - static_cast<int>(this->swipe_.start_y);
  const int current_distance_sq = dx_peak * dx_peak + dy_peak * dy_peak;

  const int peak_dx = static_cast<int>(this->swipe_.peak_x) - static_cast<int>(this->swipe_.start_x);
  const int peak_dy = static_cast<int>(this->swipe_.peak_y) - static_cast<int>(this->swipe_.start_y);
  const int peak_distance_sq = peak_dx * peak_dx + peak_dy * peak_dy;
  if (current_distance_sq >= peak_distance_sq) {
    this->swipe_.peak_x = this->swipe_.stable_x;
    this->swipe_.peak_y = this->swipe_.stable_y;
  }
}

VieweSmartTouchscreen::SwipeDirection VieweSmartTouchscreen::finish_swipe_() {
  if (!this->swipe_.active) {
    return SWIPE_NONE;
  }

  const int dx = static_cast<int>(this->swipe_.peak_x) - static_cast<int>(this->swipe_.start_x);
  const int dy = static_cast<int>(this->swipe_.peak_y) - static_cast<int>(this->swipe_.start_y);
  const int abs_dx = std::abs(dx);
  const int abs_dy = std::abs(dy);
  const int major = std::max(abs_dx, abs_dy);
  const int minor = std::min(abs_dx, abs_dy);

  this->reset_swipe_();

  if (major < this->swipe_min_distance_px_) {
    return SWIPE_NONE;
  }
  if (static_cast<float>(major) < static_cast<float>(minor) * this->swipe_min_axis_ratio_) {
    return SWIPE_NONE;
  }

  if (abs_dx > abs_dy) {
    return dx < 0 ? SWIPE_LEFT : SWIPE_RIGHT;
  }
  return dy < 0 ? SWIPE_UP : SWIPE_DOWN;
}

void VieweSmartTouchscreen::reset_swipe_() { this->swipe_ = SwipeTracker(); }

void VieweSmartTouchscreen::trigger_swipe_(SwipeDirection direction) {
  switch (direction) {
    case SWIPE_UP:
      ESP_LOGD(TAG, "Swipe up detected");
      this->swipe_up_trigger_.trigger();
      break;
    case SWIPE_DOWN:
      ESP_LOGD(TAG, "Swipe down detected");
      this->swipe_down_trigger_.trigger();
      break;
    case SWIPE_LEFT:
      ESP_LOGD(TAG, "Swipe left detected");
      this->swipe_left_trigger_.trigger();
      break;
    case SWIPE_RIGHT:
      ESP_LOGD(TAG, "Swipe right detected");
      this->swipe_right_trigger_.trigger();
      break;
    case SWIPE_NONE:
    default:
      break;
  }
}

void VieweSmartTouchscreen::continue_setup_() {
  if (this->interrupt_pin_ != nullptr) {
    this->interrupt_pin_->setup();
    this->attach_interrupt_(this->interrupt_pin_, gpio::INTERRUPT_FALLING_EDGE);
  }

  if (!this->read_byte(REG_CHIP_ID, &this->chip_id_) && !this->skip_probe_) {
    this->status_set_error(LOG_STR("Failed to read touch chip ID"));
    this->mark_failed();
    return;
  }

  if (this->chip_id_ == 0) {
    if (!this->read_byte(REG_FACTORY_ID, &this->chip_id_) && !this->skip_probe_) {
      this->status_set_error(LOG_STR("Failed to read touch chip factory ID"));
      this->mark_failed();
      return;
    }
  }

  switch (this->chip_id_) {
    case CST716_CHIP_ID:
    case CST816S_CHIP_ID:
    case CST816T_CHIP_ID:
    case CST816D_CHIP_ID:
    case CST820_CHIP_ID:
    case CST826_CHIP_ID:
    case CST836_CHIP_ID:
      break;
    default:
      if (!this->skip_probe_) {
        ESP_LOGE(TAG, "Unknown touch chip ID: 0x%02X", this->chip_id_);
        this->status_set_error(LOG_STR("Unknown touch chip ID"));
        this->mark_failed();
        return;
      }
      break;
  }

  this->write_byte(REG_DIS_AUTOSLEEP, 0x01);
  this->write_byte(REG_IRQ_CTL, IRQ_EN_TOUCH);
  this->write_byte(REG_SLEEP, 0x00);

  if (this->x_raw_max_ == this->x_raw_min_) {
    this->x_raw_max_ = this->display_->get_native_width();
  }
  if (this->y_raw_max_ == this->y_raw_min_) {
    this->y_raw_max_ = this->display_->get_native_height();
  }

  const uint16_t min_dimension = std::min(this->display_->get_width(), this->display_->get_height());
  this->swipe_min_distance_px_ =
      this->swipe_min_distance_ > 0 ? this->swipe_min_distance_ : std::max<uint16_t>(24, min_dimension / 8);
  this->swipe_settle_distance_px_ =
      this->swipe_settle_distance_ > 0 ? this->swipe_settle_distance_ : std::max<uint16_t>(3, min_dimension / 48);
}

void VieweSmartTouchscreen::setup() {
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(true);
    delay(5);
    this->reset_pin_->digital_write(false);
    delay(5);
    this->reset_pin_->digital_write(true);
    this->set_timeout(30, [this] { this->continue_setup_(); });
  } else {
    this->continue_setup_();
  }
}

void VieweSmartTouchscreen::update_touches() {
  uint8_t data[13];
  if (!this->read_bytes(REG_STATUS, data, sizeof(data))) {
    this->status_set_warning();
    return;
  }

  const uint8_t num_touches = data[REG_TOUCH_NUM] & 0x03;
  if (num_touches == 0) {
    this->trigger_swipe_(this->finish_swipe_());
    return;
  }

  const uint16_t x = encode_uint16(data[REG_XPOS_HIGH] & 0x0F, data[REG_XPOS_LOW]);
  const uint16_t y = encode_uint16(data[REG_YPOS_HIGH] & 0x0F, data[REG_YPOS_LOW]);
  ESP_LOGV(TAG, "Touch %u,%u", x, y);
  this->add_raw_touch_position_(0, x, y);
  auto it = this->touches_.find(0);
  if (it != this->touches_.end()) {
    this->update_swipe_(it->second);
  }
}

void VieweSmartTouchscreen::dump_config() {
  ESP_LOGCONFIG(TAG, "VieweSmart Touchscreen:");
  LOG_I2C_DEVICE(this);
  LOG_PIN("  Interrupt Pin: ", this->interrupt_pin_);
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  ESP_LOGCONFIG(TAG, "  X Raw Min: %d, X Raw Max: %d", this->x_raw_min_, this->x_raw_max_);
  ESP_LOGCONFIG(TAG, "  Y Raw Min: %d, Y Raw Max: %d", this->y_raw_min_, this->y_raw_max_);
  ESP_LOGCONFIG(TAG, "  Skip Probe: %s", YESNO(this->skip_probe_));
  ESP_LOGCONFIG(TAG, "  Chip ID: 0x%02X", this->chip_id_);
  ESP_LOGCONFIG(TAG, "  Swipe Min Distance: %u px", this->swipe_min_distance_px_);
  ESP_LOGCONFIG(TAG, "  Swipe Settle Distance: %u px", this->swipe_settle_distance_px_);
  ESP_LOGCONFIG(TAG, "  Swipe Min Axis Ratio: %.2f", this->swipe_min_axis_ratio_);
}

float VieweSmartTouchscreen::get_setup_priority() const { return setup_priority::IO; }

}  // namespace viewesmart_touchscreen
}  // namespace esphome
