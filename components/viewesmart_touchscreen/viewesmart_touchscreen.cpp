#include "viewesmart_touchscreen.h"

#include "esphome/core/helpers.h"

namespace esphome {
namespace viewesmart_touchscreen {

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
    return;
  }

  const uint16_t x = encode_uint16(data[REG_XPOS_HIGH] & 0x0F, data[REG_XPOS_LOW]);
  const uint16_t y = encode_uint16(data[REG_YPOS_HIGH] & 0x0F, data[REG_YPOS_LOW]);
  ESP_LOGV(TAG, "Touch %u,%u", x, y);
  this->add_raw_touch_position_(0, x, y);
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
}

float VieweSmartTouchscreen::get_setup_priority() const { return setup_priority::IO; }

}  // namespace viewesmart_touchscreen
}  // namespace esphome
