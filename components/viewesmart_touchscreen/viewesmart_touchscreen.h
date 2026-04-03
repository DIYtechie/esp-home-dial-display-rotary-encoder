#pragma once

#include "esphome/components/i2c/i2c.h"
#include "esphome/components/touchscreen/touchscreen.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace viewesmart_touchscreen {

static const char *const TAG = "viewesmart_touch";

static const uint8_t REG_STATUS = 0x00;
static const uint8_t REG_TOUCH_NUM = 0x02;
static const uint8_t REG_XPOS_HIGH = 0x03;
static const uint8_t REG_XPOS_LOW = 0x04;
static const uint8_t REG_YPOS_HIGH = 0x05;
static const uint8_t REG_YPOS_LOW = 0x06;
static const uint8_t REG_SLEEP = 0xE5;
static const uint8_t REG_IRQ_CTL = 0xFA;
static const uint8_t REG_CHIP_ID = 0xA7;
static const uint8_t REG_FACTORY_ID = 0xAA;
static const uint8_t REG_DIS_AUTOSLEEP = 0xFE;
static const uint8_t IRQ_EN_TOUCH = 0x70;

static const uint8_t CST716_CHIP_ID = 0x20;
static const uint8_t CST816S_CHIP_ID = 0xB4;
static const uint8_t CST816T_CHIP_ID = 0xB5;
static const uint8_t CST816D_CHIP_ID = 0xB6;
static const uint8_t CST820_CHIP_ID = 0xB7;
static const uint8_t CST826_CHIP_ID = 0x11;
static const uint8_t CST836_CHIP_ID = 0x13;

class VieweSmartTouchscreen : public touchscreen::Touchscreen, public i2c::I2CDevice {
 public:
  void setup() override;
  void update_touches() override;
  void dump_config() override;
  float get_setup_priority() const override;

  void set_interrupt_pin(InternalGPIOPin *pin) { this->interrupt_pin_ = pin; }
  void set_reset_pin(GPIOPin *pin) { this->reset_pin_ = pin; }
  void set_skip_probe(bool skip_probe) { this->skip_probe_ = skip_probe; }

 protected:
  void continue_setup_();

  InternalGPIOPin *interrupt_pin_{nullptr};
  GPIOPin *reset_pin_{nullptr};
  uint8_t chip_id_{0};
  bool skip_probe_{false};
};

}  // namespace viewesmart_touchscreen
}  // namespace esphome

