# VieweSmart ESPHome External Components

External components for the VieweSmart / VIEWE Smart round ESP32-S3 dial display board family.

This repository currently provides:

- `viewesmart_rotary_encoder`
- `viewesmart_knob_button`
- `viewesmart_touchscreen`

The goal is to keep LVGL and application logic in YAML while replacing the board input layer with a more robust encoder path and a reusable touchscreen integration.

## What is included

### `viewesmart_rotary_encoder`
- ESPHome `sensor` platform
- Quadrature decoder with a larger ISR event queue than the stock ESPHome encoder
- `min_value`, `max_value`, `resolution`, `restore_mode`, `publish_initial_value`
- `on_value`, `on_clockwise`, `on_anticlockwise`
- action: `sensor.viewesmart_rotary_encoder.set_value`

### `viewesmart_knob_button`
- ESPHome `binary_sensor` platform
- Defaults to `GPIO0` with pull-up and inverted logic for the board knob switch
- Built-in debounce time
- Works with normal `on_press`, `on_release`, click automations, and filters

### `viewesmart_touchscreen`
- ESPHome `touchscreen` platform
- CST816/CST820-style I2C touch support for LVGL or `on_touch` automations
- Built-in swipe detection with `on_swipe_up`, `on_swipe_down`, `on_swipe_left`, `on_swipe_right`
- Optional interrupt/reset pins
- Keeps calibration and transform options aligned with standard ESPHome touchscreen usage

## Installation

### From GitHub

```yaml
external_components:
  - source: github://DIYtechie/esp-home-dial-display-rotary-encoder
    components:
      - viewesmart_rotary_encoder
      - viewesmart_knob_button
      - viewesmart_touchscreen
```

### From a local checkout

```yaml
external_components:
  - source:
      type: local
      path: /config/esphome/external/esp-home-dial-display-rotary-encoder/components
    components:
      - viewesmart_rotary_encoder
      - viewesmart_knob_button
      - viewesmart_touchscreen
```

## Board pins

These defaults and examples target the UEDX46460015 / 466x466 round AMOLED board family:

- Encoder A: `GPIO5`
- Encoder B: `GPIO6`
- Knob button: `GPIO0`
- Touch I2C SDA: `GPIO1`
- Touch I2C SCL: `GPIO3`
- Display enable: `GPIO17`

Touch reset/interrupt pins are left optional because board revisions and public examples are inconsistent there. If your board exposes working IRQ or reset lines, you can still configure them explicitly.

## Minimal usage

```yaml
i2c:
  sda: GPIO1
  scl: GPIO3
  scan: true

sensor:
  - platform: viewesmart_rotary_encoder
    id: knob_encoder
    name: "Knob value"
    pin_a:
      number: GPIO5
      mode:
        input: true
        pullup: true
    pin_b:
      number: GPIO6
      mode:
        input: true
        pullup: true
    resolution: 2
    min_value: 0
    max_value: 100
    restore_mode: ALWAYS_ZERO
    publish_initial_value: true

binary_sensor:
  - platform: viewesmart_knob_button
    id: knob_button
    name: "Knob Button"
    debounce_time: 15ms

touchscreen:
  platform: viewesmart_touchscreen
  id: board_touch
  display: round_display
  update_interval: 16ms
  on_swipe_left:
    then:
      - logger.log: "Swipe left"
  on_swipe_right:
    then:
      - logger.log: "Swipe right"
```

Swipe tuning is optional. By default the component auto-scales the swipe distance and jitter settle threshold from the display size, which works better across 240px and 466px round boards. If needed, you can override:

```yaml
touchscreen:
  platform: viewesmart_touchscreen
  id: board_touch
  display: round_display
  swipe_min_distance: 56
  swipe_settle_distance: 8
  swipe_min_axis_ratio: 1.6
```

## Migration notes

Your existing YAML can usually stay structured the same:

- replace `platform: rotary_encoder` with `platform: viewesmart_rotary_encoder`
- replace the GPIO button definition with `platform: viewesmart_knob_button`
- add `i2c:` and `platform: viewesmart_touchscreen` for touch support
- keep your LVGL pages, scripts, and automations in YAML

The included example at [examples/dial_display.yaml](/Users/grinderslev/Documents/esphome-dial-display-encoder/examples/dial_display.yaml) shows the intended migration shape.

## Notes

- ESP-IDF only is the intended target for this first version.
- The encoder queue is much larger than ESPHome's stock 8-slot event buffer, but hardware validation is still important for very aggressive spins.
- If touch does not respond on your board revision, verify the I2C pins first and then try explicit `interrupt_pin` / `reset_pin` values from your board docs or logic trace.

This README footer was updated on branch `codex/viewesmart-rotary-polling` to verify that the moved Codex workspace is still writable and connected correctly.
