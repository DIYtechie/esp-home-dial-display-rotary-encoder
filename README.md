# VieweSmart ESPHome External Components

External components for the VieweSmart / VIEWE Smart round ESP32-S3 dial display board family.

This repository currently provides:

- `viewesmart_rotary_encoder`
- `viewesmart_knob_button`
- `viewesmart_touchscreen`

The goal is to keep LVGL and application logic in YAML while replacing the board input layer with a more robust encoder path and a reusable touchscreen integration.

## AMOLED dial dashboard

This project accompanies my search for a sofa remote that can control volume, lights, blinds, skylights and media playback. The target board is the VIEWE / VieweSmart UEDX46460015 family with a 1.5-inch, 466 × 466 AMOLED touchscreen and a physical rotary/push knob. Verify your board revision and pinout before flashing.

The [full dashboard YAML](examples/dial_display_dashboard.yaml) provides:

- Circular volume and brightness indicators, with configurable colours and backgrounds.
- Split layouts for blinds and skylights, showing the selected target while the device moves.
- A three-segment receiver input selector.
- Media controls, analogue/digital clocks and a page-selection menu.
- Rotation, physical press/hold, touch regions and directional swipe handling.

The touchscreen and encoder components can also be used with your own UI. The example is a starting point based on my configuration, not a universal configuration for every receiver or TV.

Tested with ESPHome 2026.3.2: both example configurations validate, and the full dashboard compiles for ESP32-S3. Physical behaviour still depends on your board and Home Assistant integrations.

### Run the full example

1. Clone or download this repository, keeping `components/` and `examples/` together. The examples intentionally load the adjacent local components so they can be validated before publishing. For a standalone configuration, use the GitHub component source shown below with `ref: main`.
2. Copy `examples/secrets.yaml.example` to `examples/secrets.yaml`. Enter your own Wi-Fi, OTA and fallback credentials, plus an ESPHome-generated API encryption key.
3. Search the dashboard YAML for `example_` and replace the Home Assistant entity IDs. Replace `Game Console`, `Apple TV`, and `Camera` with the receiver's actual source names. TV menu/OK commands and optional ceiling-light/skylight-blind/power-off controls depend on your integrations; adapt or remove them.
4. Keep both included PNG icons beside the YAML. The clock font is downloaded from Google Fonts on the first build.
5. Validate with `esphome config examples/dial_display_dashboard.yaml`, then compile with `esphome compile examples/dial_display_dashboard.yaml`. Flash and add the device to Home Assistant. Allow the ESPHome device to perform Home Assistant actions in the integration's device configuration.

The example includes Home Assistant API encryption and OTA updates, but no personal credentials. Never commit your `secrets.yaml`. Control limits, such as maximum volume, are examples you should review for your setup. Display wake/sleep rules based on room presence or TV use belong to your own Home Assistant automations; they are not supplied as a ready-made package here.

### Encoder behaviour

The knob exposes incremental A/B quadrature signals rather than an absolute angle. Direction comes from their relative phase; `resolution` controls how many logical steps are reported per quadrature cycle. Synchronising the logical value preserves the hardware count and partial cycle. Do not equate the resolution setting with the number of physical detents per turn.

## What is included

### `viewesmart_rotary_encoder`
- ESPHome `sensor` platform
- ESP32 hardware pulse counter (PCNT) quadrature decoder; reports each counted logical step without decoder acceleration
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
  - source:
      type: git
      url: https://github.com/DIYtechie/esp-home-dial-display-rotary-encoder
      ref: main
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

## USB power when using a Mac

In my Mac setup, the dial needed a USB hub with its own external power supply to provide steady power. Use a powered hub rather than relying on the Mac connection alone; for flashing or serial logs, the hub and cable must also support USB data. This is a setup note from my unit, not a verified requirement for every Mac or board revision.

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

The [minimal example](examples/dial_display.yaml) shows the input integration. The [full dashboard example](examples/dial_display_dashboard.yaml) includes the custom pages and Home Assistant control logic.

## Notes

- ESP-IDF only is the intended target for this first version.
- The decoder uses PCNT hardware counting. The legacy `acceleration_mode` and `max_step` settings/actions remain accepted for YAML compatibility, but do not accelerate the current decoder. Very fast spins still require hardware testing; counted events are not a guarantee that no electrical transitions were missed.
- If touch does not respond on your board revision, verify the I2C pins first and then try explicit `interrupt_pin` / `reset_pin` values from your board docs or logic trace.
