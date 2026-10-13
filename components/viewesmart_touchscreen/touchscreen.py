from esphome import pins
from esphome import automation
import esphome.codegen as cg
from esphome.components import i2c, touchscreen
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_INTERRUPT_PIN, CONF_RESET_PIN

CONF_SKIP_PROBE = "skip_probe"
CONF_ON_SWIPE_UP = "on_swipe_up"
CONF_ON_SWIPE_DOWN = "on_swipe_down"
CONF_ON_SWIPE_LEFT = "on_swipe_left"
CONF_ON_SWIPE_RIGHT = "on_swipe_right"
CONF_SWIPE_MIN_DISTANCE = "swipe_min_distance"
CONF_SWIPE_SETTLE_DISTANCE = "swipe_settle_distance"
CONF_SWIPE_MIN_AXIS_RATIO = "swipe_min_axis_ratio"

viewesmart_touchscreen_ns = cg.esphome_ns.namespace("viewesmart_touchscreen")
VieweSmartTouchscreen = viewesmart_touchscreen_ns.class_(
    "VieweSmartTouchscreen", touchscreen.Touchscreen, i2c.I2CDevice
)

CONFIG_SCHEMA = (
    touchscreen.TOUCHSCREEN_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(VieweSmartTouchscreen),
            cv.Optional(CONF_INTERRUPT_PIN): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_RESET_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_SKIP_PROBE, default=False): cv.boolean,
            cv.Optional(CONF_SWIPE_MIN_DISTANCE): cv.int_range(min=0, max=1000),
            cv.Optional(CONF_SWIPE_SETTLE_DISTANCE): cv.int_range(min=0, max=200),
            cv.Optional(CONF_SWIPE_MIN_AXIS_RATIO, default=1.6): cv.float_range(
                min=1.0, max=10.0
            ),
            cv.Optional(CONF_ON_SWIPE_UP): automation.validate_automation(single=True),
            cv.Optional(CONF_ON_SWIPE_DOWN): automation.validate_automation(single=True),
            cv.Optional(CONF_ON_SWIPE_LEFT): automation.validate_automation(single=True),
            cv.Optional(CONF_ON_SWIPE_RIGHT): automation.validate_automation(single=True),
        }
    ).extend(i2c.i2c_device_schema(0x15))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await touchscreen.register_touchscreen(var, config)
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_skip_probe(config[CONF_SKIP_PROBE]))
    cg.add(var.set_swipe_min_axis_ratio(config[CONF_SWIPE_MIN_AXIS_RATIO]))
    if swipe_min_distance := config.get(CONF_SWIPE_MIN_DISTANCE):
        cg.add(var.set_swipe_min_distance(swipe_min_distance))
    if swipe_settle_distance := config.get(CONF_SWIPE_SETTLE_DISTANCE):
        cg.add(var.set_swipe_settle_distance(swipe_settle_distance))
    if interrupt_pin := config.get(CONF_INTERRUPT_PIN):
        cg.add(var.set_interrupt_pin(await cg.gpio_pin_expression(interrupt_pin)))
    if reset_pin := config.get(CONF_RESET_PIN):
        cg.add(var.set_reset_pin(await cg.gpio_pin_expression(reset_pin)))

    if CONF_ON_SWIPE_UP in config:
        await automation.build_automation(
            var.get_swipe_up_trigger(), [], config[CONF_ON_SWIPE_UP]
        )
    if CONF_ON_SWIPE_DOWN in config:
        await automation.build_automation(
            var.get_swipe_down_trigger(), [], config[CONF_ON_SWIPE_DOWN]
        )
    if CONF_ON_SWIPE_LEFT in config:
        await automation.build_automation(
            var.get_swipe_left_trigger(), [], config[CONF_ON_SWIPE_LEFT]
        )
    if CONF_ON_SWIPE_RIGHT in config:
        await automation.build_automation(
            var.get_swipe_right_trigger(), [], config[CONF_ON_SWIPE_RIGHT]
        )
