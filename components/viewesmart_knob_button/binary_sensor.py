from esphome import pins
import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_PIN

CONF_DEBOUNCE_TIME = "debounce_time"

viewesmart_knob_button_ns = cg.esphome_ns.namespace("viewesmart_knob_button")
VieweSmartKnobButton = viewesmart_knob_button_ns.class_(
    "VieweSmartKnobButton", binary_sensor.BinarySensor, cg.Component
)

CONFIG_SCHEMA = (
    binary_sensor.binary_sensor_schema(VieweSmartKnobButton)
    .extend(
        {
            cv.Optional(CONF_PIN, default={"number": 0, "mode": {"input": True, "pullup": True}, "inverted": True}): pins.gpio_input_pin_schema,
            cv.Optional(CONF_DEBOUNCE_TIME, default="10ms"): cv.positive_time_period_milliseconds,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await binary_sensor.new_binary_sensor(config)
    await cg.register_component(var, config)

    pin = await cg.gpio_pin_expression(config[CONF_PIN])
    cg.add(var.set_pin(pin))
    cg.add(var.set_debounce_time(config[CONF_DEBOUNCE_TIME]))

