from esphome import automation, pins
import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_MAX_VALUE,
    CONF_MIN_VALUE,
    CONF_PIN_A,
    CONF_PIN_B,
    CONF_RESOLUTION,
    CONF_RESTORE_MODE,
    CONF_VALUE,
    ICON_ROTATE_RIGHT,
    UNIT_STEPS,
)

viewesmart_rotary_encoder_ns = cg.esphome_ns.namespace("viewesmart_rotary_encoder")

VieweSmartRotaryEncoderRestoreMode = viewesmart_rotary_encoder_ns.enum(
    "VieweSmartRotaryEncoderRestoreMode"
)
RESTORE_MODES = {
    "RESTORE_DEFAULT_ZERO": VieweSmartRotaryEncoderRestoreMode.VIEWESMART_ROTARY_ENCODER_RESTORE_DEFAULT_ZERO,
    "ALWAYS_ZERO": VieweSmartRotaryEncoderRestoreMode.VIEWESMART_ROTARY_ENCODER_ALWAYS_ZERO,
}

VieweSmartRotaryEncoderResolution = viewesmart_rotary_encoder_ns.enum(
    "VieweSmartRotaryEncoderResolution"
)
RESOLUTIONS = {
    1: VieweSmartRotaryEncoderResolution.VIEWESMART_ROTARY_ENCODER_1_PULSE_PER_CYCLE,
    2: VieweSmartRotaryEncoderResolution.VIEWESMART_ROTARY_ENCODER_2_PULSES_PER_CYCLE,
    4: VieweSmartRotaryEncoderResolution.VIEWESMART_ROTARY_ENCODER_4_PULSES_PER_CYCLE,
}

CONF_PIN_RESET = "pin_reset"
CONF_ON_CLOCKWISE = "on_clockwise"
CONF_ON_ANTICLOCKWISE = "on_anticlockwise"
CONF_PUBLISH_INITIAL_VALUE = "publish_initial_value"
CONF_MAX_STEP = "max_step"

VieweSmartRotaryEncoderSensor = viewesmart_rotary_encoder_ns.class_(
    "VieweSmartRotaryEncoderSensor", sensor.Sensor, cg.Component
)
VieweSmartRotaryEncoderSetValueAction = viewesmart_rotary_encoder_ns.class_(
    "VieweSmartRotaryEncoderSetValueAction", automation.Action
)
VieweSmartRotaryEncoderSetMaxStepAction = viewesmart_rotary_encoder_ns.class_(
    "VieweSmartRotaryEncoderSetMaxStepAction", automation.Action
)


def validate_min_max_value(config):
    if CONF_MIN_VALUE in config and CONF_MAX_VALUE in config:
        min_val = config[CONF_MIN_VALUE]
        max_val = config[CONF_MAX_VALUE]
        if min_val >= max_val:
            raise cv.Invalid(
                f"Max value {max_val} must be greater than min value {min_val}"
            )
    return config


CONFIG_SCHEMA = cv.All(
    sensor.sensor_schema(
        VieweSmartRotaryEncoderSensor,
        unit_of_measurement=UNIT_STEPS,
        icon=ICON_ROTATE_RIGHT,
        accuracy_decimals=0,
    )
    .extend(
        {
            cv.Required(CONF_PIN_A): pins.internal_gpio_input_pin_schema,
            cv.Required(CONF_PIN_B): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_PIN_RESET): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_RESOLUTION, default=1): cv.enum(RESOLUTIONS, int=True),
            cv.Optional(CONF_MIN_VALUE): cv.int_,
            cv.Optional(CONF_MAX_VALUE): cv.int_,
            cv.Optional(CONF_MAX_STEP, default=10): cv.int_range(min=1, max=10),
            cv.Optional(CONF_PUBLISH_INITIAL_VALUE, default=False): cv.boolean,
            cv.Optional(CONF_RESTORE_MODE, default="RESTORE_DEFAULT_ZERO"): cv.enum(
                RESTORE_MODES, upper=True, space="_"
            ),
            cv.Optional(CONF_ON_CLOCKWISE): automation.validate_automation({}),
            cv.Optional(CONF_ON_ANTICLOCKWISE): automation.validate_automation({}),
        }
    )
    .extend(cv.COMPONENT_SCHEMA),
    validate_min_max_value,
)


async def to_code(config):
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)

    pin_a = await cg.gpio_pin_expression(config[CONF_PIN_A])
    cg.add(var.set_pin_a(pin_a))
    pin_b = await cg.gpio_pin_expression(config[CONF_PIN_B])
    cg.add(var.set_pin_b(pin_b))
    cg.add(var.set_publish_initial_value(config[CONF_PUBLISH_INITIAL_VALUE]))
    cg.add(var.set_restore_mode(config[CONF_RESTORE_MODE]))
    cg.add(var.set_resolution(config[CONF_RESOLUTION]))

    if CONF_PIN_RESET in config:
        pin_reset = await cg.gpio_pin_expression(config[CONF_PIN_RESET])
        cg.add(var.set_reset_pin(pin_reset))
    if CONF_MIN_VALUE in config:
        cg.add(var.set_min_value(config[CONF_MIN_VALUE]))
    if CONF_MAX_VALUE in config:
        cg.add(var.set_max_value(config[CONF_MAX_VALUE]))
    cg.add(var.set_max_step(config[CONF_MAX_STEP]))

    for conf in config.get(CONF_ON_CLOCKWISE, []):
        await automation.build_callback_automation(
            var, "add_on_clockwise_callback", [], conf
        )
    for conf in config.get(CONF_ON_ANTICLOCKWISE, []):
        await automation.build_callback_automation(
            var, "add_on_anticlockwise_callback", [], conf
        )


@automation.register_action(
    "sensor.viewesmart_rotary_encoder.set_value",
    VieweSmartRotaryEncoderSetValueAction,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(VieweSmartRotaryEncoderSensor),
            cv.Required(CONF_VALUE): cv.templatable(cv.int_),
        }
    ),
    synchronous=True,
)
async def sensor_viewesmart_rotary_encoder_set_value_to_code(
    config, action_id, template_arg, args
):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent)
    template_ = await cg.templatable(config[CONF_VALUE], args, int)
    cg.add(var.set_value(template_))
    return var


@automation.register_action(
    "sensor.viewesmart_rotary_encoder.set_max_step",
    VieweSmartRotaryEncoderSetMaxStepAction,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(VieweSmartRotaryEncoderSensor),
            cv.Required(CONF_MAX_STEP): cv.templatable(cv.int_range(min=1, max=10)),
        }
    ),
    synchronous=True,
)
async def sensor_viewesmart_rotary_encoder_set_max_step_to_code(
    config, action_id, template_arg, args
):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent)
    template_ = await cg.templatable(config[CONF_MAX_STEP], args, int)
    cg.add(var.set_max_step(template_))
    return var
