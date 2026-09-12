from contextlib import suppress
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins, automation
from esphome.components import spi
from esphome.cpp_generator import LambdaExpression
from esphome.const import (
    CONF_ID,
    CONF_RESET_PIN,
    CONF_IRQ_PIN,
    CONF_TRIGGER_ID,
    CONF_FORMAT,
    CONF_DATA,
)
from pathlib import Path

CODEOWNERS = ["@kubasaw"]

DEPENDENCIES = ["esp32", "spi"]

AUTO_LOAD = ["wmbus_common"]

MULTI_CONF = True

CONF_RADIO_ID = "radio_id"
CONF_ON_PACKET = "on_packet"
CONF_ON_FRAME = "on_frame"
CONF_RADIO_TYPE = "radio_type"
CONF_MARK_AS_HANDLED = "mark_as_handled"
CONF_PACKET_TRIGGER_ID = "packet_trigger_id"
CONF_METER_ID = "meter_id"
CONF_PERIOD = "period"
CONF_VERSION = "version"
CONF_DEVICE_TYPE = "device_type"
CONF_AES_KEY = "aes_key"
CONF_ATTEMPTS = "attempts"
CONF_POWER_DBM = "power_dbm"
CONF_ON_APATOR_PROGRAMMING_RESULT = "on_apator_programming_result"
CONF_APATOR_RESULT_TRIGGER_ID = "apator_result_trigger_id"
CONF_ON_APATOR_READ_RESULT = "on_apator_read_result"
CONF_APATOR_READ_RESULT_TRIGGER_ID = "apator_read_result_trigger_id"

radio_ns = cg.esphome_ns.namespace("wmbus_radio")
RadioComponent = radio_ns.class_("Radio", cg.Component)
RadioTransceiver = radio_ns.class_("RadioTransceiver", spi.SPIDevice, cg.Component)
Frame = radio_ns.class_("Frame")
FrameOutputFormat = Frame.enum("OutputFormat")
FramePtr = Frame.operator("ptr")
FrameTrigger = radio_ns.class_("FrameTrigger", automation.Trigger.template(FramePtr))

Packet = radio_ns.class_("Packet")
PacketPtr = Packet.operator("ptr")
PacketTrigger = radio_ns.class_("PacketTrigger", automation.Trigger.template(PacketPtr))
ApatorSetPeriodAction = radio_ns.class_("ApatorSetPeriodAction", automation.Action)
ApatorReadPeriodsAction = radio_ns.class_("ApatorReadPeriodsAction", automation.Action)
ApatorProgrammingResultTrigger = radio_ns.class_(
    "ApatorProgrammingResultTrigger",
    automation.Trigger.template(cg.std_string, cg.uint16, cg.uint16),
)
ApatorReadResultTrigger = radio_ns.class_(
    "ApatorReadResultTrigger",
    automation.Trigger.template(
        cg.std_string,
        cg.uint16,
        cg.uint16,
        cg.uint16,
        cg.uint16,
        cg.uint16,
    ),
)

TRANSCEIVER_NAMES = {
    r.stem.removeprefix("transceiver_").upper()
    for r in Path(__file__).parent.glob("transceiver_*.cpp")
    if r.is_file()
}

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(RadioComponent),
            cv.GenerateID(CONF_RADIO_ID): cv.declare_id(RadioTransceiver),
            cv.Required(CONF_RADIO_TYPE): cv.one_of(*TRANSCEIVER_NAMES, upper=True),
            cv.Required(CONF_RESET_PIN): pins.internal_gpio_output_pin_schema,
            cv.Required(CONF_IRQ_PIN): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_ON_FRAME): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(FrameTrigger),
                    cv.Optional(CONF_MARK_AS_HANDLED, default=False): cv.boolean,
                }
            ),
            cv.Optional(CONF_ON_PACKET): automation.validate_automation(
                {
                    cv.GenerateID(CONF_PACKET_TRIGGER_ID): cv.declare_id(PacketTrigger),
                }
            ),
            cv.Optional(CONF_ON_APATOR_PROGRAMMING_RESULT): automation.validate_automation(
                {
                    cv.GenerateID(CONF_APATOR_RESULT_TRIGGER_ID): cv.declare_id(
                        ApatorProgrammingResultTrigger
                    ),
                }
            ),
            cv.Optional(CONF_ON_APATOR_READ_RESULT): automation.validate_automation(
                {
                    cv.GenerateID(
                        CONF_APATOR_READ_RESULT_TRIGGER_ID
                    ): cv.declare_id(ApatorReadResultTrigger),
                }
            ),
        }
    )
    .extend(spi.spi_device_schema())
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    cg.add(cg.LineComment("WMBus RadioTransceiver"))

    config[CONF_RADIO_ID].type = radio_ns.class_(
        config[CONF_RADIO_TYPE], RadioTransceiver
    )
    radio_var = cg.new_Pvariable(config[CONF_RADIO_ID])

    reset_pin = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
    cg.add(radio_var.set_reset_pin(reset_pin))

    irq_pin = await cg.gpio_pin_expression(config[CONF_IRQ_PIN])
    cg.add(radio_var.set_irq_pin(irq_pin))

    await spi.register_spi_device(radio_var, config)
    await cg.register_component(radio_var, config)

    cg.add(cg.LineComment("WMBus Component"))
    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(var.set_radio(radio_var))

    await cg.register_component(var, config)

    for conf in config.get(CONF_ON_FRAME, []):
        trig = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var, conf[CONF_MARK_AS_HANDLED])
        await automation.build_automation(
            trig,
            [(FramePtr, "frame")],
            conf,
        )

    for conf in config.get(CONF_ON_PACKET, []):
        trig = cg.new_Pvariable(conf[CONF_PACKET_TRIGGER_ID], var)
        await automation.build_automation(
            trig,
            [(PacketPtr, "packet")],
            conf,
        )

    for conf in config.get(CONF_ON_APATOR_PROGRAMMING_RESULT, []):
        trig = cg.new_Pvariable(conf[CONF_APATOR_RESULT_TRIGGER_ID], var)
        await automation.build_automation(
            trig,
            [
                (cg.std_string, "result"),
                (cg.uint16, "desired_period"),
                (cg.uint16, "actual_period"),
            ],
            conf,
        )

    for conf in config.get(CONF_ON_APATOR_READ_RESULT, []):
        trig = cg.new_Pvariable(conf[CONF_APATOR_READ_RESULT_TRIGGER_ID], var)
        await automation.build_automation(
            trig,
            [
                (cg.std_string, "result"),
                (cg.uint16, "normal_period"),
                (cg.uint16, "economy_hours_period"),
                (cg.uint16, "economy_weekday_period"),
                (cg.uint16, "economy_month_day_period"),
                (cg.uint16, "economy_month_period"),
            ],
            conf,
        )


with suppress(ImportError):
    from esphome.components.socket_transmitter import (
        SOCKET_SEND_ACTION_SCHEMA,
        SocketTransmitterSendAction,
    )

    FRAME_SOCKET_SEND_SCHEMA = SOCKET_SEND_ACTION_SCHEMA.extend(
        {
            cv.Required(CONF_FORMAT): cv.one_of(
                "hex",
                "raw",
                "rtlwmbus",
                lower=True,
            ),
            cv.Optional(CONF_DATA): cv.invalid(
                "If you want to specify data to be sent, use generic 'socket_transmitter.send' action"
            ),
        }
    )

    @automation.register_action(
        "wmbus_radio.send_frame_with_socket",
        SocketTransmitterSendAction,
        FRAME_SOCKET_SEND_SCHEMA,
        synchronous=True,
    )
    async def send_frame_with_socket_to_code(config, action_id, template_arg, args):
        output_type = {
            "hex": cg.std_string,
            "raw": cg.std_vector.template(cg.uint8),
            "rtlwmbus": cg.std_string,
        }[config[CONF_FORMAT]]

        paren = await cg.get_variable(config[CONF_ID])
        var = cg.new_Pvariable(
            action_id, cg.TemplateArguments(output_type, *template_arg), paren
        )
        template_ = LambdaExpression(
            f"return frame->as_{config[CONF_FORMAT]}();", args, ""
        )

        cg.add(var.set_data(template_))

        return var


def validate_apator_meter_id(value):
    value = cv.string_strict(value)
    if not value.isdigit() or not 1 <= len(value) <= 8:
        raise cv.Invalid("meter_id must contain 1 to 8 decimal digits")
    return value


def validate_apator_key(value):
    value = cv.string_strict(value)
    if len(value) != 32 or any(c not in "0123456789abcdefABCDEF" for c in value):
        raise cv.Invalid("aes_key must contain exactly 32 hexadecimal characters")
    return value


def validate_apator_period(value):
    value = cv.int_range(min=10, max=2550)(value)
    if value % 10:
        raise cv.Invalid("period must be a multiple of 10 seconds")
    return value


APATOR_SET_PERIOD_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(RadioComponent),
        cv.Required(CONF_METER_ID): cv.templatable(validate_apator_meter_id),
        cv.Required(CONF_PERIOD): cv.templatable(validate_apator_period),
        cv.Optional(CONF_VERSION, default=5): cv.templatable(cv.int_range(min=0, max=255)),
        cv.Optional(CONF_DEVICE_TYPE, default=7): cv.templatable(cv.int_range(min=0, max=255)),
        cv.Optional(CONF_AES_KEY, default="00000000000000000000000000000000"): cv.templatable(validate_apator_key),
        cv.Optional(CONF_ATTEMPTS, default=3): cv.templatable(cv.int_range(min=1, max=10)),
        cv.Optional(CONF_POWER_DBM, default=10): cv.templatable(cv.int_range(min=2, max=17)),
    }
)


@automation.register_action(
    "wmbus_radio.apator_set_period",
    ApatorSetPeriodAction,
    APATOR_SET_PERIOD_SCHEMA,
    synchronous=True,
)
async def apator_set_period_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent)
    cg.add(var.set_meter_id(await cg.templatable(config[CONF_METER_ID], args, cg.std_string)))
    cg.add(var.set_period_seconds(await cg.templatable(config[CONF_PERIOD], args, cg.uint16)))
    cg.add(var.set_version(await cg.templatable(config[CONF_VERSION], args, cg.uint8)))
    cg.add(var.set_device_type(await cg.templatable(config[CONF_DEVICE_TYPE], args, cg.uint8)))
    cg.add(var.set_aes_key(await cg.templatable(config[CONF_AES_KEY], args, cg.std_string)))
    cg.add(var.set_attempts(await cg.templatable(config[CONF_ATTEMPTS], args, cg.uint8)))
    cg.add(var.set_power_dbm(await cg.templatable(config[CONF_POWER_DBM], args, cg.uint8)))
    return var


APATOR_READ_PERIODS_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(RadioComponent),
        cv.Required(CONF_METER_ID): cv.templatable(validate_apator_meter_id),
        cv.Optional(CONF_VERSION, default=5): cv.templatable(cv.int_range(min=0, max=255)),
        cv.Optional(CONF_DEVICE_TYPE, default=7): cv.templatable(cv.int_range(min=0, max=255)),
        cv.Optional(CONF_AES_KEY, default="00000000000000000000000000000000"): cv.templatable(validate_apator_key),
        cv.Optional(CONF_ATTEMPTS, default=3): cv.templatable(cv.int_range(min=1, max=10)),
        cv.Optional(CONF_POWER_DBM, default=10): cv.templatable(cv.int_range(min=2, max=17)),
    }
)


@automation.register_action(
    "wmbus_radio.apator_read_periods",
    ApatorReadPeriodsAction,
    APATOR_READ_PERIODS_SCHEMA,
    synchronous=True,
)
async def apator_read_periods_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent)
    cg.add(var.set_meter_id(await cg.templatable(config[CONF_METER_ID], args, cg.std_string)))
    cg.add(var.set_version(await cg.templatable(config[CONF_VERSION], args, cg.uint8)))
    cg.add(var.set_device_type(await cg.templatable(config[CONF_DEVICE_TYPE], args, cg.uint8)))
    cg.add(var.set_aes_key(await cg.templatable(config[CONF_AES_KEY], args, cg.std_string)))
    cg.add(var.set_attempts(await cg.templatable(config[CONF_ATTEMPTS], args, cg.uint8)))
    cg.add(var.set_power_dbm(await cg.templatable(config[CONF_POWER_DBM], args, cg.uint8)))
    return var
