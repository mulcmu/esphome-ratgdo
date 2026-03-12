import logging

import esphome.codegen as cg
import esphome.config_validation as cv
import voluptuous as vol
from esphome import automation, pins
from esphome.components import binary_sensor
from esphome.const import CONF_ID, CONF_TRIGGER_ID
from esphome.core import CORE

_LOGGER = logging.getLogger(__name__)

DEPENDENCIES = ["preferences"]
MULTI_CONF = True


ratgdo_ns = cg.esphome_ns.namespace("ratgdo")
RATGDO = ratgdo_ns.class_("RATGDOComponent", cg.Component)


SyncFailed = ratgdo_ns.class_("SyncFailed", automation.Trigger.template())

CONF_OUTPUT_GDO = "output_gdo_pin"
DEFAULT_OUTPUT_GDO = (
    "D4"  # D4 red control terminal / GarageDoorOpener (UART1 TX) pin is D4 on D1 Mini
)
CONF_INPUT_GDO = "input_gdo_pin"
DEFAULT_INPUT_GDO = (
    "D2"  # D2 red control terminal / GarageDoorOpener (UART1 RX) pin is D2 on D1 Mini
)
CONF_INPUT_OBST = "input_obst_pin"
DEFAULT_INPUT_OBST = "D7"  # D7 black obstruction sensor terminal

CONF_DISCRETE_OPEN_PIN = "discrete_open_pin"
CONF_DISCRETE_CLOSE_PIN = "discrete_close_pin"

CONF_RATGDO_ID = "ratgdo_id"

CONF_ON_SYNC_FAILED = "on_sync_failed"

CONF_PROTOCOL = "protocol"

PROTOCOL_SECPLUSV1 = "secplusv1"
PROTOCOL_SECPLUSV2 = "secplusv2"
PROTOCOL_DRYCONTACT = "drycontact"
SUPPORTED_PROTOCOLS = [PROTOCOL_SECPLUSV1, PROTOCOL_SECPLUSV2, PROTOCOL_DRYCONTACT]

CONF_DRY_CONTACT_OPEN_SENSOR = "dry_contact_open_sensor"
CONF_DRY_CONTACT_CLOSE_SENSOR = "dry_contact_close_sensor"
CONF_DRY_CONTACT_SENSOR_GROUP = "dry_contact_sensor_group"


def validate_protocol(config):
    if config.get(CONF_PROTOCOL, None) == PROTOCOL_DRYCONTACT and (
        CONF_DRY_CONTACT_CLOSE_SENSOR not in config
        or CONF_DRY_CONTACT_OPEN_SENSOR not in config
    ):
        raise cv.Invalid(
            "dry_contact_close_sensor and dry_contact_open_sensor are required when using protocol drycontact"
        )
    if config.get(CONF_PROTOCOL, None) != PROTOCOL_DRYCONTACT and (
        CONF_DRY_CONTACT_CLOSE_SENSOR in config
        or CONF_DRY_CONTACT_OPEN_SENSOR in config
    ):
        raise cv.Invalid(
            "dry_contact_close_sensor and dry_contact_open_sensor are only valid when using protocol drycontact"
        )
    #    if config.get(CONF_PROTOCOL, None) == PROTOCOL_DRYCONTACT and CONF_DRY_CONTACT_OPEN_SENSOR not in config:
    #        raise cv.Invalid("dry_contact_open_sensor is required when using protocol drycontact")
    return config


CONF_ESP32_UART_NUM = "esp32_uart_num"


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(RATGDO),
            cv.Optional(
                CONF_OUTPUT_GDO, default=DEFAULT_OUTPUT_GDO
            ): pins.gpio_output_pin_schema,
            cv.Optional(
                CONF_INPUT_GDO, default=DEFAULT_INPUT_GDO
            ): pins.gpio_input_pin_schema,
            cv.Optional(CONF_INPUT_OBST, default=DEFAULT_INPUT_OBST): cv.Any(
                cv.none, pins.gpio_input_pin_schema
            ),
            cv.Optional(CONF_DISCRETE_OPEN_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_DISCRETE_CLOSE_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_ON_SYNC_FAILED): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(SyncFailed),
                }
            ),
            cv.Optional(CONF_PROTOCOL, default=PROTOCOL_SECPLUSV2): cv.All(
                vol.In(SUPPORTED_PROTOCOLS)
            ),
            # cv.Inclusive(CONF_DRY_CONTACT_OPEN_SENSOR,CONF_DRY_CONTACT_SENSOR_GROUP): cv.use_id(binary_sensor.BinarySensor),
            # cv.Inclusive(CONF_DRY_CONTACT_CLOSE_SENSOR,CONF_DRY_CONTACT_SENSOR_GROUP): cv.use_id(binary_sensor.BinarySensor),
            cv.Optional(CONF_DRY_CONTACT_OPEN_SENSOR): cv.use_id(
                binary_sensor.BinarySensor
            ),
            cv.Optional(CONF_DRY_CONTACT_CLOSE_SENSOR): cv.use_id(
                binary_sensor.BinarySensor
            ),
            # ESP32-only: select which hardware UART port (0, 1, or 2) the GDO
            # communication uses. Default is 1 (UART_NUM_1). Change this if
            # another ESPHome 'uart:' component is already using that port.
            cv.Optional(CONF_ESP32_UART_NUM, default=1): cv.int_range(min=0, max=2),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    validate_protocol,
)

RATGDO_CLIENT_SCHMEA = cv.Schema(
    {
        cv.GenerateID(CONF_RATGDO_ID): cv.use_id(RATGDO),
    }
)


async def register_ratgdo_child(var, config):
    parent = await cg.get_variable(config[CONF_RATGDO_ID])
    cg.add(var.set_parent(parent))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    pin = await cg.gpio_pin_expression(config[CONF_OUTPUT_GDO])
    cg.add(var.set_output_gdo_pin(pin))
    pin = await cg.gpio_pin_expression(config[CONF_INPUT_GDO])
    cg.add(var.set_input_gdo_pin(pin))
    if config.get(CONF_INPUT_OBST):
        pin = await cg.gpio_pin_expression(config[CONF_INPUT_OBST])
        cg.add(var.set_input_obst_pin(pin))

    if CORE.is_esp32:
        esp32_uart_num = config[CONF_ESP32_UART_NUM]
        cg.add(var.set_esp32_uart_num(esp32_uart_num))

        # Count uart: components so the C++ compile-time check can detect conflicts.
        uart_configs = CORE.config.get("uart", [])
        if not isinstance(uart_configs, list):
            uart_configs = [uart_configs]
        num_uarts = len(uart_configs)

        # Emit defines consumed by ratgdo_uart_check.h.
        cg.add_build_flag(f"-DRATGDO_ESP32_UART_PORT={esp32_uart_num}")
        cg.add_build_flag(f"-DRATGDO_NUM_ESPHOME_UARTS={num_uarts}")

        # Python-side early check: catch the conflict before C++ compilation starts
        # so the user gets a clear, actionable error message.
        if num_uarts > 0:
            if CORE.using_esp_idf:
                # ESP-IDF: uart: components claim UART_NUM_0, UART_NUM_1, …
                conflicting = set(range(num_uarts))
            else:
                # Arduino: UART_NUM_0 is the logger; uart: components start at UART_NUM_1
                conflicting = set(range(1, num_uarts + 1))

            if esp32_uart_num in conflicting:
                raise cv.Invalid(
                    f"esp32_uart_num={esp32_uart_num} conflicts with a configured "
                    f"uart: component (ports {sorted(conflicting)} are already in use). "
                    f"Set esp32_uart_num to a port outside that range.",
                    [CONF_ESP32_UART_NUM],
                )

    if config.get(CONF_DRY_CONTACT_OPEN_SENSOR):
        dry_contact_open_sensor = await cg.get_variable(
            config[CONF_DRY_CONTACT_OPEN_SENSOR]
        )
        cg.add(var.set_dry_contact_open_sensor(dry_contact_open_sensor))

    if config.get(CONF_DRY_CONTACT_CLOSE_SENSOR):
        dry_contact_close_sensor = await cg.get_variable(
            config[CONF_DRY_CONTACT_CLOSE_SENSOR]
        )
        cg.add(var.set_dry_contact_close_sensor(dry_contact_close_sensor))

    for conf in config.get(CONF_ON_SYNC_FAILED, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)

    cg.add_library(
        name="secplus",
        repository="https://github.com/ratgdo/secplus#f98c3220356c27717a25102c0b35815ebbd26ccc",
        version=None,
    )
    # espsoftwareserial is only needed on ESP8266; ESP32 uses the built-in hardware UART driver
    if CORE.is_esp8266:
        cg.add_library(
            name="espsoftwareserial",
            repository="https://github.com/ratgdo/espsoftwareserial#autobaud",
            version=None,
        )

    if config[CONF_PROTOCOL] == PROTOCOL_SECPLUSV1:
        cg.add_build_flag("-DPROTOCOL_SECPLUSV1")
    elif config[CONF_PROTOCOL] == PROTOCOL_SECPLUSV2:
        cg.add_build_flag("-DPROTOCOL_SECPLUSV2")
    elif config[CONF_PROTOCOL] == PROTOCOL_DRYCONTACT:
        cg.add_build_flag("-DPROTOCOL_DRYCONTACT")
    cg.add(var.init_protocol())

    if config.get(CONF_DISCRETE_OPEN_PIN):
        pin = await cg.gpio_pin_expression(config[CONF_DISCRETE_OPEN_PIN])
        cg.add(var.set_discrete_open_pin(pin))
    if config.get(CONF_DISCRETE_CLOSE_PIN):
        pin = await cg.gpio_pin_expression(config[CONF_DISCRETE_CLOSE_PIN])
        cg.add(var.set_discrete_close_pin(pin))
