from pathlib import Path

import esphome.config_validation as cv
from esphome import codegen as cg
from esphome.const import CONF_ID, CONF_NAME, SOURCE_FILE_EXTENSIONS
from esphome.core import CORE
from esphome.yaml_util import ESPHomeDumper

from .driver_loader import CppDriver, Driver, DriverManager

CODEOWNERS = ["@kubasaw"]
CONF_ALL = "all"
CONF_DRIVERS = "drivers"
CONF_FIELDS = "fields"

CURRENT_DIR = Path(__file__).parent
SOURCE_FILE_EXTENSIONS.add(".cc")


wmbus_common_ns = cg.esphome_ns.namespace("wmbus_common")
WMBusCommon = wmbus_common_ns.class_("WMBusCommon", cg.Component)


ESPHomeDumper.add_multi_representer(
    Driver, lambda s, v: s.represent_stringify(v.name)
)


def maybe_all(replacements):
    def validator(v):
        if v == CONF_ALL:
            return replacements
        else:
            return v

    return validator


def driver_field_validator(conf):
    driver = conf[CONF_NAME]

    conf[CONF_FIELDS] = cv.All(
        maybe_all(driver.available_fields),
        [driver.request_field],
    )(conf[CONF_FIELDS])

    return conf


DRIVER_ENTRY_SCHEMA = cv.maybe_simple_value(
    {
        cv.Required(CONF_NAME): cv.All(
            cv.one_of(*DriverManager.available_drivers), DriverManager.request_driver
        ),
        cv.Optional(CONF_FIELDS, default=CONF_ALL): cv.valid,
    },
    driver_field_validator,
    key=CONF_NAME,
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(WMBusCommon),
        cv.Optional(CONF_DRIVERS, default=[]): cv.All(
            maybe_all(DriverManager.available_drivers), [DRIVER_ENTRY_SCHEMA]
        ),
    }
)


def _request_configured_drivers(config):
    """Restore driver requests from validated config before source generation.

    ESPHome may reload external component Python modules between validation and
    code generation. DriverManager's in-memory request set must therefore not
    be the sole source of truth. The validated config already contains Driver
    objects, so rebuild the request set from it immediately before syncing the
    generated C++ driver sources.
    """
    for driver_config in config.get(CONF_DRIVERS, []):
        driver = driver_config[CONF_NAME]
        DriverManager.request_driver(driver.name)

    for meter_config in CORE.config.get("wmbus_meter", []):
        driver = meter_config.get("type")
        if isinstance(driver, Driver):
            DriverManager.request_driver(driver.name)
        elif driver is not None:
            DriverManager.request_driver(str(driver).split(":", 1)[0])


async def to_code(config):
    cg.add_define(
        "WMBUSMETERS_TAG",
        CURRENT_DIR.joinpath(".wmbusmeters_tag").read_text(),
    )

    _request_configured_drivers(config)
    target_dir = CORE.relative_src_path("wmbusmeters_drivers")
    DriverManager.sync_to_directory(target_dir)

    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(var.load_drivers())
    await cg.register_component(var, config)


def FILTER_SOURCE_FILES() -> list[str]:
    return [
        d.source_path.name
        for d in DriverManager._all_drivers.values()
        if isinstance(d, CppDriver)
    ]
