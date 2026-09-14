import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button
from . import ups_hid_ns, CONF_UPS_HID_ID, UpsHidComponent

DEPENDENCIES = ["ups_hid"]

UpsHidButton = ups_hid_ns.class_("UpsHidButton", button.Button, cg.Component)

CONF_BEEPER_ACTION = "beeper_action"
CONF_TEST_ACTION = "test_action"
CONF_SHUTDOWN_ACTION = "shutdown_action"
CONF_TEST_DURATION = "test_duration"
CONF_REQUIRE_CONFIRMATION = "require_confirmation"

BEEPER_ACTIONS = {
    "enable": "enable",
    "disable": "disable",
    "mute": "mute",
    "test": "test"
}

TEST_ACTIONS = {
    "battery_quick": "battery_quick",
    "battery_deep": "battery_deep",
    "battery_timed": "battery_timed",
    "battery_stop": "battery_stop",
    "ups_test": "ups_test",
    "ups_stop": "ups_stop"
}

# shutdown_return, shutdown_stayoff and load_off cut power to everything on the
# UPS. Unless require_confirmation is false they only run on a second press
# within 5 seconds.
SHUTDOWN_ACTIONS = {
    "shutdown_return": "shutdown_return",
    "shutdown_stayoff": "shutdown_stayoff",
    "shutdown_cancel": "shutdown_cancel",
    "load_off": "load_off",
    "load_on": "load_on",
}

ACTION_KEYS = (CONF_BEEPER_ACTION, CONF_TEST_ACTION, CONF_SHUTDOWN_ACTION)

TEST_DURATION_MIN_MINUTES = 1
TEST_DURATION_MAX_MINUTES = 99


def validate_button_config(config):
    configured = [key for key in ACTION_KEYS if key in config]
    if len(configured) != 1:
        raise cv.Invalid(
            "Specify exactly one of 'beeper_action', 'test_action' or 'shutdown_action'"
        )

    if CONF_TEST_DURATION in config:
        if config.get(CONF_TEST_ACTION) != "battery_timed":
            raise cv.Invalid("'test_duration' only applies to test_action: battery_timed")
        minutes = config[CONF_TEST_DURATION].total_seconds // 60
        if not TEST_DURATION_MIN_MINUTES <= minutes <= TEST_DURATION_MAX_MINUTES:
            raise cv.Invalid(
                f"'test_duration' must be {TEST_DURATION_MIN_MINUTES}-"
                f"{TEST_DURATION_MAX_MINUTES} minutes"
            )

    return config


CONFIG_SCHEMA = cv.All(
    button.button_schema(UpsHidButton).extend({
        cv.GenerateID(CONF_UPS_HID_ID): cv.use_id(UpsHidComponent),
        cv.Optional(CONF_BEEPER_ACTION): cv.enum(BEEPER_ACTIONS, lower=True),
        cv.Optional(CONF_TEST_ACTION): cv.enum(TEST_ACTIONS, lower=True),
        cv.Optional(CONF_SHUTDOWN_ACTION): cv.enum(SHUTDOWN_ACTIONS, lower=True),
        cv.Optional(CONF_TEST_DURATION): cv.positive_time_period_minutes,
        cv.Optional(CONF_REQUIRE_CONFIRMATION, default=True): cv.boolean,
    }).extend(cv.COMPONENT_SCHEMA),
    validate_button_config,
)


async def to_code(config):
    var = await button.new_button(config)
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_UPS_HID_ID])
    cg.add(var.set_ups_hid_parent(parent))

    # Set action based on which type was configured
    if CONF_BEEPER_ACTION in config:
        cg.add(var.set_beeper_action(config[CONF_BEEPER_ACTION]))
    elif CONF_TEST_ACTION in config:
        cg.add(var.set_test_action(config[CONF_TEST_ACTION]))
        if CONF_TEST_DURATION in config:
            cg.add(var.set_test_duration_minutes(config[CONF_TEST_DURATION].total_seconds // 60))
    elif CONF_SHUTDOWN_ACTION in config:
        cg.add(var.set_shutdown_action(config[CONF_SHUTDOWN_ACTION]))
        cg.add(var.set_require_confirmation(config[CONF_REQUIRE_CONFIRMATION]))
