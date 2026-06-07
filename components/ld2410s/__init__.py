import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor, binary_sensor, number
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_DISTANCE,
    DEVICE_CLASS_OCCUPANCY,
    STATE_CLASS_MEASUREMENT,
    UNIT_CENTIMETER,
    ICON_RULER,
)

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "binary_sensor", "number"]
CODEOWNERS = ["@willy40"]

CONF_HAS_TARGET       = "has_target"
CONF_LAST_CMD_OK      = "last_command_success"
CONF_DISTANCE         = "distance"
CONF_MAX_GATE         = "max_distance_gate"
CONF_MIN_GATE         = "min_distance_gate"
CONF_NONE_DURATION    = "none_duration"
CONF_OFF_DELAY        = "off_delay"
CONF_GATE_ENERGY      = "gate_energy"   # list of 0–16 sensor entries

NUM_GATES = 16

ld2410s_ns = cg.esphome_ns.namespace("ld2410s")

LD2410SComponent = ld2410s_ns.class_(
    "LD2410SComponent", cg.PollingComponent, uart.UARTDevice
)
LD2410SNumber = ld2410s_ns.class_(
    "LD2410SNumber", number.Number, cg.Component
)

GATE_ENERGY_SCHEMA = sensor.sensor_schema(
    unit_of_measurement="",
    accuracy_decimals=0,
    state_class=STATE_CLASS_MEASUREMENT,
    icon="mdi:alpha-e-box",
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(LD2410SComponent),
            cv.Optional(CONF_OFF_DELAY, default="5s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_HAS_TARGET): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_OCCUPANCY,
            ),
            cv.Optional(CONF_LAST_CMD_OK): binary_sensor.binary_sensor_schema(),
            cv.Optional(CONF_DISTANCE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CENTIMETER,
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
                device_class=DEVICE_CLASS_DISTANCE,
                icon=ICON_RULER,
            ),
            cv.Optional(CONF_MAX_GATE):      number.number_schema(LD2410SNumber),
            cv.Optional(CONF_MIN_GATE):      number.number_schema(LD2410SNumber),
            cv.Optional(CONF_NONE_DURATION): number.number_schema(LD2410SNumber),
            # gate_energy: list of up to 16 sensor definitions
            # each entry must have an 'index' key (0–15) so you can define only
            # the gates you care about without creating all 16
            cv.Optional(CONF_GATE_ENERGY): cv.ensure_list(
                GATE_ENERGY_SCHEMA.extend(
                    {cv.Required("index"): cv.int_range(min=0, max=NUM_GATES - 1)}
                )
            ),
        }
    )
    .extend(cv.polling_component_schema("15s"))
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_off_delay(config[CONF_OFF_DELAY]))

    if has_target_config := config.get(CONF_HAS_TARGET):
        sens = await binary_sensor.new_binary_sensor(has_target_config)
        cg.add(var.set_has_target_binary_sensor(sens))

    if last_cmd_config := config.get(CONF_LAST_CMD_OK):
        sens = await binary_sensor.new_binary_sensor(last_cmd_config)
        cg.add(var.set_last_cmd_ok_binary_sensor(sens))

    if distance_config := config.get(CONF_DISTANCE):
        sens = await sensor.new_sensor(distance_config)
        cg.add(var.set_distance_sensor(sens))

    if max_gate_config := config.get(CONF_MAX_GATE):
        num = await number.new_number(max_gate_config, min_value=1, max_value=16, step=1)
        await cg.register_component(num, max_gate_config)
        cg.add(num.set_parent(var))
        cg.add(num.set_role(0))
        cg.add(var.set_max_gate_number(num))

    if min_gate_config := config.get(CONF_MIN_GATE):
        num = await number.new_number(min_gate_config, min_value=0, max_value=16, step=1)
        await cg.register_component(num, min_gate_config)
        cg.add(num.set_parent(var))
        cg.add(num.set_role(1))
        cg.add(var.set_min_gate_number(num))

    if none_dur_config := config.get(CONF_NONE_DURATION):
        num = await number.new_number(none_dur_config, min_value=10, max_value=120, step=1)
        await cg.register_component(num, none_dur_config)
        cg.add(num.set_parent(var))
        cg.add(num.set_role(2))
        cg.add(var.set_none_duration_number(num))

    for gate_cfg in config.get(CONF_GATE_ENERGY, []):
        idx = gate_cfg["index"]
        sens = await sensor.new_sensor(gate_cfg)
        cg.add(var.set_gate_energy_sensor(idx, sens))
