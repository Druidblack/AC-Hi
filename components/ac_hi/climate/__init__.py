# SPDX-License-Identifier: MIT
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, uart, sensor
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_UPDATE_INTERVAL,
    DEVICE_CLASS_TEMPERATURE,
    UNIT_CELSIUS,
    ICON_THERMOMETER,
)

AUTO_LOAD = ["uart", "climate", "sensor"]
CODEOWNERS = ["@artshevchenko"]

ac_hi_ns = cg.esphome_ns.namespace("ac_hi")
ACHiClimate = ac_hi_ns.class_("ACHiClimate", climate.Climate, cg.Component, uart.UARTDevice)

CONF_SENSORS = "sensors"
CONF_T_SET = "temperature_set"
CONF_T_CUR = "temperature_current"
CONF_T_OUT = "temperature_outdoor"
CONF_T_PIPE = "temperature_pipe"
CONF_COMP_FREQ = "compressor_frequency"

SENSOR_SCHEMA = cv.Schema({
    cv.Optional(CONF_T_SET): sensor.sensor_schema(unit_of_measurement=UNIT_CELSIUS, accuracy_decimals=0, device_class=DEVICE_CLASS_TEMPERATURE, icon=ICON_THERMOMETER),
    cv.Optional(CONF_T_CUR): sensor.sensor_schema(unit_of_measurement=UNIT_CELSIUS, accuracy_decimals=0, device_class=DEVICE_CLASS_TEMPERATURE, icon=ICON_THERMOMETER),
    cv.Optional(CONF_T_OUT): sensor.sensor_schema(unit_of_measurement=UNIT_CELSIUS, accuracy_decimals=0, device_class=DEVICE_CLASS_TEMPERATURE, icon=ICON_THERMOMETER),
    cv.Optional(CONF_T_PIPE): sensor.sensor_schema(unit_of_measurement=UNIT_CELSIUS, accuracy_decimals=0, device_class=DEVICE_CLASS_TEMPERATURE, icon=ICON_THERMOMETER),
    cv.Optional(CONF_COMP_FREQ): sensor.sensor_schema(accuracy_decimals=0),
})

CONFIG_SCHEMA = (
    climate.CLIMATE_SCHEMA.extend(
        {
            cv.GenerateID(CONF_ID): cv.declare_id(ACHiClimate),
            cv.Optional(CONF_NAME, default="Hisense/Ballu AC"): cv.string,
            cv.Optional(CONF_UPDATE_INTERVAL, default="2s"): cv.update_interval,
            cv.Optional(CONF_SENSORS, default={}): SENSOR_SCHEMA,
        }
    ).extend(uart.UART_DEVICE_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await climate.register_climate(var, config)
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))

    sens = config.get(CONF_SENSORS, {})
    if t := sens.get(CONF_T_SET):
        s = await sensor.new_sensor(t); cg.add(var.set_tset_sensor(s))
    if t := sens.get(CONF_T_CUR):
        s = await sensor.new_sensor(t); cg.add(var.set_tcur_sensor(s))
    if t := sens.get(CONF_T_OUT):
        s = await sensor.new_sensor(t); cg.add(var.set_tout_sensor(s))
    if t := sens.get(CONF_T_PIPE):
        s = await sensor.new_sensor(t); cg.add(var.set_tpipe_sensor(s))
    if t := sens.get(CONF_COMP_FREQ):
        s = await sensor.new_sensor(t); cg.add(var.set_compfreq_sensor(s))
