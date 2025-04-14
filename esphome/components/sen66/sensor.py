from esphome import automation
from esphome.automation import maybe_simple_id
import esphome.codegen as cg
from esphome.components import i2c, sensirion_common, sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_CO2,
    CONF_HUMIDITY,
    CONF_ID,
    CONF_OFFSET,
    CONF_PM_1_0,
    CONF_PM_2_5,
    CONF_PM_4_0,
    CONF_PM_10_0,
    CONF_TEMPERATURE,
    DEVICE_CLASS_AQI,
    DEVICE_CLASS_CARBON_DIOXIDE,
    DEVICE_CLASS_HUMIDITY,
    DEVICE_CLASS_PM1,
    DEVICE_CLASS_PM10,
    DEVICE_CLASS_PM25,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLATILE_ORGANIC_COMPOUNDS,
    ICON_CHEMICAL_WEAPON,
    ICON_COUNTER,
    ICON_MOLECULE_CO2,
    ICON_THERMOMETER,
    ICON_WATER_PERCENT,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_COUNTS_PER_CUBIC_CENTIMETER,
    UNIT_MICROGRAMS_PER_CUBIC_METER,
    UNIT_PARTS_PER_MILLION,
    UNIT_PERCENT,
)

CODEOWNERS = ["@labonnepuree"]
DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["sensirion_common"]

sen66_ns = cg.esphome_ns.namespace("sen66")
SEN66Component = sen66_ns.class_(
    "SEN66Component", cg.PollingComponent, sensirion_common.SensirionI2CDevice
)

# --- Configuration Keys ---
CONF_AMBIENT_PRESSURE_HPA = "ambient_pressure"
CONF_SENSOR_ALTITUDE_M = "sensor_altitude"
CONF_ALGORITHM_TUNING = (
    "algorithm_tuning"  # Generic key used under VOC/NOx sensor config
)
CONF_CO2_AUTOMATIC_SELF_CALIBRATION = "co2_automatic_self_calibration"
CONF_GAIN_FACTOR = "gain_factor"
CONF_GATING_MAX_DURATION_MINUTES = "gating_max_duration_minutes"
CONF_INDEX_OFFSET = "index_offset"
CONF_K = "k"
CONF_LEARNING_TIME_GAIN_HOURS = "learning_time_gain_hours"
CONF_LEARNING_TIME_OFFSET_HOURS = "learning_time_offset_hours"
CONF_MAX_ERRORS_BEFORE_REBOOT = "max_errors_before_reboot"
CONF_NORMALIZED_OFFSET_SLOPE = "normalized_offset_slope"
CONF_NOX = "nox"  # Sensor key
CONF_P = "p"
CONF_SLOT = "slot"
CONF_STD_INITIAL = "std_initial"
CONF_T1 = "t1"
CONF_T2 = "t2"
CONF_TEMPERATURE_ACCELERATION = "temperature_acceleration"
CONF_TEMPERATURE_COMPENSATION = "temperature_compensation"
CONF_TIME_CONSTANT = "time_constant"
CONF_TARGET_CO2_CONCENTRATION = "target_co2_concentration"  # For FRC service
CONF_VOC = "voc"  # Sensor key

# Number Concentration sensor keys
CONF_NC_0_5 = "number_concentration_0_5"
CONF_NC_1_0 = "number_concentration_1_0"
CONF_NC_2_5 = "number_concentration_2_5"
CONF_NC_4_0 = "number_concentration_4_0"
CONF_NC_10_0 = "number_concentration_10_0"

# --- Actions & Services ---
StartFanAction = sen66_ns.class_("StartFanAction", automation.Action)
ActivateShtHeaterAction = sen66_ns.class_("ActivateShtHeaterAction", automation.Action)
PerformForcedCo2RecalibrationAction = sen66_ns.class_(
    "PerformForcedCo2RecalibrationAction", automation.Action
)

# --- Schemas ---

# Gas Tuning Schema (used for both VOC and NOx sensor configs)
GAS_SENSOR_TUNING = cv.Schema(
    {
        cv.Optional(CONF_INDEX_OFFSET): cv.int_range(1, 250),
        cv.Optional(CONF_LEARNING_TIME_OFFSET_HOURS): cv.int_range(1, 1000),
        cv.Optional(CONF_LEARNING_TIME_GAIN_HOURS): cv.int_range(1, 1000),
        cv.Optional(CONF_GATING_MAX_DURATION_MINUTES): cv.int_range(0, 3000),
        cv.Optional(CONF_STD_INITIAL): cv.int_range(10, 5000),
        cv.Optional(CONF_GAIN_FACTOR): cv.int_range(1, 1000),
    }
)

# Schema for the VOC sensor allowing tuning parameters
VOC_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=None,  # No unit
    icon=ICON_CHEMICAL_WEAPON,
    accuracy_decimals=1,  # Default accuracy
    device_class=DEVICE_CLASS_VOLATILE_ORGANIC_COMPOUNDS,
    state_class=STATE_CLASS_MEASUREMENT,
).extend(
    {cv.Optional(CONF_ALGORITHM_TUNING): GAS_SENSOR_TUNING}
)  # Only extend for tuning

# Schema for the NOx sensor allowing tuning parameters
NOX_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=None,  # No unit
    icon=ICON_CHEMICAL_WEAPON,
    accuracy_decimals=1,  # Default accuracy
    device_class=DEVICE_CLASS_AQI,
    state_class=STATE_CLASS_MEASUREMENT,
).extend(
    {cv.Optional(CONF_ALGORITHM_TUNING): GAS_SENSOR_TUNING}
)  # Only extend for tuning

# Main Configuration Schema
CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SEN66Component),  # Main component ID
            # Sensor Configuration (Define directly under the component key)
            cv.Optional(CONF_PM_1_0): sensor.sensor_schema(
                unit_of_measurement=UNIT_MICROGRAMS_PER_CUBIC_METER,
                icon=ICON_CHEMICAL_WEAPON,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_PM1,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_PM_2_5): sensor.sensor_schema(
                unit_of_measurement=UNIT_MICROGRAMS_PER_CUBIC_METER,
                icon=ICON_CHEMICAL_WEAPON,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_PM25,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_PM_4_0): sensor.sensor_schema(
                unit_of_measurement=UNIT_MICROGRAMS_PER_CUBIC_METER,
                icon=ICON_CHEMICAL_WEAPON,
                accuracy_decimals=1,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_PM_10_0): sensor.sensor_schema(
                unit_of_measurement=UNIT_MICROGRAMS_PER_CUBIC_METER,
                icon=ICON_CHEMICAL_WEAPON,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_PM10,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_NC_0_5): sensor.sensor_schema(
                unit_of_measurement=UNIT_COUNTS_PER_CUBIC_CENTIMETER,
                icon=ICON_COUNTER,
                accuracy_decimals=1,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_NC_1_0): sensor.sensor_schema(
                unit_of_measurement=UNIT_COUNTS_PER_CUBIC_CENTIMETER,
                icon=ICON_COUNTER,
                accuracy_decimals=1,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_NC_2_5): sensor.sensor_schema(
                unit_of_measurement=UNIT_COUNTS_PER_CUBIC_CENTIMETER,
                icon=ICON_COUNTER,
                accuracy_decimals=1,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_NC_4_0): sensor.sensor_schema(
                unit_of_measurement=UNIT_COUNTS_PER_CUBIC_CENTIMETER,
                icon=ICON_COUNTER,
                accuracy_decimals=1,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_NC_10_0): sensor.sensor_schema(
                unit_of_measurement=UNIT_COUNTS_PER_CUBIC_CENTIMETER,
                icon=ICON_COUNTER,
                accuracy_decimals=1,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                icon=ICON_THERMOMETER,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_HUMIDITY): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                icon=ICON_WATER_PERCENT,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_HUMIDITY,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_VOC): VOC_SENSOR_SCHEMA,  # Use extended schema
            cv.Optional(CONF_NOX): NOX_SENSOR_SCHEMA,  # Use extended schema
            cv.Optional(CONF_CO2): sensor.sensor_schema(
                unit_of_measurement=UNIT_PARTS_PER_MILLION,
                icon=ICON_MOLECULE_CO2,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_CARBON_DIOXIDE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            # Component-Wide Configuration Settings
            cv.Optional(CONF_TEMPERATURE_COMPENSATION): cv.Schema(
                {
                    cv.Optional(CONF_OFFSET, default=0.0): cv.float_,
                    cv.Optional(CONF_NORMALIZED_OFFSET_SLOPE, default=0.0): cv.float_,
                    cv.Optional(CONF_TIME_CONSTANT, default=0): cv.positive_int,
                    cv.Optional(CONF_SLOT, default=0): cv.int_range(0, 4),
                }
            ),
            cv.Optional(CONF_TEMPERATURE_ACCELERATION): cv.Schema(
                {
                    # Scale factors are handled internally by sensor
                    cv.Required(CONF_K): cv.positive_int,
                    cv.Required(CONF_P): cv.positive_int,
                    cv.Required(CONF_T1): cv.positive_int,
                    cv.Required(CONF_T2): cv.positive_int,
                }
            ),
            cv.Optional(CONF_CO2_AUTOMATIC_SELF_CALIBRATION): cv.boolean,
            cv.Optional(CONF_AMBIENT_PRESSURE_HPA): cv.int_range(700, 1200),
            cv.Optional(CONF_SENSOR_ALTITUDE_M): cv.int_range(0, 3000),
            cv.Optional(CONF_MAX_ERRORS_BEFORE_REBOOT, default=10): cv.positive_int,
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x69))
)

# Mapping from config key to C++ Sensor Setter Method
SENSOR_SETTERS = {
    CONF_PM_1_0: "set_pm_1_0_sensor",
    CONF_PM_2_5: "set_pm_2_5_sensor",
    CONF_PM_4_0: "set_pm_4_0_sensor",
    CONF_PM_10_0: "set_pm_10_0_sensor",
    CONF_NC_0_5: "set_nc_0_5_sensor",
    CONF_NC_1_0: "set_nc_1_0_sensor",
    CONF_NC_2_5: "set_nc_2_5_sensor",
    CONF_NC_4_0: "set_nc_4_0_sensor",
    CONF_NC_10_0: "set_nc_10_0_sensor",
    CONF_TEMPERATURE: "set_temperature_sensor",
    CONF_HUMIDITY: "set_humidity_sensor",
    CONF_VOC: "set_voc_sensor",
    CONF_NOX: "set_nox_sensor",
    CONF_CO2: "set_co2_sensor",
}


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    # Register configured sensors and handle nested tuning
    for key, func_name in SENSOR_SETTERS.items():
        if key in config:
            sensor_config = config[key]
            sens = await sensor.new_sensor(
                sensor_config
            )  # Pass the whole sensor block config
            cg.add(getattr(var, func_name)(sens))

            # Handle nested algorithm tuning for VOC
            if key == CONF_VOC and CONF_ALGORITHM_TUNING in sensor_config:
                cfg = sensor_config[CONF_ALGORITHM_TUNING]
                cg.add(
                    var.set_voc_algorithm_tuning(
                        cfg.get(CONF_INDEX_OFFSET, 100),
                        cfg.get(CONF_LEARNING_TIME_OFFSET_HOURS, 12),
                        cfg.get(CONF_LEARNING_TIME_GAIN_HOURS, 12),
                        cfg.get(CONF_GATING_MAX_DURATION_MINUTES, 180),
                        cfg.get(CONF_STD_INITIAL, 50),
                        cfg.get(CONF_GAIN_FACTOR, 230),
                    )
                )
            # Handle nested algorithm tuning for NOx
            elif key == CONF_NOX and CONF_ALGORITHM_TUNING in sensor_config:
                cfg = sensor_config[CONF_ALGORITHM_TUNING]
                gating = cfg.get(CONF_GATING_MAX_DURATION_MINUTES, 720)
                if gating == 180:  # Check if still default VOC value
                    print(
                        f"Note: Overriding {CONF_GATING_MAX_DURATION_MINUTES} to 720 for NOx tuning."
                    )
                    gating = 720
                cg.add(
                    var.set_nox_algorithm_tuning(
                        cfg.get(CONF_INDEX_OFFSET, 1),
                        cfg.get(CONF_LEARNING_TIME_OFFSET_HOURS, 12),
                        cfg.get(CONF_LEARNING_TIME_GAIN_HOURS, 12),
                        gating,
                        cfg.get(CONF_STD_INITIAL, 50),
                        cfg.get(CONF_GAIN_FACTOR, 230),
                    )
                )

    # Handle component-wide settings
    if CONF_TEMPERATURE_COMPENSATION in config:
        cfg = config[CONF_TEMPERATURE_COMPENSATION]
        cg.add(
            var.set_temperature_compensation(
                cfg[CONF_OFFSET],
                cfg[CONF_NORMALIZED_OFFSET_SLOPE],
                cfg[CONF_TIME_CONSTANT],
                cfg[CONF_SLOT],  # Pass slot
            )
        )

    # Handle temperature acceleration
    if CONF_TEMPERATURE_ACCELERATION in config:
        cfg = config[CONF_TEMPERATURE_ACCELERATION]
        cg.add(
            var.set_temperature_acceleration_parameters(
                cfg[CONF_K], cfg[CONF_P], cfg[CONF_T1], cfg[CONF_T2]
            )
        )

    # Handle CO2 ASC (pass value only if explicitly set, otherwise C++ uses default)
    if CONF_CO2_AUTOMATIC_SELF_CALIBRATION in config:
        cg.add(
            var.set_co2_automatic_self_calibration(
                config[CONF_CO2_AUTOMATIC_SELF_CALIBRATION]
            )
        )

    # Handle Pressure/Altitude
    if CONF_AMBIENT_PRESSURE_HPA in config:
        cg.add(var.set_ambient_pressure(config[CONF_AMBIENT_PRESSURE_HPA]))

    if CONF_SENSOR_ALTITUDE_M in config:
        cg.add(var.set_sensor_altitude(config[CONF_SENSOR_ALTITUDE_M]))

    if max_errors := config.get(CONF_MAX_ERRORS_BEFORE_REBOOT):
        cg.add(var.set_max_consecutive_failures(max_errors))


# --- Action Registrations ---
SEN66_ACTION_BASE_SCHEMA = maybe_simple_id(
    {
        cv.Required(CONF_ID): cv.use_id(SEN66Component),
    }
)


@automation.register_action(
    "sen66.start_fan_cleaning", StartFanAction, SEN66_ACTION_BASE_SCHEMA
)
async def sen66_fan_clean_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_action(
    "sen66.activate_sht_heater", ActivateShtHeaterAction, SEN66_ACTION_BASE_SCHEMA
)
async def sen66_heater_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


# --- Service/Action Registration for FRC ---
SEN66_FRC_ACTION_SCHEMA = cv.Schema(  # Schema for the action arguments
    {
        cv.Required(CONF_ID): cv.use_id(SEN66Component),  # Need component ID here
        cv.Required(CONF_TARGET_CO2_CONCENTRATION): cv.templatable(cv.positive_int),
    }
)


@automation.register_action(
    "sen66.perform_forced_co2_recalibration",
    PerformForcedCo2RecalibrationAction,
    SEN66_FRC_ACTION_SCHEMA,
)
async def sen66_frc_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])  # Get component Pvariable
    var = cg.new_Pvariable(action_id, template_arg, paren)  # Create action Pvariable
    # Get the target concentration argument from the template args
    template_ = await cg.templatable(
        config[CONF_TARGET_CO2_CONCENTRATION], args, cg.uint16
    )
    cg.add(
        var.set_target_co2(template_)
    )  # Set the target_co2 member of the C++ action object
    return var
