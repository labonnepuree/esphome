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

# Component metadata: Information for ESPHome frontend and library management
CODEOWNERS = ["@labonnepuree"]
DEPENDENCIES = ["i2c"]  # Requires the I2C bus component
AUTO_LOAD = ["sensirion_common"]  # Automatically loads the Sensirion helper library

# Define the C++ namespace and component class for code generation
sen66_ns = cg.esphome_ns.namespace("sen66")
# Define SEN66Component type, inheriting from PollingComponent and SensirionI2CDevice
SEN66Component = sen66_ns.class_(
    "SEN66Component", cg.PollingComponent, sensirion_common.SensirionI2CDevice
)

# --- Configuration Keys (Python constants for YAML keys) ---
# These constants ensure consistency and prevent typos when referring to YAML keys.

# General configuration keys
CONF_AMBIENT_PRESSURE_HPA = (
    "ambient_pressure"  # Optional ambient pressure for CO2 comp.
)
CONF_SENSOR_ALTITUDE_M = "sensor_altitude"  # Optional sensor altitude for CO2 comp.
CONF_ALGORITHM_TUNING = "algorithm_tuning"  # Key for VOC/NOx algorithm tuning block
CONF_CO2_AUTOMATIC_SELF_CALIBRATION = (
    "co2_automatic_self_calibration"  # Enable/disable CO2 ASC
)
CONF_GAIN_FACTOR = "gain_factor"  # VOC/NOx algorithm tuning parameter
CONF_GATING_MAX_DURATION_MINUTES = (
    "gating_max_duration_minutes"  # VOC/NOx algorithm tuning parameter
)
CONF_INDEX_OFFSET = "index_offset"  # VOC/NOx algorithm tuning parameter
CONF_K = "k"  # Temperature acceleration parameter
CONF_LEARNING_TIME_GAIN_HOURS = (
    "learning_time_gain_hours"  # VOC/NOx algorithm tuning parameter
)
CONF_LEARNING_TIME_OFFSET_HOURS = (
    "learning_time_offset_hours"  # VOC/NOx algorithm tuning parameter
)
CONF_MAX_ERRORS_BEFORE_REBOOT = (
    "max_errors_before_reboot"  # Max communication errors before rebooting ESP
)
CONF_NORMALIZED_OFFSET_SLOPE = (
    "normalized_offset_slope"  # Temperature compensation parameter
)
CONF_NOX = "nox"  # NOx sensor key
CONF_P = "p"  # Temperature acceleration parameter
CONF_SLOT = "slot"  # Temperature compensation slot
CONF_STD_INITIAL = "std_initial"  # VOC/NOx algorithm tuning parameter
CONF_T1 = "t1"  # Temperature acceleration parameter
CONF_T2 = "t2"  # Temperature acceleration parameter
CONF_TEMPERATURE_ACCELERATION = (
    "temperature_acceleration"  # Key for temperature acceleration config
)
CONF_TEMPERATURE_COMPENSATION = (
    "temperature_compensation"  # Key for temperature compensation config
)
CONF_TIME_CONSTANT = "time_constant"  # Temperature compensation parameter
CONF_TARGET_CO2_CONCENTRATION = (
    "target_co2_concentration"  # Target concentration for FRC action
)
CONF_VOC = "voc"  # VOC sensor key

# Number Concentration (NC) sensor keys
CONF_NC_0_5 = "number_concentration_0_5"
CONF_NC_1_0 = "number_concentration_1_0"
CONF_NC_2_5 = "number_concentration_2_5"
CONF_NC_4_0 = "number_concentration_4_0"
CONF_NC_10_0 = "number_concentration_10_0"

# --- Actions & Services ---
# Define Python representations of the C++ automation action classes.
# These are used by the action registration system.
StartFanAction = sen66_ns.class_("StartFanAction", automation.Action)
ActivateShtHeaterAction = sen66_ns.class_("ActivateShtHeaterAction", automation.Action)
PerformForcedCo2RecalibrationAction = sen66_ns.class_(
    "PerformForcedCo2RecalibrationAction", automation.Action
)
FactoryResetAction = sen66_ns.class_("FactoryResetAction", automation.Action)

# --- Schemas (Define and validate the YAML configuration structure) ---

# Schema for VOC/NOx algorithm tuning parameters.
# Ensures that the nested tuning parameters are within valid ranges.
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

# Base schema for the VOC sensor, extended to allow optional algorithm tuning.
VOC_SENSOR_SCHEMA = sensor.sensor_schema(
    # Standard sensor properties
    icon=ICON_CHEMICAL_WEAPON,
    accuracy_decimals=1,  # Default accuracy
    device_class=DEVICE_CLASS_VOLATILE_ORGANIC_COMPOUNDS,
    state_class=STATE_CLASS_MEASUREMENT,
).extend(
    # Add optional tuning block
    {cv.Optional(CONF_ALGORITHM_TUNING): GAS_SENSOR_TUNING}
)

# Base schema for the NOx sensor, extended to allow optional algorithm tuning.
NOX_SENSOR_SCHEMA = sensor.sensor_schema(
    # Standard sensor properties
    icon=ICON_CHEMICAL_WEAPON,
    accuracy_decimals=1,  # Default accuracy
    device_class=DEVICE_CLASS_AQI,
    state_class=STATE_CLASS_MEASUREMENT,
).extend(
    # Add optional tuning block
    {cv.Optional(CONF_ALGORITHM_TUNING): GAS_SENSOR_TUNING}
)

# Main Configuration Schema for the SEN66 component.
# Validates the top-level 'sen66:' YAML block.
CONFIG_SCHEMA = (
    cv.Schema(
        {
            # Declare the ID for this component instance
            cv.GenerateID(): cv.declare_id(SEN66Component),
            # Sensor Configurations: Optional blocks for each supported sensor type.
            # Uses standard sensor.sensor_schema or the extended VOC/NOX schemas.
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
            cv.Optional(CONF_VOC): VOC_SENSOR_SCHEMA,  # Use schema with tuning
            cv.Optional(CONF_NOX): NOX_SENSOR_SCHEMA,  # Use schema with tuning
            cv.Optional(CONF_CO2): sensor.sensor_schema(
                unit_of_measurement=UNIT_PARTS_PER_MILLION,
                icon=ICON_MOLECULE_CO2,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_CARBON_DIOXIDE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            # Component-Wide Configuration Settings: Optional blocks for global settings.
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
                    # Scale factors are handled internally by the C++ code
                    cv.Required(CONF_K): cv.float_,
                    cv.Required(CONF_P): cv.float_,
                    cv.Required(CONF_T1): cv.float_,
                    cv.Required(CONF_T2): cv.float_,
                }
            ),
            cv.Optional(CONF_CO2_AUTOMATIC_SELF_CALIBRATION): cv.boolean,
            cv.Optional(CONF_AMBIENT_PRESSURE_HPA): cv.int_range(700, 1200),
            cv.Optional(CONF_SENSOR_ALTITUDE_M): cv.int_range(0, 3000),
            cv.Optional(CONF_MAX_ERRORS_BEFORE_REBOOT, default=10): cv.positive_int,
        }
    )
    # Inherit standard polling component settings (update_interval)
    .extend(cv.polling_component_schema("60s"))
    # Inherit standard I2C device settings (address, bus_id)
    .extend(i2c.i2c_device_schema(0x6B))  # Default I2C address 0x6b
)

# Mapping from YAML config key to the C++ Sensor Setter Method name in SEN66Component.
# Used in to_code to dynamically call the correct C++ method.
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
    """Generate the C++ code for the SEN66 component based on YAML config.

    This function is called by ESPHome during compilation. It takes the validated
    YAML configuration (`config`) and generates C++ code to instantiate and
    configure the SEN66Component object.

    Args:
        config: The validated configuration dictionary for this component.
    """
    # Create the main C++ component object variable (Pvariable)
    var = cg.new_Pvariable(config[CONF_ID])

    # Register the component with ESPHome core (for setup, loop, etc.)
    await cg.register_component(var, config)
    # Register the component as an I2C device
    await i2c.register_i2c_device(var, config)

    # --- Sensor Setup Loop ---
    # Iterate through all possible sensors defined in SENSOR_SETTERS.
    for key, func_name in SENSOR_SETTERS.items():
        # Check if this sensor is configured in the user's YAML
        if key in config:
            sensor_config = config[key]  # Get the config block for this sensor
            # Create a new C++ sensor::Sensor object using the standard helper
            sens = await sensor.new_sensor(sensor_config)
            # Call the corresponding C++ setter method on the SEN66Component instance
            # e.g., cg.add(var.set_pm_2_5_sensor(sens))
            cg.add(getattr(var, func_name)(sens))

            # --- Nested Algorithm Tuning Setup ---
            # Special handling for VOC sensor tuning parameters
            if key == CONF_VOC and CONF_ALGORITHM_TUNING in sensor_config:
                tuning_cfg = sensor_config[CONF_ALGORITHM_TUNING]
                # Call the C++ method to set VOC tuning parameters.
                # Uses .get() with defaults, so if a parameter is missing in YAML,
                # the default value specified here is used.
                cg.add(
                    var.set_voc_algorithm_tuning(
                        tuning_cfg.get(CONF_INDEX_OFFSET, 100),
                        tuning_cfg.get(CONF_LEARNING_TIME_OFFSET_HOURS, 12),
                        tuning_cfg.get(CONF_LEARNING_TIME_GAIN_HOURS, 12),
                        tuning_cfg.get(CONF_GATING_MAX_DURATION_MINUTES, 180),
                        tuning_cfg.get(CONF_STD_INITIAL, 50),
                        tuning_cfg.get(CONF_GAIN_FACTOR, 230),
                    )
                )
            # Special handling for NOx sensor tuning parameters
            elif key == CONF_NOX and CONF_ALGORITHM_TUNING in sensor_config:
                tuning_cfg = sensor_config[CONF_ALGORITHM_TUNING]
                # Call the C++ method to set NOx tuning parameters with defaults.
                cg.add(
                    var.set_nox_algorithm_tuning(
                        tuning_cfg.get(CONF_INDEX_OFFSET, 1),
                        tuning_cfg.get(CONF_LEARNING_TIME_OFFSET_HOURS, 12),
                        tuning_cfg.get(CONF_LEARNING_TIME_GAIN_HOURS, 12),
                        tuning_cfg.get(CONF_GATING_MAX_DURATION_MINUTES, 720),
                        tuning_cfg.get(CONF_STD_INITIAL, 50),
                        tuning_cfg.get(CONF_GAIN_FACTOR, 230),
                    )
                )

    # --- Component-Wide Settings Setup ---
    # Configure temperature compensation if the block is present in YAML
    if CONF_TEMPERATURE_COMPENSATION in config:
        cfg = config[CONF_TEMPERATURE_COMPENSATION]
        cg.add(
            var.set_temperature_compensation(
                cfg[CONF_OFFSET],
                cfg[CONF_NORMALIZED_OFFSET_SLOPE],
                cfg[CONF_TIME_CONSTANT],
                cfg[CONF_SLOT],  # Pass the specified slot
            )
        )

    # Configure temperature acceleration parameters if the block is present
    if CONF_TEMPERATURE_ACCELERATION in config:
        cfg = config[CONF_TEMPERATURE_ACCELERATION]
        cg.add(
            var.set_temperature_acceleration_parameters(
                cfg[CONF_K], cfg[CONF_P], cfg[CONF_T1], cfg[CONF_T2]
            )
        )

    # Configure CO2 Automatic Self-Calibration (ASC)
    # Only call the C++ setter if the key is explicitly in the config.
    # The C++ code handles the default behavior if not set.
    if CONF_CO2_AUTOMATIC_SELF_CALIBRATION in config:
        cg.add(
            var.set_co2_automatic_self_calibration(
                config[CONF_CO2_AUTOMATIC_SELF_CALIBRATION]
            )
        )

    # Configure Ambient Pressure Compensation for CO2
    if CONF_AMBIENT_PRESSURE_HPA in config:
        cg.add(var.set_ambient_pressure(config[CONF_AMBIENT_PRESSURE_HPA]))

    # Configure Sensor Altitude Compensation for CO2
    if CONF_SENSOR_ALTITUDE_M in config:
        cg.add(var.set_sensor_altitude(config[CONF_SENSOR_ALTITUDE_M]))

    # Configure max consecutive communication errors before triggering reboot
    if max_errors := config.get(CONF_MAX_ERRORS_BEFORE_REBOOT):
        cg.add(var.set_max_consecutive_failures(max_errors))


# --- Action Registrations (Link YAML actions to C++ classes) ---

# Base schema for simple actions requiring only the SEN66 component ID.
SEN66_ACTION_BASE_SCHEMA = maybe_simple_id(
    {
        cv.Required(CONF_ID): cv.use_id(
            SEN66Component
        ),  # Requires the ID of the SEN66 component
    }
)


# Register the 'sen66.start_fan_cleaning' action.
@automation.register_action(
    "sen66.start_fan_cleaning", StartFanAction, SEN66_ACTION_BASE_SCHEMA
)
async def sen66_fan_clean_to_code(config, action_id, template_arg, args):
    """Generate C++ code for the 'start_fan_cleaning' action.

    Args:
        config: The action's configuration dictionary (contains the component ID).
        action_id: The unique ID for this specific action instance.
        template_arg: Template arguments for the action class (usually empty).
        args: Arguments passed to the action (usually empty for base schema actions).

    Returns:
        A Pvariable representing the instantiated C++ StartFanAction object.
    """
    # Get the Pvariable for the parent SEN66Component instance
    paren = await cg.get_variable(config[CONF_ID])
    # Create a new C++ StartFanAction object, passing the parent component
    return cg.new_Pvariable(action_id, template_arg, paren)


# Register the 'sen66.activate_sht_heater' action.
@automation.register_action(
    "sen66.activate_sht_heater", ActivateShtHeaterAction, SEN66_ACTION_BASE_SCHEMA
)
async def sen66_heater_to_code(config, action_id, template_arg, args):
    """Generate C++ code for the 'activate_sht_heater' action.

    Args: See sen66_fan_clean_to_code.
    Returns: A Pvariable representing the instantiated C++ ActivateShtHeaterAction object.
    """
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


# --- Service/Action Registration for Forced CO2 Recalibration (FRC) ---

# Schema for the FRC action, requiring component ID and target CO2 concentration.
SEN66_FRC_ACTION_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ID): cv.use_id(SEN66Component),
        # Target CO2 concentration is required and can be templated.
        cv.Required(CONF_TARGET_CO2_CONCENTRATION): cv.templatable(cv.positive_int),
    }
)


# Register the 'sen66.perform_forced_co2_recalibration' action.
@automation.register_action(
    "sen66.perform_forced_co2_recalibration",
    PerformForcedCo2RecalibrationAction,
    SEN66_FRC_ACTION_SCHEMA,
)
async def sen66_frc_to_code(config, action_id, template_arg, args):
    """Generate C++ code for the 'perform_forced_co2_recalibration' action.

    This action requires an additional parameter (target CO2 concentration).

    Args:
        config: The action's config (contains ID and target CO2 template).
        action_id: Unique ID for the action instance.
        template_arg: Template arguments for the action class.
        args: Arguments passed to the action (used for template evaluation).

    Returns:
        A Pvariable representing the instantiated C++ PerformForcedCo2RecalibrationAction object.
    """
    # Get the parent SEN66Component Pvariable
    paren = await cg.get_variable(config[CONF_ID])
    # Create the C++ PerformForcedCo2RecalibrationAction Pvariable
    var = cg.new_Pvariable(action_id, template_arg, paren)

    # Evaluate the template for the target CO2 concentration
    template_ = await cg.templatable(
        config[CONF_TARGET_CO2_CONCENTRATION], args, cg.uint16
    )
    # Call the C++ action object's setter method to store the target CO2 value
    cg.add(var.set_target_co2(template_))
    return var


# --- Action Registration for Factory Reset ---


# Register the 'sen66.factory_reset' action.
@automation.register_action(
    "sen66.factory_reset", FactoryResetAction, SEN66_ACTION_BASE_SCHEMA
)
async def sen66_factory_reset_to_code(config, action_id, template_arg, args):
    """Generate C++ code for the 'factory_reset' action.

    Args: See sen66_fan_clean_to_code.
    Returns: A Pvariable representing the instantiated C++ FactoryResetAction object.
    """
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)
