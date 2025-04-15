#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/sensirion_common/i2c_sensirion.h"
#include "esphome/core/application.h"
#include "esphome/core/preferences.h"

namespace esphome {
/**
 * @brief Namespace for the SEN66 sensor component.
 */
namespace sen66 {

/**
 * @brief Defines the operational states of the SEN66 component.
 *
 * Used internally to manage transitions between measuring, performing actions (cleaning, heating, FRC),
 * and initialization states, preventing conflicting operations.
 */
enum ComponentState {
  IDLE,                           ///< Component initialized, sensor potentially stopped, not polling.
  MEASURING,                      ///< Component actively polling and reading measurements.
  WAITING_FOR_CLEANING,           ///< Fan cleaning command sent, waiting for completion (~10s). Polling paused.
  WAITING_FOR_HEATER,             ///< Heater activation command sent, waiting for completion (~20s). Polling paused.
  WAITING_FOR_RECALIBRATION_CMD,  ///< FRC initiated, command sent, waiting for sensor processing time (~3min) before
                                  ///< reading result. Polling paused.
  // Note: Waiting for FRC result reading is handled within handle_frc_read_result_() triggered by timeout.
};

/** @brief I2C Command IDs for the SEN66 sensor. Sourced from Sensirion documentation/headers. */
typedef enum : uint16_t {
  SEN66_START_CONTINUOUS_MEASUREMENT_CMD_ID = 0x21,
  SEN66_STOP_MEASUREMENT_CMD_ID = 0x104,
  SEN66_GET_DATA_READY_CMD_ID = 0x202,
  SEN66_READ_MEASURED_VALUES_AS_INTEGERS_CMD_ID = 0x300,
  SEN66_READ_NUMBER_CONCENTRATION_VALUES_AS_INTEGERS_CMD_ID = 0x316,
  SEN66_READ_MEASURED_RAW_VALUES_CMD_ID = 0x405,
  SEN66_START_FAN_CLEANING_CMD_ID = 0x5607,
  SEN66_SET_TEMPERATURE_OFFSET_PARAMETERS_CMD_ID = 0x60b2,
  SEN66_SET_VOC_ALGORITHM_TUNING_PARAMETERS_CMD_ID = 0x60d0,
  SEN66_GET_VOC_ALGORITHM_TUNING_PARAMETERS_CMD_ID = 0x60d0,  // Same command ID for get/set
  SEN66_SET_NOX_ALGORITHM_TUNING_PARAMETERS_CMD_ID = 0x60e1,
  SEN66_GET_NOX_ALGORITHM_TUNING_PARAMETERS_CMD_ID = 0x60e1,  // Same command ID for get/set
  SEN66_SET_TEMPERATURE_ACCELERATION_PARAMETERS_CMD_ID = 0x6100,
  SEN66_SET_VOC_ALGORITHM_STATE_CMD_ID = 0x6181,
  SEN66_GET_VOC_ALGORITHM_STATE_CMD_ID = 0x6181,  // Same command ID for get/set
  SEN66_PERFORM_FORCED_CO2_RECALIBRATION_CMD_ID = 0x6707,
  SEN66_SET_CO2_SENSOR_AUTOMATIC_SELF_CALIBRATION_CMD_ID = 0x6711,
  SEN66_GET_CO2_SENSOR_AUTOMATIC_SELF_CALIBRATION_CMD_ID = 0x6711,  // Same command ID for get/set
  SEN66_SET_AMBIENT_PRESSURE_CMD_ID = 0x6720,
  SEN66_GET_AMBIENT_PRESSURE_CMD_ID = 0x6720,  // Same command ID for get/set
  SEN66_SET_SENSOR_ALTITUDE_CMD_ID = 0x6736,
  SEN66_GET_SENSOR_ALTITUDE_CMD_ID = 0x6736,  // Same command ID for get/set
  SEN66_ACTIVATE_SHT_HEATER_CMD_ID = 0x6765,
  SEN66_GET_SHT_HEATER_MEASUREMENTS_CMD_ID = 0x6790,
  SEN66_GET_PRODUCT_NAME_CMD_ID = 0xd014,
  SEN66_GET_SERIAL_NUMBER_CMD_ID = 0xd033,
  SEN66_GET_VERSION_CMD_ID = 0xd100,
  SEN66_READ_DEVICE_STATUS_CMD_ID = 0xd206,
  SEN66_READ_AND_CLEAR_DEVICE_STATUS_CMD_ID = 0xd210,
  SEN66_DEVICE_RESET_CMD_ID = 0xd304,
} SEN66_CMD_ID;

/** @brief Represents the device status register flags. Sourced from Sensirion documentation. */
typedef union {
  struct {
    uint32_t reserved1 : 4;          ///< Reserved bits
    uint32_t fan_error : 1;          ///< Fan error flag
    uint32_t reserved2 : 1;          ///< Reserved bit
    uint32_t rht_error : 1;          ///< RHT (Humidity/Temp) sensor error flag
    uint32_t gas_error : 1;          ///< Gas sensor (VOC/NOx) error flag
    uint32_t reserved3 : 1;          ///< Reserved bit
    uint32_t co2_2_error : 1;        ///< CO2 sensor error flag
    uint32_t reserved4 : 1;          ///< Reserved bit
    uint32_t pm_error : 1;           ///< PM sensor error flag
    uint32_t reserved5 : 1;          ///< Reserved bit
    uint32_t reserved6 : 8;          ///< Reserved bits
    uint32_t fan_speed_warning : 1;  ///< Fan speed warning flag (low or high)
  } bits;                            ///< Access individual status flags
  uint32_t value;                    ///< Access the full 32-bit status register value
} sen66_device_status;

/** @brief Structure to hold VOC/NOx algorithm tuning parameters. */
struct GasTuning {
  int16_t index_offset;                 ///< Index offset parameter
  int16_t learning_time_offset_hours;   ///< Learning time offset parameter (hours)
  int16_t learning_time_gain_hours;     ///< Learning time gain parameter (hours)
  int16_t gating_max_duration_minutes;  ///< Gating maximum duration parameter (minutes)
  int16_t std_initial;                  ///< Standard deviation initial parameter
  int16_t gain_factor;                  ///< Gain factor parameter
};

/** @brief Structure to hold temperature compensation parameters. */
struct TemperatureCompensation {
  int16_t offset;                   ///< Temperature offset (scaled by 200)
  int16_t normalized_offset_slope;  ///< Normalized offset slope (scaled by 10000)
  uint16_t time_constant;           ///< Time constant in seconds. Determines how fast new slope and offset are applied.
                           ///< After this time, 63% of new values are applied. Zero means immediate application.
  uint16_t slot;  ///< Slot number (0-4)
};

/** @brief Structure to hold temperature acceleration parameters. */
struct TemperatureAcceleration {
  uint16_t k;   ///< Parameter K (scaled by 10)
  uint16_t p;   ///< Parameter P (scaled by 10)
  uint16_t t1;  ///< Parameter T1 (scaled by 10)
  uint16_t t2;  ///< Parameter T2 (scaled by 10)
};

/**
 * @brief Main class for the SEN66 sensor component.
 *
 * Handles I2C communication, reading sensor values, applying configurations (tuning, compensation),
 * managing operational states, persisting VOC state, and exposing automation actions.
 * Inherits from PollingComponent for periodic updates and SensirionI2CDevice for I2C communication helpers.
 */
class SEN66Component : public PollingComponent, public sensirion_common::SensirionI2CDevice {
 public:
  /** @brief Set component setup priority. */
  float get_setup_priority() const override { return setup_priority::DATA; }
  /** @brief Initialize the sensor, read static info, apply configurations, load state, and start measurement. */
  void setup() override;
  /** @brief Log device information and configuration settings. */
  void dump_config() override;
  /** @brief Called periodically by the scheduler to read sensor data. Manages state transitions. */
  void update() override;

  // --- Sensor Setters (Called by sensor.py during code generation) ---
  /** @brief Set the sensor object for PM1.0 readings. */
  void set_pm_1_0_sensor(sensor::Sensor *pm_1_0) { pm_1_0_sensor_ = pm_1_0; }
  /** @brief Set the sensor object for PM2.5 readings. */
  void set_pm_2_5_sensor(sensor::Sensor *pm_2_5) { pm_2_5_sensor_ = pm_2_5; }
  /** @brief Set the sensor object for PM4.0 readings. */
  void set_pm_4_0_sensor(sensor::Sensor *pm_4_0) { pm_4_0_sensor_ = pm_4_0; }
  /** @brief Set the sensor object for PM10.0 readings. */
  void set_pm_10_0_sensor(sensor::Sensor *pm_10_0) { pm_10_0_sensor_ = pm_10_0; }
  /** @brief Set the sensor object for Number Concentration 0.5µm readings. */
  void set_nc_0_5_sensor(sensor::Sensor *nc_0_5) { nc_0_5_sensor_ = nc_0_5; }
  /** @brief Set the sensor object for Number Concentration 1.0µm readings. */
  void set_nc_1_0_sensor(sensor::Sensor *nc_1_0) { nc_1_0_sensor_ = nc_1_0; }
  /** @brief Set the sensor object for Number Concentration 2.5µm readings. */
  void set_nc_2_5_sensor(sensor::Sensor *nc_2_5) { nc_2_5_sensor_ = nc_2_5; }
  /** @brief Set the sensor object for Number Concentration 4.0µm readings. */
  void set_nc_4_0_sensor(sensor::Sensor *nc_4_0) { nc_4_0_sensor_ = nc_4_0; }
  /** @brief Set the sensor object for Number Concentration 10.0µm readings. */
  void set_nc_10_0_sensor(sensor::Sensor *nc_10_0) { nc_10_0_sensor_ = nc_10_0; }
  /** @brief Set the sensor object for VOC Index readings. */
  void set_voc_sensor(sensor::Sensor *voc_sensor) { voc_sensor_ = voc_sensor; }
  /** @brief Set the sensor object for NOx Index readings. */
  void set_nox_sensor(sensor::Sensor *nox_sensor) { nox_sensor_ = nox_sensor; }
  /** @brief Set the sensor object for Relative Humidity readings. */
  void set_humidity_sensor(sensor::Sensor *humidity_sensor) { humidity_sensor_ = humidity_sensor; }
  /** @brief Set the sensor object for Temperature readings. */
  void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }
  /** @brief Set the sensor object for CO2 readings. */
  void set_co2_sensor(sensor::Sensor *co2_sensor) { co2_sensor_ = co2_sensor; }

  // --- Configuration Setters (Called by sensor.py during code generation) ---
  /** @brief Set VOC algorithm tuning parameters from YAML configuration. */
  void set_voc_algorithm_tuning(int16_t index_offset, int16_t learning_time_offset_hours,
                                int16_t learning_time_gain_hours, int16_t gating_max_duration_minutes,
                                int16_t std_initial, int16_t gain_factor);
  /** @brief Get the currently configured VOC algorithm tuning parameters. Returns nullopt if not set. */
  optional<GasTuning> get_voc_algorithm_tuning();

  /** @brief Set NOx algorithm tuning parameters from YAML configuration. */
  void set_nox_algorithm_tuning(int16_t index_offset, int16_t learning_time_offset_hours,
                                int16_t learning_time_gain_hours, int16_t gating_max_duration_minutes,
                                int16_t std_initial, int16_t gain_factor);
  /** @brief Get the currently configured NOx algorithm tuning parameters. Returns nullopt if not set. */
  optional<GasTuning> get_nox_algorithm_tuning();

  /** @brief Set temperature compensation parameters from YAML configuration for a specific slot. */
  void set_temperature_compensation(float offset, float normalized_offset_slope, uint16_t time_constant,
                                    uint16_t slot = 0);

  /** @brief Set temperature acceleration parameters from YAML configuration. */
  void set_temperature_acceleration_parameters(float k, float p, float t1, float t2);

  /** @brief Enable or disable CO2 Automatic Self-Calibration (ASC). */
  void set_co2_automatic_self_calibration(bool enable);
  /** @brief Get the current status of CO2 Automatic Self-Calibration (ASC). Returns nullopt on error. */
  optional<bool> get_co2_automatic_self_calibration();

  /** @brief Set the ambient pressure for CO2 compensation. */
  void set_ambient_pressure(uint16_t ambient_pressure);
  /** @brief Get the currently set ambient pressure. Returns nullopt on error. */
  optional<uint16_t> get_ambient_pressure();

  /** @brief Set the sensor altitude for CO2 compensation. */
  void set_sensor_altitude(uint16_t altitude);
  /** @brief Get the currently set sensor altitude. Returns nullopt on error. */
  optional<uint16_t> get_sensor_altitude();

  /** @brief Set the maximum number of consecutive communication failures before triggering a device reboot. */
  void set_max_consecutive_failures(uint8_t max_failures);

  // --- Public Methods for Actions & Services ---
  /**
   * @brief Perform a Forced Recalibration (FRC) for the CO2 sensor.
   * Stops measurement, sends the FRC command, waits, reads the result, and restarts measurement.
   * @param target_co2_concentration The target CO2 concentration in ppm.
   * @return The correction factor applied by the sensor (scaled by 10000), or nullopt on failure.
   */
  optional<uint16_t> perform_forced_co2_recalibration(uint16_t target_co2_concentration);

  /**
   * @brief Activate the SHT sensor's internal heater.
   * Stops measurement, activates heater, waits (~20s), deactivates (implicitly), and restarts measurement.
   * Useful in high humidity to prevent condensation.
   * @return True if the heater activation command was sent successfully, false otherwise.
   */
  bool activate_sht_heater();

  /**
   * @brief Get the measurements taken during the SHT heater activation cycle.
   * Should be called *after* activate_sht_heater completes.
   * @return A pair containing <humidity, temperature> measured during heating, or nullopt on error.
   */
  optional<std::pair<float, float>> get_sht_heater_measurements();

  /**
   * @brief Read the current device status register.
   * @return The device status flags, or nullopt on communication error.
   */
  optional<sen66_device_status> read_device_status();

  /**
   * @brief Read the device status register and clear any latched status flags.
   * @return The device status flags before clearing, or nullopt on communication error.
   */
  optional<sen66_device_status> read_and_clear_device_status();

  /**
   * @brief Start the fan cleaning cycle.
   * Stops measurement, starts cleaning, waits (~10s), and restarts measurement.
   * @return True if the cleaning command was sent successfully, false otherwise.
   */
  bool start_fan_cleaning();

  /**
   * @brief Perform a factory reset on the sensor.
   * Stops measurement, clears the component's persistent data (VOC state and last save timestamp)
   * from NVS using `global_preferences->reset()`, sends the hardware reset command to the sensor,
   * and then triggers a safe reboot of the ESPHome device to ensure re-initialization.
   * **Warning:** This clears the learned VOC algorithm state.
   */
  void factory_reset();

 protected:
  // --- Internal Setup Steps ---
  /** @brief Performs setup steps that must happen *after* a potential device reset during initial setup. */
  void continue_setup_after_reset_();
  /** @brief Performs setup steps that must happen *after* stopping measurement during initial setup. */
  void continue_setup_after_stop_();

  // --- Internal I2C Write Helpers ---
  /** @brief Writes VOC or NOx tuning parameters to the sensor. */
  bool write_tuning_parameters_(uint16_t i2c_command, const GasTuning &tuning);
  /** @brief Writes temperature compensation parameters to the specified slot on the sensor. */
  bool write_temperature_compensation_(const TemperatureCompensation &compensation);
  /** @brief Writes temperature acceleration parameters to the sensor. */
  bool write_temperature_acceleration_(const TemperatureAcceleration &params);
  /** @brief Writes the saved VOC algorithm state to the sensor. */
  bool write_voc_algorithm_state_(const uint8_t state[8]);
  /** @brief Writes the CO2 Automatic Self-Calibration status to the sensor. */
  bool write_co2_asc_status_(bool enable);
  /** @brief Writes the ambient pressure value to the sensor. */
  bool write_ambient_pressure_(uint16_t pressure);
  /** @brief Writes the sensor altitude value to the sensor. */
  bool write_sensor_altitude_(uint16_t altitude);

  // --- Internal I2C Read Helpers ---
  /** @brief Reads the VOC algorithm state from the sensor. */
  bool read_voc_algorithm_state_(uint8_t state[8]);
  /** @brief Reads the CO2 Automatic Self-Calibration status from the sensor. */
  bool read_co2_asc_status_(bool &enabled);
  /** @brief Reads the ambient pressure value from the sensor. */
  bool read_ambient_pressure_(uint16_t &pressure);
  /** @brief Reads the sensor altitude value from the sensor. */
  bool read_sensor_altitude_(uint16_t &altitude);
  /** @brief Reads humidity and temperature values after heater activation from the sensor. */
  bool read_sht_heater_measurements_(float &humidity, float &temperature);
  /** @brief Generic helper to read either VOC or NOx tuning parameters. */
  bool read_tuning_parameters_(uint16_t i2c_command, GasTuning &tuning);
  /** @brief Internal helper to read device status (with or without clearing). */
  bool read_device_status_internal_(uint16_t command, sen66_device_status &status);

  // --- State Management & Timing Helpers ---
  /**
   * @brief Stops the sensor's measurement and the ESPHome poller if currently active.
   *
   * Called before initiating actions like cleaning, heating, FRC, or reset.
   * Manages `current_state_`, `polling_active_before_action_`, `original_interval_before_action_`,
   * and sends the `SEN66_STOP_MEASUREMENT_CMD_ID`.
   *
   * @return true if measurement was stopped successfully or was already stopped.
   * @return false if the stop command failed.
   */
  bool stop_measurement_if_needed_();

  /**
   * @brief Restarts measurements and polling after an action completes.
   *
   * Scheduled via `set_timeout` after the action's duration (or FRC result read).
   * Checks `polling_active_before_action_` to determine if polling should resume.
   * Sends `SEN66_START_CONTINUOUS_MEASUREMENT_CMD_ID`, restores polling interval,
   * sets `next_update_allowed_time_` for stabilization, and sets `current_state_` to `MEASURING` or `IDLE`.
   *
   * @param command_success Flag indicating if the preceding action command was successful.
   */
  void handle_action_completion_(bool command_success = true);

  /**
   * @brief Reads the FRC result after the required sensor processing delay.
   *
   * Scheduled via `set_timeout` after sending the FRC command. Reads the correction factor,
   * logs it, and then calls `handle_action_completion_` to restart measurements.
   */
  void handle_frc_read_result_();

  // --- VOC Algorithm State Persistence ---
  /** @brief Reads the VOC algorithm state from the sensor and saves it to ESPHome's preferences. */
  bool save_voc_algorithm_state_();
  /** @brief Loads the VOC algorithm state from ESPHome's preferences and writes it to the sensor. */
  bool load_voc_algorithm_state_();

  // --- Member Variables ---
  // Sensor entity pointers (null if not configured)
  sensor::Sensor *pm_1_0_sensor_{nullptr};
  sensor::Sensor *pm_2_5_sensor_{nullptr};
  sensor::Sensor *pm_4_0_sensor_{nullptr};
  sensor::Sensor *pm_10_0_sensor_{nullptr};
  sensor::Sensor *nc_0_5_sensor_{nullptr};
  sensor::Sensor *nc_1_0_sensor_{nullptr};
  sensor::Sensor *nc_2_5_sensor_{nullptr};
  sensor::Sensor *nc_4_0_sensor_{nullptr};
  sensor::Sensor *nc_10_0_sensor_{nullptr};
  sensor::Sensor *voc_sensor_{nullptr};
  sensor::Sensor *nox_sensor_{nullptr};
  sensor::Sensor *humidity_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *co2_sensor_{nullptr};

  // Configuration storage
  optional<GasTuning> voc_tuning_params_{};                ///< Optional storage for configured VOC tuning params.
  optional<GasTuning> nox_tuning_params_{};                ///< Optional storage for configured NOx tuning params.
  optional<TemperatureCompensation> temp_comp_params_{};   ///< Optional storage for configured Temp Comp params.
  optional<TemperatureAcceleration> temp_accel_params_{};  ///< Optional storage for configured Temp Accel params.
  optional<bool> co2_asc_enabled_{};                       ///< Optional storage for configured CO2 ASC state.
  optional<uint16_t> ambient_pressure_hpa_{};              ///< Optional storage for configured ambient pressure.
  optional<uint16_t> sensor_altitude_m_{};                 ///< Optional storage for configured sensor altitude.

  // State management
  ComponentState current_state_{IDLE};  ///< Current operational state of the component.
  uint8_t consecutive_failures_{0};     ///< Counter for consecutive communication errors.
  uint8_t max_consecutive_failures_{10};
  bool initialized_{false};  ///< Tracks if the component has been initialized.

  // Timing and action flow control
  bool polling_active_before_action_{false};     ///< Tracks if polling was active before an action started.
  uint32_t original_interval_before_action_{0};  ///< Stores the original update interval when polling is paused.
  uint32_t next_update_allowed_time_{0};         ///< Earliest time `update()` is allowed to run after state changes.

  // FRC state
  optional<uint16_t> frc_target_concentration_{};  ///< Target concentration stored during FRC process.

  // VOC state persistence
  ESPPreferenceObject voc_state_pref_;  ///< Preference object for storing the raw VOC algorithm state persistently.
  ESPPreferenceObject last_voc_save_time_pref_;  ///< Preference object for storing the timestamp of the last VOC state
                                                 ///< save persistently.
  uint32_t last_voc_save_time_{0};  ///< In-memory timestamp (millis()) of the last VOC state save attempt (success or
                                    ///< failure). Loaded from/saved to `last_voc_save_time_pref_`.

  // Static sensor information
  struct VersionInfo {
    uint8_t firmware_major;
    uint8_t firmware_minor;
  } version_info_{};               ///< Sensor version information read during setup.
  std::string product_name_{""};   ///< Sensor product name read during setup.
  std::string serial_number_{""};  ///< Sensor serial number read during setup.
};

}  // namespace sen66
}  // namespace esphome
