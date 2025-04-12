#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/sensirion_common/i2c_sensirion.h"
#include "esphome/core/application.h"
#include "esphome/core/preferences.h"

namespace esphome {
namespace sen66 {

// Add the SEN66 Command IDs from the official header
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
  SEN66_GET_VOC_ALGORITHM_TUNING_PARAMETERS_CMD_ID = 0x60d0,
  SEN66_SET_NOX_ALGORITHM_TUNING_PARAMETERS_CMD_ID = 0x60e1,
  SEN66_GET_NOX_ALGORITHM_TUNING_PARAMETERS_CMD_ID = 0x60e1,
  SEN66_SET_TEMPERATURE_ACCELERATION_PARAMETERS_CMD_ID = 0x6100,
  SEN66_SET_VOC_ALGORITHM_STATE_CMD_ID = 0x6181,
  SEN66_GET_VOC_ALGORITHM_STATE_CMD_ID = 0x6181,
  SEN66_PERFORM_FORCED_CO2_RECALIBRATION_CMD_ID = 0x6707,
  SEN66_SET_CO2_SENSOR_AUTOMATIC_SELF_CALIBRATION_CMD_ID = 0x6711,
  SEN66_GET_CO2_SENSOR_AUTOMATIC_SELF_CALIBRATION_CMD_ID = 0x6711,
  SEN66_SET_AMBIENT_PRESSURE_CMD_ID = 0x6720,
  SEN66_GET_AMBIENT_PRESSURE_CMD_ID = 0x6720,
  SEN66_SET_SENSOR_ALTITUDE_CMD_ID = 0x6736,
  SEN66_GET_SENSOR_ALTITUDE_CMD_ID = 0x6736,
  SEN66_ACTIVATE_SHT_HEATER_CMD_ID = 0x6765,
  SEN66_GET_SHT_HEATER_MEASUREMENTS_CMD_ID = 0x6790,
  SEN66_GET_PRODUCT_NAME_CMD_ID = 0xd014,
  SEN66_GET_SERIAL_NUMBER_CMD_ID = 0xd033,
  SEN66_GET_VERSION_CMD_ID = 0xd100,
  SEN66_READ_DEVICE_STATUS_CMD_ID = 0xd206,
  SEN66_READ_AND_CLEAR_DEVICE_STATUS_CMD_ID = 0xd210,
  SEN66_DEVICE_RESET_CMD_ID = 0xd304,
} SEN66_CMD_ID;

// Add the device status struct from Sensirion header
typedef union {
  struct {
    uint32_t reserved1 : 4;
    uint32_t fan_error : 1;
    uint32_t reserved2 : 1;
    uint32_t rht_error : 1;
    uint32_t gas_error : 1;
    uint32_t reserved3 : 1;
    uint32_t co2_2_error : 1;
    uint32_t reserved4 : 1;
    uint32_t pm_error : 1;
    uint32_t reserved5 : 1;
    uint32_t reserved6 : 8;
    uint32_t fan_speed_warning : 1;
  };
  uint32_t value;
} sen66_device_status;

struct GasTuning {
  int16_t index_offset;
  int16_t learning_time_offset_hours;
  int16_t learning_time_gain_hours;
  int16_t gating_max_duration_minutes;
  int16_t std_initial;
  int16_t gain_factor;
};

struct TemperatureCompensation {
  int16_t offset;
  int16_t normalized_offset_slope;
  uint16_t time_constant;
};

struct TemperatureAcceleration {
  uint16_t k;
  uint16_t p;
  uint16_t t1;
  uint16_t t2;
};

class SEN66Component : public PollingComponent, public sensirion_common::SensirionI2CDevice {
 public:
  float get_setup_priority() const override { return setup_priority::DATA; }
  void setup() override;
  void dump_config() override;
  void update() override;

  void set_pm_1_0_sensor(sensor::Sensor *pm_1_0) { pm_1_0_sensor_ = pm_1_0; }
  void set_pm_2_5_sensor(sensor::Sensor *pm_2_5) { pm_2_5_sensor_ = pm_2_5; }
  void set_pm_4_0_sensor(sensor::Sensor *pm_4_0) { pm_4_0_sensor_ = pm_4_0; }
  void set_pm_10_0_sensor(sensor::Sensor *pm_10_0) { pm_10_0_sensor_ = pm_10_0; }

  // Add Number Concentration sensors
  void set_nc_0_5_sensor(sensor::Sensor *nc_0_5) { nc_0_5_sensor_ = nc_0_5; }
  void set_nc_1_0_sensor(sensor::Sensor *nc_1_0) { nc_1_0_sensor_ = nc_1_0; }
  void set_nc_2_5_sensor(sensor::Sensor *nc_2_5) { nc_2_5_sensor_ = nc_2_5; }
  void set_nc_4_0_sensor(sensor::Sensor *nc_4_0) { nc_4_0_sensor_ = nc_4_0; }
  void set_nc_10_0_sensor(sensor::Sensor *nc_10_0) { nc_10_0_sensor_ = nc_10_0; }

  void set_voc_sensor(sensor::Sensor *voc_sensor) { voc_sensor_ = voc_sensor; }
  void set_nox_sensor(sensor::Sensor *nox_sensor) { nox_sensor_ = nox_sensor; }
  void set_humidity_sensor(sensor::Sensor *humidity_sensor) { humidity_sensor_ = humidity_sensor; }
  void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }
  // Add CO2 sensor
  void set_co2_sensor(sensor::Sensor *co2_sensor) { co2_sensor_ = co2_sensor; }

  // Update types to int16_t
  void set_voc_algorithm_tuning(int16_t index_offset, int16_t learning_time_offset_hours,
                                int16_t learning_time_gain_hours, int16_t gating_max_duration_minutes,
                                int16_t std_initial, int16_t gain_factor) {
    GasTuning tuning_params;
    tuning_params.index_offset = index_offset;
    tuning_params.learning_time_offset_hours = learning_time_offset_hours;
    tuning_params.learning_time_gain_hours = learning_time_gain_hours;
    tuning_params.gating_max_duration_minutes = gating_max_duration_minutes;
    tuning_params.std_initial = std_initial;
    tuning_params.gain_factor = gain_factor;
    voc_tuning_params_ = tuning_params;
  }
  // Add getter
  optional<GasTuning> get_voc_algorithm_tuning();

  // Update types to int16_t and add std_initial argument (even if unused by NOx)
  void set_nox_algorithm_tuning(int16_t index_offset, int16_t learning_time_offset_hours,
                                int16_t learning_time_gain_hours, int16_t gating_max_duration_minutes,
                                int16_t std_initial, int16_t gain_factor) {
    GasTuning tuning_params;
    tuning_params.index_offset = index_offset;
    tuning_params.learning_time_offset_hours = learning_time_offset_hours;
    tuning_params.learning_time_gain_hours = learning_time_gain_hours;  // Should be 12 according to sensirion doc
    tuning_params.gating_max_duration_minutes = gating_max_duration_minutes;
    tuning_params.std_initial = std_initial;  // Should be 50 according to sensirion doc
    tuning_params.gain_factor = gain_factor;
    nox_tuning_params_ = tuning_params;
  }
  // Add getter
  optional<GasTuning> get_nox_algorithm_tuning();

  // Update to take slot parameter
  void set_temperature_compensation(float offset, float normalized_offset_slope, uint16_t time_constant,
                                    uint16_t slot = 0);

  // Add Temperature Acceleration parameters
  void set_temperature_acceleration_parameters(uint16_t k, uint16_t p, uint16_t t1, uint16_t t2);
  // Add VOC algorithm state methods
  bool set_voc_algorithm_state(const std::vector<uint8_t> &state);
  optional<std::vector<uint8_t>> get_voc_algorithm_state();
  // Add CO2 related methods
  optional<uint16_t> perform_forced_co2_recalibration(uint16_t target_co2_concentration);
  void set_co2_automatic_self_calibration(bool enable);
  optional<bool> get_co2_automatic_self_calibration();
  // Add Pressure/Altitude methods
  void set_ambient_pressure(uint16_t ambient_pressure);
  optional<uint16_t> get_ambient_pressure();
  void set_sensor_altitude(uint16_t altitude);
  optional<uint16_t> get_sensor_altitude();
  // Add SHT Heater methods
  bool activate_sht_heater();
  optional<std::pair<float, float>> get_sht_heater_measurements();  // Returns <humidity, temperature>
  // Add Device Status methods
  optional<sen66_device_status> read_device_status();
  optional<sen66_device_status> read_and_clear_device_status();

  bool start_fan_cleaning();

  // Setter for max consecutive failures
  void set_max_consecutive_failures(uint8_t max_failures);

 protected:
  bool write_tuning_parameters_(uint16_t i2c_command, const GasTuning &tuning);
  // Update to include slot
  bool write_temperature_compensation_(const TemperatureCompensation &compensation, uint16_t slot);
  // Add write methods for new parameters
  bool write_temperature_acceleration_(const TemperatureAcceleration &params);
  bool write_voc_algorithm_state_(const std::vector<uint8_t> &state);
  bool write_co2_asc_status_(bool enable);
  bool write_ambient_pressure_(uint16_t pressure);
  bool write_sensor_altitude_(uint16_t altitude);
  // Add read methods for new parameters/state
  bool read_voc_tuning_parameters_(GasTuning &tuning);
  bool read_nox_tuning_parameters_(GasTuning &tuning);
  bool read_voc_algorithm_state_(std::vector<uint8_t> &state);
  bool read_co2_asc_status_(bool &enabled);
  bool read_ambient_pressure_(uint16_t &pressure);
  bool read_sensor_altitude_(uint16_t &altitude);
  bool read_sht_heater_measurements_(float &humidity, float &temperature);
  bool read_tuning_parameters_(uint16_t i2c_command, GasTuning &tuning);
  bool read_device_status_internal_(uint16_t command, sen66_device_status &status);

  // Internal helpers for actions
  uint32_t stop_measurement_if_needed_();
  void restart_measurement_if_needed_(uint32_t original_interval);

  bool initialized_{false};
  // Mass Concentration Sensors
  sensor::Sensor *pm_1_0_sensor_{nullptr};
  sensor::Sensor *pm_2_5_sensor_{nullptr};
  sensor::Sensor *pm_4_0_sensor_{nullptr};
  sensor::Sensor *pm_10_0_sensor_{nullptr};
  // Number Concentration Sensors
  sensor::Sensor *nc_0_5_sensor_{nullptr};
  sensor::Sensor *nc_1_0_sensor_{nullptr};
  sensor::Sensor *nc_2_5_sensor_{nullptr};
  sensor::Sensor *nc_4_0_sensor_{nullptr};
  sensor::Sensor *nc_10_0_sensor_{nullptr};
  // Gas Sensors
  sensor::Sensor *voc_sensor_{nullptr};
  sensor::Sensor *nox_sensor_{nullptr};
  sensor::Sensor *co2_sensor_{nullptr};
  // Environmental Sensors
  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *humidity_sensor_{nullptr};

  std::string product_name_;
  uint8_t serial_number_[4];
  uint16_t firmware_version_;
  optional<GasTuning> voc_tuning_params_;
  optional<GasTuning> nox_tuning_params_;
  optional<TemperatureCompensation> temperature_compensation_;
  uint16_t temperature_compensation_slot_{0};  // Add slot storage
  // Add members for new configurations
  optional<TemperatureAcceleration> temperature_acceleration_;
  optional<bool> co2_asc_enabled_;
  optional<uint16_t> ambient_pressure_hpa_;
  optional<uint16_t> sensor_altitude_m_;
  std::vector<uint8_t> voc_algorithm_state_to_restore_;  // Buffer to hold state before measurement starts
  ESPPreferenceObject pref_;                             // Change back to ESPPreferenceObject for raw data

  // Member variables for error counting
  uint8_t consecutive_update_failures_{0};
  uint8_t max_consecutive_failures_{10};  // Default value, can be overridden by config
};

}  // namespace sen66
}  // namespace esphome
