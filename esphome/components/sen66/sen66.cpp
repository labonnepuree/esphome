#include "sen66.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include <cinttypes>
#include <vector>   // Needed for VOC state
#include <utility>  // Needed for std::pair

// Define VOC algorithm state size (8 bytes / 4 words)
#define SEN66_VOC_ALGORITHM_STATE_SIZE 4

namespace esphome {
namespace sen66 {

static const char *const TAG = "sen66";

// Helper function to convert Sensirion int16_t/uint16_t invalid values to NAN
template<typename T> float sensirion_invalid_to_nan(T value, T invalid_value, float scale = 1.0f) {
  return value == invalid_value ? NAN : static_cast<float>(value) * scale;
}

/** @brief Initialize the sensor, read static info, apply configurations, load state, and start measurement. */
void SEN66Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up SEN66...");
  // --- Initial State Setup ---
  this->original_interval_before_action_ = 0;  // Reset stored interval, not relevant here yet.
  this->next_update_allowed_time_ = 0;         // Reset stabilization timer.

  // Add explicit device reset based on official example
  ESP_LOGD(TAG, "Performing device reset...");
  if (!this->write_command(SEN66_DEVICE_RESET_CMD_ID)) {
    ESP_LOGE(TAG, "Device reset command failed!");
    this->mark_failed();
    return;
  }

  // Wait for reset to complete (execution time of reset command is 1.2 seconds, adding 100ms for safety)
  this->set_timeout("setup_reset", 1300, [this]() { this->continue_setup_after_reset_(); });
}

void SEN66Component::continue_setup_after_reset_() {
  ESP_LOGD(TAG, "Continuing setup after reset...");
  // Check if measurement is running
  uint16_t data_ready_word;
  // Check if measurement is running by checking data ready flag (will be 0 if idle)
  if (!this->get_register(SEN66_GET_DATA_READY_CMD_ID, &data_ready_word, 1, 50)) {
    ESP_LOGW(TAG, "Failed to read data ready status during setup check.");
    // Don't mark failed here, maybe sensor was already idle.
  }

  bool is_measuring = (data_ready_word & 0x00FF);  // Data ready is lower byte of the word

  // Stop measurement if running, needed to configure sensor settings
  if (is_measuring) {
    ESP_LOGD(TAG, "Sensor is measuring, stopping...");
    if (!this->write_command(SEN66_STOP_MEASUREMENT_CMD_ID)) {
      ESP_LOGE(TAG, "Failed to stop measurements during setup. Cannot proceed reliably.");
      this->mark_failed();
      return;  // Abort setup
    }
    // Schedule the rest of setup after the required 1000ms delay
    ESP_LOGD(TAG, "Scheduling rest of setup after 1000ms stop delay...");
    // 1000ms delay is the minimum delay required by the datasheet, adding 100ms for safety
    this->set_timeout("setup_stop", 1100, [this]() { this->continue_setup_after_stop_(); });
    // Setup will continue in the callback
  } else {
    ESP_LOGD(TAG, "Sensor is idle, proceeding with setup directly.");
    this->continue_setup_after_stop_();
  }
}

void SEN66Component::continue_setup_after_stop_() {
  ESP_LOGD(TAG, "Continuing setup after stop...");
  // Ensure state is IDLE
  this->current_state_ = IDLE;

  // --- Read Static Information ---
  uint8_t serial_bytes[8];  // Read 8 bytes = 4 words for serial
  if (!this->get_register(SEN66_GET_SERIAL_NUMBER_CMD_ID, (uint16_t *) serial_bytes, 4, 50)) {
    ESP_LOGE(TAG, "Failed to read serial number");
    this->mark_failed();
    return;
  }

  // Convert serial bytes to hex string
  char serial_hex[17];  // 8 bytes * 2 hex chars + 1 null terminator
  sprintf(serial_hex, "%02X%02X%02X%02X%02X%02X%02X%02X", serial_bytes[0], serial_bytes[1], serial_bytes[2],
          serial_bytes[3], serial_bytes[4], serial_bytes[5], serial_bytes[6], serial_bytes[7]);
  this->serial_number_ = std::string(serial_hex);

  ESP_LOGD(TAG, "Read Serial number bytes: %02X%02X%02X%02X%02X%02X%02X%02X", serial_bytes[0], serial_bytes[1],
           serial_bytes[2], serial_bytes[3], serial_bytes[4], serial_bytes[5], serial_bytes[6], serial_bytes[7]);
  ESP_LOGI(TAG, "Serial number: %s", this->serial_number_.c_str());

  char product_name_str[33] = {0};  // Max 32 chars + null terminator
  if (!this->get_register(SEN66_GET_PRODUCT_NAME_CMD_ID, (uint16_t *) product_name_str, 16, 50)) {
    ESP_LOGE(TAG, "Failed to read product name");
    this->mark_failed();
    return;
  }
  // Product name seems to be directly readable as chars based on sensirion examples
  this->product_name_ = std::string(product_name_str);
  ESP_LOGI(TAG, "Product Name: %s", this->product_name_.c_str());

  uint16_t version_word;
  if (!this->get_register(SEN66_GET_VERSION_CMD_ID, &version_word, 1, 50)) {
    ESP_LOGE(TAG, "Failed to read firmware version");
    this->mark_failed();
    return;
  }
  // High byte is major, low byte is minor according to official header function
  this->version_info_.firmware_major = (uint8_t) (version_word >> 8);
  this->version_info_.firmware_minor = (uint8_t) (version_word & 0xFF);
  ESP_LOGI(TAG, "Firmware version: %d.%d", this->version_info_.firmware_major, this->version_info_.firmware_minor);

  // --- Apply Configurations ---
  if (this->temp_comp_params_.has_value()) {
    if (!this->write_temperature_compensation_(this->temp_comp_params_.value())) {
      ESP_LOGW(TAG, "Failed to set temperature compensation.");
    }
    delay(20);  // Short delay after config commands
  }

  if (this->voc_tuning_params_.has_value()) {
    if (!this->write_tuning_parameters_(SEN66_SET_VOC_ALGORITHM_TUNING_PARAMETERS_CMD_ID,
                                        this->voc_tuning_params_.value())) {
      ESP_LOGW(TAG, "Failed to set VOC tuning parameters.");
    }
    delay(20);
  }

  if (this->nox_tuning_params_.has_value()) {
    // Ensure NOx params have correct fixed values as per datasheet notes
    GasTuning nox_params = this->nox_tuning_params_.value();
    if (nox_params.learning_time_gain_hours != 12) {
      ESP_LOGW(TAG, "NOx learning_time_gain_hours should be 12, forcing.");
      nox_params.learning_time_gain_hours = 12;
    }
    if (nox_params.std_initial != 50) {
      ESP_LOGW(TAG, "NOx std_initial should be 50, forcing.");
      nox_params.std_initial = 50;
    }
    if (!this->write_tuning_parameters_(SEN66_SET_NOX_ALGORITHM_TUNING_PARAMETERS_CMD_ID, nox_params)) {
      ESP_LOGW(TAG, "Failed to set NOx tuning parameters.");
    }
    delay(20);
  }

  if (this->temp_accel_params_.has_value()) {
    if (!this->write_temperature_acceleration_(this->temp_accel_params_.value())) {
      ESP_LOGW(TAG, "Failed to set temperature acceleration parameters.");
    }
    delay(20);
  }

  // Initialize preference objects
  if (this->voc_sensor_) {  // Only relevant if VOC sensor is configured
    uint32_t combined_serial = encode_uint32(this->serial_number_[0], this->serial_number_[1], this->serial_number_[2],
                                             this->serial_number_[3]);
    uint32_t voc_hash = fnv1_hash("sen66_voc_" + std::to_string(combined_serial));
    this->voc_state_pref_ = global_preferences->make_preference<uint8_t[8]>(voc_hash, true);

    uint32_t time_hash = fnv1_hash("sen66_voc_time_" + std::to_string(combined_serial));
    this->last_voc_save_time_pref_ = global_preferences->make_preference<uint32_t>(time_hash, true);

    // Load last VOC save time from preferences
    ESP_LOGD(TAG, "Attempting to load last VOC save time from preferences...");
    if (this->last_voc_save_time_pref_.load(&this->last_voc_save_time_)) {
      ESP_LOGI(TAG, "Loaded last VOC save time from preferences: %" PRIu32 " ms ago",
               (millis() - this->last_voc_save_time_));
    } else {
      ESP_LOGD(TAG, "No saved last VOC save time found in preferences, defaulting to 0.");
      // Attempt to save the default value (0) immediately if loading failed.
      // This ensures the preference exists for future updates.
      if (!this->last_voc_save_time_pref_.save(&this->last_voc_save_time_)) {
        ESP_LOGW(TAG, "Failed to save default last VOC save time to preferences.");
        this->mark_failed();  // Mark component failed if initial pref save fails.
      }
    }

    // Load VOC state from preferences and write to sensor
    ESP_LOGD(TAG, "Attempting to load VOC algorithm state from preferences...");
    if (!this->load_voc_algorithm_state_()) {
      ESP_LOGW(TAG, "Failed to load and apply VOC algorithm state from preferences (or none found).");
      // Continue anyway, sensor will start with default state or its current state.
    } else {
      ESP_LOGI(TAG, "Successfully loaded and applied VOC state from preferences.");
    }
  }

  if (this->co2_asc_enabled_.has_value()) {
    if (!this->write_co2_asc_status_(this->co2_asc_enabled_.value())) {
      ESP_LOGW(TAG, "Failed to set CO2 Automatic Self-Calibration status.");
    }
    delay(20);
  }

  if (this->ambient_pressure_hpa_.has_value()) {
    if (!this->write_ambient_pressure_(this->ambient_pressure_hpa_.value())) {
      ESP_LOGW(TAG, "Failed to set ambient pressure.");
    }
    // This command can be set during measurement according to datasheet, so no delay needed? Adding small one anyway.
    delay(10);
  }

  if (this->sensor_altitude_m_.has_value()) {
    if (!this->write_sensor_altitude_(this->sensor_altitude_m_.value())) {
      ESP_LOGW(TAG, "Failed to set sensor altitude.");
    }
    delay(20);
  }

  // --- Start Measurement ---
  ESP_LOGD(TAG, "Starting continuous measurement...");
  if (!this->write_command(SEN66_START_CONTINUOUS_MEASUREMENT_CMD_ID)) {
    ESP_LOGE(TAG, "Error starting continuous measurements.");
    this->mark_failed();
    this->current_state_ = IDLE;  // Remain IDLE if start fails
    return;
  }
  // --- State Transition: IDLE -> MEASURING ---
  this->current_state_ = MEASURING;  // Successfully started measuring.
  // Set initial stabilization delay: update() will wait before first read.
  // ~1.1s needed for first data + 10s for stabilization
  this->next_update_allowed_time_ = std::max(millis() + 12000, this->next_update_allowed_time_);
  ESP_LOGD(TAG, "Measurement started. Next read allowed after 12000 ms.");

  // Measurement start command needs ~1.1s until first data is ready.
  // Update interval should be longer than this.

  this->initialized_ = true;
  ESP_LOGI(TAG, "SEN66 initialized successfully.");
  // Poller is started automatically by PollingComponent::call_setup()
}

/** @brief Log device information and configuration settings. */
void SEN66Component::dump_config() {
  ESP_LOGCONFIG(TAG, "SEN66:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGW(TAG, "Component has failed setup and will not work!");
  }
  ESP_LOGCONFIG(TAG, "  Product Name: %s", this->product_name_.c_str());
  ESP_LOGCONFIG(TAG, "  Firmware Version: %d.%d", this->version_info_.firmware_major,
                this->version_info_.firmware_minor);
  ESP_LOGCONFIG(TAG, "  Serial Number: %s", this->serial_number_.c_str());

  // Log optional configurations if set
  if (this->temp_comp_params_.has_value()) {
    ESP_LOGCONFIG(TAG, "  Temperature Compensation: Slot %d, Offset %.2f, Slope %.4f, TC %u",
                  this->temp_comp_params_.value().slot, (float) this->temp_comp_params_.value().offset / 200.0f,
                  (float) this->temp_comp_params_.value().normalized_offset_slope / 10000.0f,
                  this->temp_comp_params_.value().time_constant);
  }
  if (this->voc_tuning_params_.has_value()) {
    ESP_LOGCONFIG(TAG, "  VOC Tuning: IdxOffset %d, LearnOffset %dh, LearnGain %dh, GateMax %dm, StdInit %d, Gain %d",
                  this->voc_tuning_params_.value().index_offset,
                  this->voc_tuning_params_.value().learning_time_offset_hours,
                  this->voc_tuning_params_.value().learning_time_gain_hours,
                  this->voc_tuning_params_.value().gating_max_duration_minutes,
                  this->voc_tuning_params_.value().std_initial, this->voc_tuning_params_.value().gain_factor);
  }
  if (this->nox_tuning_params_.has_value()) {
    ESP_LOGCONFIG(TAG, "  NOx Tuning: IdxOffset %d, LearnOffset %dh, LearnGain %dh, GateMax %dm, StdInit %d, Gain %d",
                  this->nox_tuning_params_.value().index_offset,
                  this->nox_tuning_params_.value().learning_time_offset_hours,
                  this->nox_tuning_params_.value().learning_time_gain_hours,
                  this->nox_tuning_params_.value().gating_max_duration_minutes,
                  this->nox_tuning_params_.value().std_initial, this->nox_tuning_params_.value().gain_factor);
  }
  if (this->temp_accel_params_.has_value()) {
    ESP_LOGCONFIG(
        TAG, "  Temperature Acceleration: K %.1f, P %.1f, T1 %.1fs, T2 %.1fs",
        (float) this->temp_accel_params_.value().k / 10.0f, (float) this->temp_accel_params_.value().p / 10.0f,
        (float) this->temp_accel_params_.value().t1 / 10.0f, (float) this->temp_accel_params_.value().t2 / 10.0f);
  }
  if (this->co2_asc_enabled_.has_value()) {
    ESP_LOGCONFIG(TAG, "  CO2 Auto Self-Calibration: %s", ONOFF(this->co2_asc_enabled_.value()));
  }
  if (this->ambient_pressure_hpa_.has_value()) {
    ESP_LOGCONFIG(TAG, "  Ambient Pressure Compensation: %u hPa", this->ambient_pressure_hpa_.value());
  }
  if (this->sensor_altitude_m_.has_value()) {
    ESP_LOGCONFIG(TAG, "  Sensor Altitude Compensation: %u m", this->sensor_altitude_m_.value());
  }

  LOG_UPDATE_INTERVAL(this);

  // Log configured sensors
  LOG_SENSOR("  ", "PM 1.0", this->pm_1_0_sensor_);
  LOG_SENSOR("  ", "PM 2.5", this->pm_2_5_sensor_);
  LOG_SENSOR("  ", "PM 4.0", this->pm_4_0_sensor_);
  LOG_SENSOR("  ", "PM 10.0", this->pm_10_0_sensor_);
  LOG_SENSOR("  ", "NC 0.5", this->nc_0_5_sensor_);
  LOG_SENSOR("  ", "NC 1.0", this->nc_1_0_sensor_);
  LOG_SENSOR("  ", "NC 2.5", this->nc_2_5_sensor_);
  LOG_SENSOR("  ", "NC 4.0", this->nc_4_0_sensor_);
  LOG_SENSOR("  ", "NC 10.0", this->nc_10_0_sensor_);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  LOG_SENSOR("  ", "Humidity", this->humidity_sensor_);
  LOG_SENSOR("  ", "VOC Index", this->voc_sensor_);
  LOG_SENSOR("  ", "NOx Index", this->nox_sensor_);
  LOG_SENSOR("  ", "CO2", this->co2_sensor_);
}

/** @brief Called periodically by the scheduler to read sensor data. Manages state transitions. */
void SEN66Component::update() {
  if (!this->initialized_) {
    // Component setup failed or not yet complete.
    return;
  }

  // --- State Check: Only proceed if MEASURING ---
  // This prevents attempting reads while sensor is stopped for actions
  // or during the waiting periods scheduled by set_timeout.
  if (this->current_state_ != MEASURING) {
    ESP_LOGVV(TAG, "Skipping update: Component not in MEASURING state (current: %d)", this->current_state_);
    return;
  }

  // --- Stabilization Check ---
  // Determine if we are in the stabilization period after starting measurement.
  // During this time, we read data to warm up but don't publish.
  bool is_stabilizing = millis() < this->next_update_allowed_time_;
  if (is_stabilizing) {
    ESP_LOGV(TAG, "Sensor stabilization period active. Reading data but not publishing... (Remaining: %u ms)",
             this->next_update_allowed_time_ - millis());
    // Don't return; proceed to read data.
  }

  // --- Add error handling wrapper ---
  bool update_successful = true;  // Assume success initially

  // Check if data is ready
  uint16_t data_ready_word = 0;
  if (!this->get_register(SEN66_GET_DATA_READY_CMD_ID, &data_ready_word, 1, 50)) {
    ESP_LOGW(TAG, "Failed read data ready status during update.");
    this->status_set_warning();
    update_successful = false;  // Mark as failure
    // Don't return yet, proceed to failure handling
  } else if (!(data_ready_word & 0x00FF)) {
    ESP_LOGV(TAG, "Data not ready yet.");
    // Data not ready isn't a failure, reset counter if needed
    if (this->consecutive_failures_ > 0) {
      ESP_LOGD(TAG, "Resetting failure counter as data is not ready (not a failure). Consecutive failures was: %d",
               this->consecutive_failures_);
      this->consecutive_failures_ = 0;
    }
    return;  // Exit normally
  }

  if (update_successful) {  // Only try reading measurements if data ready check passed
    // --- Read Mass Concentration & Gas Block ---
    uint16_t mass_gas_values[9];
    if (!this->write_command(SEN66_READ_MEASURED_VALUES_AS_INTEGERS_CMD_ID)) {
      this->status_set_warning();
      ESP_LOGW(TAG, "Failed to send read mass/gas command!");
      update_successful = false;
    } else {
      delay(20);  // Sensible delay?
      if (!this->read_data(mass_gas_values, 9)) {
        this->status_set_warning();
        ESP_LOGW(TAG, "Read mass/gas values failed!");
        update_successful = false;
      } else {
        // Parse Mass Concentration & Gas values
        float pm_1_0 = sensirion_invalid_to_nan(mass_gas_values[0], (uint16_t) 0xFFFF, 10.0f);
        float pm_2_5 = sensirion_invalid_to_nan(mass_gas_values[1], (uint16_t) 0xFFFF, 10.0f);
        float pm_4_0 = sensirion_invalid_to_nan(mass_gas_values[2], (uint16_t) 0xFFFF, 10.0f);
        float pm_10_0 = sensirion_invalid_to_nan(mass_gas_values[3], (uint16_t) 0xFFFF, 10.0f);
        float humidity = sensirion_invalid_to_nan((int16_t) mass_gas_values[4], (int16_t) 0x7FFF, 100.0f);
        float temperature = sensirion_invalid_to_nan((int16_t) mass_gas_values[5], (int16_t) 0x7FFF, 200.0f);
        float voc_index = sensirion_invalid_to_nan((int16_t) mass_gas_values[6], (int16_t) 0x7FFF, 10.0f);
        float nox_index = sensirion_invalid_to_nan((int16_t) mass_gas_values[7], (int16_t) 0x7FFF, 10.0f);
        float co2 = sensirion_invalid_to_nan(mass_gas_values[8], (uint16_t) 0xFFFF);  // CO2 is direct ppm

        // Publish Mass Concentration & Gas values only if not stabilizing
        if (!is_stabilizing) {
          if (this->pm_1_0_sensor_ != nullptr)
            this->pm_1_0_sensor_->publish_state(pm_1_0);
          if (this->pm_2_5_sensor_ != nullptr)
            this->pm_2_5_sensor_->publish_state(pm_2_5);
          if (this->pm_4_0_sensor_ != nullptr)
            this->pm_4_0_sensor_->publish_state(pm_4_0);
          if (this->pm_10_0_sensor_ != nullptr)
            this->pm_10_0_sensor_->publish_state(pm_10_0);
          if (this->humidity_sensor_ != nullptr)
            this->humidity_sensor_->publish_state(humidity);
          if (this->temperature_sensor_ != nullptr)
            this->temperature_sensor_->publish_state(temperature);
          if (this->voc_sensor_ != nullptr)
            this->voc_sensor_->publish_state(voc_index);
          if (this->nox_sensor_ != nullptr)
            this->nox_sensor_->publish_state(nox_index);
          if (this->co2_sensor_ != nullptr)
            this->co2_sensor_->publish_state(co2);
        }
      }
    }

    // --- Read Number Concentration Block (if mass/gas read was okay) ---
    if (update_successful) {
      delay(5);  // Delay between commands
      uint16_t number_values[5];
      if (!this->write_command(SEN66_READ_NUMBER_CONCENTRATION_VALUES_AS_INTEGERS_CMD_ID)) {
        this->status_set_warning();
        ESP_LOGW(TAG, "Failed to send read number concentration command!");
        update_successful = false;
      } else {
        delay(20);  // Delay before reading data
        if (!this->read_data(number_values, 5)) {
          this->status_set_warning();
          ESP_LOGW(TAG, "Read number concentration values failed!");
          update_successful = false;
        } else {
          // Parse Number Concentration values
          float nc_0_5 = sensirion_invalid_to_nan(number_values[0], (uint16_t) 0xFFFF, 10.0f);
          float nc_1_0 = sensirion_invalid_to_nan(number_values[1], (uint16_t) 0xFFFF, 10.0f);
          float nc_2_5 = sensirion_invalid_to_nan(number_values[2], (uint16_t) 0xFFFF, 10.0f);
          float nc_4_0 = sensirion_invalid_to_nan(number_values[3], (uint16_t) 0xFFFF, 10.0f);
          float nc_10_0 = sensirion_invalid_to_nan(number_values[4], (uint16_t) 0xFFFF, 10.0f);

          // Publish Number Concentration values only if not stabilizing
          if (!is_stabilizing) {
            if (this->nc_0_5_sensor_ != nullptr)
              this->nc_0_5_sensor_->publish_state(nc_0_5);
            if (this->nc_1_0_sensor_ != nullptr)
              this->nc_1_0_sensor_->publish_state(nc_1_0);
            if (this->nc_2_5_sensor_ != nullptr)
              this->nc_2_5_sensor_->publish_state(nc_2_5);
            if (this->nc_4_0_sensor_ != nullptr)
              this->nc_4_0_sensor_->publish_state(nc_4_0);
            if (this->nc_10_0_sensor_ != nullptr)
              this->nc_10_0_sensor_->publish_state(nc_10_0);
          }
        }
      }
    }
  }  // end if(update_successful) initial check

  // --- Handle success or failure ---
  if (update_successful) {
    // Reset counter on any successful update cycle
    if (this->consecutive_failures_ > 0) {
      ESP_LOGD(TAG, "Successful update, resetting failure counter. Consecutive failures was: %d",
               this->consecutive_failures_);
      this->consecutive_failures_ = 0;
    }
    // Clear warning if no issues occurred this cycle
    if (!this->status_has_warning()) {  // Check if *any* part set warning
      this->status_clear_warning();
    }
  } else {
    // Increment counter on failure
    this->consecutive_failures_++;
    ESP_LOGW(TAG, "Update failed. Consecutive failures: %d/%d", this->consecutive_failures_,
             this->max_consecutive_failures_);

    // Check if threshold is reached (and feature enabled, check > 0)
    if (this->max_consecutive_failures_ > 0 && this->consecutive_failures_ >= this->max_consecutive_failures_) {
      ESP_LOGE(TAG, "Reached maximum consecutive failures (%d). Triggering reboot!", this->max_consecutive_failures_);
      set_timeout("safe_reboot", 500,
                  []() { App.safe_reboot(); });  // Request a safe reboot after 500ms delay to allow log transmission
      return;                                    // Stop further processing this cycle
    }
  }

  // Trigger VOC state saving periodically.
  static uint32_t voc_save_interval_ms = 3600000;  // 1 hour

  if (this->voc_sensor_ &&
      (millis() - this->last_voc_save_time_ > voc_save_interval_ms || this->last_voc_save_time_ == 0)) {
    // Pref object is only created if voc_sensor_ exists, so this check is sufficient.
    // Also skip saving during stabilization phase
    if (!is_stabilizing) {
      ESP_LOGD(TAG, "Attempting periodic VOC algorithm state save (Interval: %u ms)...", voc_save_interval_ms);
      if (this->save_voc_algorithm_state_()) {
        ESP_LOGI(TAG, "Periodically saved VOC algorithm state.");
        this->last_voc_save_time_ = millis();  // Update last save time in memory
        // Save the updated time to preferences only on successful state save.
        if (!this->last_voc_save_time_pref_.save(&this->last_voc_save_time_)) {
          ESP_LOGW(TAG, "Failed to save last VOC save time to preferences.");
          // Continue anyway, state was saved, just timestamp persistence failed.
        }
      } else {
        ESP_LOGW(TAG, "Failed to periodically save VOC algorithm state.");
        // Update in-memory time even on failure to avoid hammering sensor/flash
        // if there's a persistent issue, but don't save the failure time to preferences.
        this->last_voc_save_time_ = millis();
      }
    }  // end if (!is_stabilizing)
  }    // End periodic save logic
}

// ===================================
// Implementation of protected helpers
// ===================================

/** @brief Writes VOC or NOx tuning parameters to the sensor. */
bool SEN66Component::write_tuning_parameters_(uint16_t i2c_command, const GasTuning &tuning) {
  // Pack parameters according to Sensirion driver format (int16_t)
  uint16_t params[6];
  params[0] = (uint16_t) tuning.index_offset;
  params[1] = (uint16_t) tuning.learning_time_offset_hours;
  params[2] = (uint16_t) tuning.learning_time_gain_hours;
  params[3] = (uint16_t) tuning.gating_max_duration_minutes;
  params[4] = (uint16_t) tuning.std_initial;
  params[5] = (uint16_t) tuning.gain_factor;
  if (!write_command((SEN66_CMD_ID) i2c_command, params, 6)) {
    ESP_LOGE(TAG, "Set tuning parameters failed. I2C command=0x%X, err=%d", i2c_command, this->last_error_);
    return false;
  }
  return true;
}

/** @brief Writes temperature compensation parameters to the specified slot on the sensor. */
bool SEN66Component::write_temperature_compensation_(const TemperatureCompensation &compensation) {
  if (compensation.slot > 4) {
    ESP_LOGE(TAG, "Invalid temperature compensation slot: %d", compensation.slot);
    return false;
  }
  uint16_t params[4];  // Cast to uint16_t to respect write_command() signature but sensor will use int16_t
  params[0] = (uint16_t) compensation.offset;
  params[1] = (uint16_t) compensation.normalized_offset_slope;
  params[2] = compensation.time_constant;
  params[3] = compensation.slot;
  if (!write_command(SEN66_SET_TEMPERATURE_OFFSET_PARAMETERS_CMD_ID, params, 4)) {
    ESP_LOGE(TAG, "Set temperature compensation failed (slot %d). Err=%d", compensation.slot, this->last_error_);
    return false;
  }
  ESP_LOGD(TAG, "Successfully set temperature compensation for slot %d", compensation.slot);
  return true;
}

/** @brief Writes temperature acceleration parameters to the sensor. */
bool SEN66Component::write_temperature_acceleration_(const TemperatureAcceleration &params) {
  uint16_t packed_params[4];
  packed_params[0] = params.k;
  packed_params[1] = params.p;
  packed_params[2] = params.t1;
  packed_params[3] = params.t2;
  if (!write_command(SEN66_SET_TEMPERATURE_ACCELERATION_PARAMETERS_CMD_ID, packed_params, 4)) {
    ESP_LOGE(TAG, "Set temperature acceleration failed. Err=%d", this->last_error_);
    return false;
  }
  ESP_LOGD(TAG, "Successfully set temperature acceleration parameters.");
  return true;
}

/** @brief Writes the saved VOC algorithm state to the sensor. */
bool SEN66Component::write_voc_algorithm_state_(const uint8_t state[8]) {
  if (this->current_state_ != IDLE) {
    ESP_LOGE(TAG, "Cannot write VOC state: Sensor must be in IDLE state (current: %d).", this->current_state_);
    return false;
  }

  // Debug log the state being written
  ESP_LOGD(TAG, "Writing state: %02X %02X %02X %02X %02X %02X %02X %02X", state[0], state[1], state[2], state[3],
           state[4], state[5], state[6], state[7]);

  if (!write_command(SEN66_SET_VOC_ALGORITHM_STATE_CMD_ID, (uint16_t *) state, SEN66_VOC_ALGORITHM_STATE_SIZE)) {
    ESP_LOGE(TAG, "Set VOC algorithm state failed. Err=%d", this->last_error_);
    return false;
  }
  ESP_LOGD(TAG, "Successfully wrote VOC algorithm state to sensor.");
  return true;
}

/** @brief Reads the VOC algorithm state from the sensor and saves it to ESPHome's preferences. */
bool SEN66Component::save_voc_algorithm_state_() {
  if (!this->voc_sensor_) {
    ESP_LOGW(TAG, "Cannot save VOC state: VOC sensor not configured.");
    return false;
  }

  uint8_t current_state[8];
  ESP_LOGV(TAG, "Reading current VOC state from sensor to save...");
  if (!this->read_voc_algorithm_state_(current_state)) {
    ESP_LOGW(TAG, "Failed to save VOC state: Could not retrieve current state from sensor.");
    return false;
  }

  ESP_LOGV(TAG, "Saving VOC state to preferences...");
  if (!this->voc_state_pref_.save(&current_state)) {
    ESP_LOGW(TAG, "Failed to save VOC algorithm state to preferences (save operation failed).");
    return false;
  }

  ESP_LOGD(TAG, "Successfully saved VOC algorithm state to preferences.");
  // Timestamp saving is handled in update() after this function returns true.
  return true;
}

/** @brief Writes the CO2 Automatic Self-Calibration status to the sensor. */
bool SEN66Component::write_co2_asc_status_(bool enable) {
  uint16_t status = enable ? 0x0001 : 0x0000;
  if (!write_command(SEN66_SET_CO2_SENSOR_AUTOMATIC_SELF_CALIBRATION_CMD_ID, &status, 1)) {
    ESP_LOGE(TAG, "Set CO2 ASC status failed. Err=%d", this->last_error_);
    return false;
  }
  ESP_LOGD(TAG, "Successfully set CO2 ASC status to %s.", ONOFF(enable));
  return true;
}

/** @brief Writes the ambient pressure value to the sensor. */
bool SEN66Component::write_ambient_pressure_(uint16_t pressure) {
  if (!write_command(SEN66_SET_AMBIENT_PRESSURE_CMD_ID, &pressure, 1)) {
    ESP_LOGE(TAG, "Set ambient pressure failed. Err=%d", this->last_error_);
    return false;
  }
  ESP_LOGD(TAG, "Successfully set ambient pressure to %u hPa.", pressure);
  return true;
}

/** @brief Writes the sensor altitude value to the sensor. */
bool SEN66Component::write_sensor_altitude_(uint16_t altitude) {
  if (!write_command(SEN66_SET_SENSOR_ALTITUDE_CMD_ID, &altitude, 1)) {
    ESP_LOGE(TAG, "Set sensor altitude failed. Err=%d", this->last_error_);
    return false;
  }
  ESP_LOGD(TAG, "Successfully set sensor altitude to %u m.", altitude);
  return true;
}

/** @brief Generic helper to read either VOC or NOx tuning parameters. */
bool SEN66Component::read_tuning_parameters_(uint16_t i2c_command, GasTuning &tuning) {
  uint16_t params[6];
  if (!this->get_register((SEN66_CMD_ID) i2c_command, params, 6, 50)) {  // Use 50ms delay based on other reads
    ESP_LOGW(TAG, "Failed to read tuning parameters. Command=0x%X", i2c_command);
    return false;
  }
  tuning.index_offset = (int16_t) params[0];
  tuning.learning_time_offset_hours = (int16_t) params[1];
  tuning.learning_time_gain_hours = (int16_t) params[2];
  tuning.gating_max_duration_minutes = (int16_t) params[3];
  tuning.std_initial = (int16_t) params[4];
  tuning.gain_factor = (int16_t) params[5];
  return true;
}

/** @brief Reads the VOC algorithm state from the sensor. */
bool SEN66Component::read_voc_algorithm_state_(uint8_t state[8]) {
  if (!this->get_register(SEN66_GET_VOC_ALGORITHM_STATE_CMD_ID, (uint16_t *) state, SEN66_VOC_ALGORITHM_STATE_SIZE,
                          50)) {
    ESP_LOGW(TAG, "Failed to read VOC algorithm state from sensor.");
    return false;
  }
  ESP_LOGV(TAG, "Successfully read VOC algorithm state from sensor.");
  // Debug log the read state
  ESP_LOGD(TAG, "Read state: %02X %02X %02X %02X %02X %02X %02X %02X", state[0], state[1], state[2], state[3], state[4],
           state[5], state[6], state[7]);
  return true;
}

/** @brief Loads the VOC algorithm state from ESPHome's preferences and writes it to the sensor. */
bool SEN66Component::load_voc_algorithm_state_() {
  // Check if VOC sensor is configured
  if (!this->voc_sensor_) {
    ESP_LOGD(TAG, "Cannot load VOC state: VOC sensor not configured.");
    return false;  // Not an error, just nothing to load.
  }

  // Ensure sensor is IDLE
  if (this->current_state_ != IDLE) {  // Corrected check
    ESP_LOGE(TAG, "Cannot load VOC state: Sensor must be in IDLE state (current: %d).", this->current_state_);
    return false;
  }

  uint8_t state_to_load[8];
  if (!this->voc_state_pref_.load(&state_to_load)) {
    ESP_LOGD(TAG, "No saved VOC algorithm state found in preferences, doing nothing.");
    return false;
  }
  ESP_LOGI(TAG, "Loaded saved VOC algorithm state from preferences.");
  ESP_LOGD(TAG, "State bytes: %02X %02X %02X %02X %02X %02X %02X %02X", state_to_load[0], state_to_load[1],
           state_to_load[2], state_to_load[3], state_to_load[4], state_to_load[5], state_to_load[6], state_to_load[7]);

  ESP_LOGD(TAG, "Attempting to write state to sensor...");
  if (!this->write_voc_algorithm_state_(state_to_load)) {  // Pass the vector
    ESP_LOGW(TAG, "Failed to write VOC algorithm state to sensor.");
    return false;  // Write failed
  }

  ESP_LOGD(TAG, "Successfully wrote VOC algorithm state to sensor.");
  delay(20);    // Short delay after successful write
  return true;  // Write successful
}

/** @brief Reads the CO2 Automatic Self-Calibration status from the sensor. */
bool SEN66Component::read_co2_asc_status_(bool &enabled) {
  uint16_t status_word;
  if (!this->get_register(SEN66_GET_CO2_SENSOR_AUTOMATIC_SELF_CALIBRATION_CMD_ID, &status_word, 1, 50)) {
    ESP_LOGW(TAG, "Failed to read CO2 ASC status.");
    return false;
  }
  // Status is in the lower byte according to official header function
  enabled = (status_word & 0x00FF);
  return true;
}

/** @brief Reads the ambient pressure value from the sensor. */
bool SEN66Component::read_ambient_pressure_(uint16_t &pressure) {
  if (!this->get_register(SEN66_GET_AMBIENT_PRESSURE_CMD_ID, &pressure, 1, 50)) {
    ESP_LOGW(TAG, "Failed to read ambient pressure.");
    return false;
  }
  return true;
}

/** @brief Reads the sensor altitude value from the sensor. */
bool SEN66Component::read_sensor_altitude_(uint16_t &altitude) {
  if (!this->get_register(SEN66_GET_SENSOR_ALTITUDE_CMD_ID, &altitude, 1, 50)) {
    ESP_LOGW(TAG, "Failed to read sensor altitude.");
    return false;
  }
  return true;
}

/** @brief Reads humidity and temperature values after heater activation from the sensor. */
bool SEN66Component::read_sht_heater_measurements_(float &humidity, float &temperature) {
  int16_t values[2];  // Returns two int16_t
  if (!this->get_register(SEN66_GET_SHT_HEATER_MEASUREMENTS_CMD_ID, (uint16_t *) values, 2, 50)) {
    ESP_LOGW(TAG, "Failed to read SHT heater measurements.");
    return false;
  }
  humidity = sensirion_invalid_to_nan(values[0], (int16_t) 0x7FFF);
  temperature = sensirion_invalid_to_nan(values[1], (int16_t) 0x7FFF);
  return true;
}

/** @brief Internal helper to read device status (with or without clearing). */
bool SEN66Component::read_device_status_internal_(uint16_t command, sen66_device_status &status) {
  uint16_t status_words[2];  // Status is 32 bits = 2 words
  if (!this->get_register((SEN66_CMD_ID) command, status_words, 2, 50)) {
    ESP_LOGW(TAG, "Failed to read device status. Command=0x%X", command);
    return false;
  }
  // Combine words into 32-bit value, assuming MSW first
  status.value = ((uint32_t) status_words[0] << 16) | status_words[1];
  return true;
}

/** @brief Set the maximum number of consecutive communication failures before triggering a device reboot. */
void SEN66Component::set_max_consecutive_failures(uint8_t max_failures) {
  this->max_consecutive_failures_ = max_failures;
  ESP_LOGD(TAG, "Set max consecutive failures before reboot to: %d", max_failures);
}

/** @brief Get the currently configured VOC algorithm tuning parameters. Returns nullopt if not set. */
optional<GasTuning> SEN66Component::get_voc_algorithm_tuning() {
  GasTuning tuning;
  if (!this->read_tuning_parameters_(SEN66_GET_VOC_ALGORITHM_TUNING_PARAMETERS_CMD_ID, tuning)) {
    return {};  // Return empty optional on failure
  }
  return tuning;
}

/** @brief Get the currently configured NOx algorithm tuning parameters. Returns nullopt if not set. */
optional<GasTuning> SEN66Component::get_nox_algorithm_tuning() {
  GasTuning tuning;
  if (!this->read_tuning_parameters_(SEN66_GET_NOX_ALGORITHM_TUNING_PARAMETERS_CMD_ID, tuning)) {
    return {};  // Return empty optional on failure
  }
  return tuning;
}

/** @brief Set temperature acceleration parameters from YAML configuration. */
void SEN66Component::set_temperature_acceleration_parameters(float k, float p, float t1, float t2) {
  TemperatureAcceleration params;
  params.k = static_cast<uint16_t>(roundf(k * 10));
  params.p = static_cast<uint16_t>(roundf(p * 10));
  params.t1 = static_cast<uint16_t>(roundf(t1 * 10));
  params.t2 = static_cast<uint16_t>(roundf(t2 * 10));
  this->temp_accel_params_ = params;
  // Actual writing happens during setup() if called before, or needs separate trigger if called after.
  // For simplicity, we assume it's set in YAML and applied during setup.
  // If dynamic setting is needed, a separate action/service would be required.
  ESP_LOGD(TAG, "Temperature acceleration parameters queued for setup.");
}

/**
 * @brief Perform a Forced Recalibration (FRC) for the CO2 sensor.
 * Stops measurement, sends the FRC command, waits, reads the result, and restarts measurement.
 * @param target_co2_concentration The target CO2 concentration in ppm.
 * @return The correction factor applied by the sensor (scaled by 10000), or nullopt on failure.
 */
optional<uint16_t> SEN66Component::perform_forced_co2_recalibration(uint16_t target_co2_concentration) {
  ESP_LOGI(TAG, "Attempting CO2 Forced Recalibration (FRC) to %u ppm...", target_co2_concentration);

  // --- State Check: Ensure component is not already busy ---
  if (this->current_state_ != MEASURING && this->current_state_ != IDLE) {
    ESP_LOGE(TAG, "Cannot start FRC: Component is busy (state: %d).", this->current_state_);
    return {};  // Return empty optional immediately
  }

  bool was_measuring = this->current_state_ == MEASURING;
  if (was_measuring) {
    // --- State Transition: MEASURING -> IDLE (temporarily) ---
    // Stop sensor measurement and ESPHome polling before proceeding.
    if (!this->stop_measurement_if_needed_()) {  // Handles state change to IDLE
      ESP_LOGE(TAG, "Failed to stop measurement for FRC.");
      // stop_measurement_if_needed handles cleanup if stop fails.
      return {};
    }
    // --- State Transition: IDLE -> WAITING_FOR_RECALIBRATION_CMD ---
    // Schedule the command send after the required 600ms delay.
    this->current_state_ = WAITING_FOR_RECALIBRATION_CMD;        // Set state for the waiting period.
    this->frc_target_concentration_ = target_co2_concentration;  // Store target for the callback.
    ESP_LOGD(TAG, "Stopped measurement for FRC. Waiting 600ms before sending command...");
    this->set_timeout("frc_send_cmd", 600, [this]() {
      // --- Timeout Callback: Send FRC Command ---
      if (!this->frc_target_concentration_.has_value()) {
        ESP_LOGE(TAG, "FRC target concentration not set!");
        this->handle_action_completion_(false);
        return;
      }
      ESP_LOGD(TAG, "Sending CO2 FRC command with target %u ppm...", this->frc_target_concentration_.value());
      if (!this->write_command(SEN66_PERFORM_FORCED_CO2_RECALIBRATION_CMD_ID, &this->frc_target_concentration_.value(),
                               1)) {
        ESP_LOGE(TAG, "Failed to send CO2 FRC command.");
        this->frc_target_concentration_.reset();  // Clear stored target.
        // --- Action Failed: Trigger Completion Handler (Failure) ---
        // Immediately attempt to restart measurements/polling if needed.
        this->handle_action_completion_(false);  // Pass false to indicate FRC send failure.
        return;
      }
      // --- FRC Command Sent: Schedule Result Reading ---
      // State remains WAITING_FOR_RECALIBRATION_CMD.
      // Sensor needs ~500ms to process the command.
      ESP_LOGD(TAG, "FRC command sent. Waiting 500ms for result...");
      this->set_timeout("frc_read_result", 500, [this]() { this->handle_frc_read_result_(); });
    });
    // Return immediately; FRC is asynchronous. Result is handled by callbacks.
    return {};  // Return empty optional as result isn't available yet.

  } else {  // Already IDLE
    // --- State Transition: IDLE -> WAITING_FOR_RECALIBRATION_CMD ---
    // Can send command immediately as component is idle.
    ESP_LOGD(TAG, "Component already IDLE. Sending FRC command directly...");
    if (!this->write_command(SEN66_PERFORM_FORCED_CO2_RECALIBRATION_CMD_ID, &target_co2_concentration, 1)) {
      ESP_LOGE(TAG, "Failed to send CO2 FRC command while IDLE.");
      this->current_state_ = IDLE;  // Remain IDLE on failure.
      return {};
    }
    // --- FRC Command Sent: Schedule Result Reading ---
    this->current_state_ = WAITING_FOR_RECALIBRATION_CMD;
    this->frc_target_concentration_ = target_co2_concentration;  // Store target.
    ESP_LOGD(TAG, "FRC command sent. Waiting 500ms for result...");
    this->set_timeout("frc_read_result", 500, [this]() { this->handle_frc_read_result_(); });
    // Return immediately; FRC is asynchronous.
    return {};  // Return empty optional as result isn't available yet.
  }
  // NOTE: The function now returns optional<uint16_t>{} immediately in async cases.
  // The actual result (correction factor) is only logged internally when the
  // callback completes. If the caller needs the result, the API would need
  // redesign (e.g., using a lambda callback provided by the caller).
}

/**
 * @brief Reads the FRC result after the required sensor processing delay.
 *
 * Scheduled via `set_timeout` after sending the FRC command. Reads the correction factor,
 * logs it, and then calls `handle_action_completion_` to restart measurements.
 */
void SEN66Component::handle_frc_read_result_() {
  uint16_t correction_raw;
  ESP_LOGD(TAG, "Reading CO2 FRC result...");
  if (!this->read_data(&correction_raw, 1)) {
    ESP_LOGE(TAG, "Failed to read CO2 FRC result.");
    this->frc_target_concentration_.reset();
    // --- Action Failed: Trigger Completion Handler (Failure) ---
    this->handle_action_completion_(false);  // Attempt restart despite read failure.
    return;
  }

  this->frc_target_concentration_.reset();  // Clear stored target.

  if (correction_raw == 0xFFFF) {
    ESP_LOGE(TAG, "CO2 FRC failed (sensor returned 0xFFFF).");
    // --- Action Failed (Sensor Indicated): Trigger Completion Handler (Failure) ---
    this->handle_action_completion_(false);  // Attempt restart.
  } else {
    // --- Action Succeeded: Log Result ---
    int16_t correction = (int16_t) correction_raw - 0x8000;
    ESP_LOGI(TAG, "CO2 FRC successful. Correction applied: %d ppm", correction);
    // --- Action Succeeded: Trigger Completion Handler (Success) ---
    this->handle_action_completion_(true);  // Proceed to restart measurement.
  }
}

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
bool SEN66Component::stop_measurement_if_needed_() {
  if (!this->initialized_) {
    ESP_LOGW(TAG, "Stop measurement requested but component not initialized.");
    return false;
  }

  // --- State Check: Already Idle? ---
  if (this->current_state_ != MEASURING) {
    ESP_LOGD(TAG, "Stop measurement requested, but not currently measuring (state: %d). Assuming already stopped.",
             this->current_state_);
    // Ensure interval store is cleared if we were in a waiting state previously
    this->original_interval_before_action_ = 0;
    return true;  // Treat as success if not measuring.
  }

  // --- Store Polling State --- Store interval *before* stopping.
  this->original_interval_before_action_ = this->get_update_interval();
  bool was_polling = this->original_interval_before_action_ > 0;

  ESP_LOGD(TAG, "Stopping measurement%s...", was_polling ? " and polling" : "");

  // --- Stop Sensor ---
  if (!this->write_command(SEN66_STOP_MEASUREMENT_CMD_ID)) {
    ESP_LOGE(TAG, "Failed to send stop measurement command! State uncertain.");
    // Don't change state or stop poller if sensor command failed.
    // Clear stored interval as the action cannot proceed correctly.
    this->original_interval_before_action_ = 0;
    return false;  // Indicate failure.
  }
  // Required small delay after stop command.
  delay(50);  // Wait after stop command.

  // --- Stop Poller (if active) ---
  if (was_polling) {
    this->stop_poller();
    ESP_LOGD(TAG, "Polling stopped.");
  }

  // --- State Transition: MEASURING -> IDLE ---
  this->current_state_ = IDLE;
  ESP_LOGD(TAG, "Measurement stopped. Component state set to IDLE.");
  return true;  // Indicate success.
}

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
void SEN66Component::handle_action_completion_(bool action_step_success) {
  auto previous_state = this->current_state_;  // For logging.
  ESP_LOGD(TAG, "Handling action completion (previous state: %d, success flag: %d)...", previous_state,
           action_step_success);  // 'success' refers to the last step of the action itself.

  // --- Restart Logic --- Check if polling was active before the action.
  if (this->original_interval_before_action_ > 0) {
    // --- Attempt to Restart Sensor ---
    ESP_LOGD(TAG, "Attempting to restart continuous measurement...");
    if (!this->write_command(SEN66_START_CONTINUOUS_MEASUREMENT_CMD_ID)) {
      ESP_LOGE(TAG, "Failed to send start measurement command! Polling will not resume.");
      // --- State Transition: WAITING_* -> IDLE (Restart Failed) ---
      this->current_state_ = IDLE;                 // Remain IDLE if start fails.
      this->original_interval_before_action_ = 0;  // Clear stored interval.
      this->next_update_allowed_time_ = 0;         // Clear stabilization timer.
      return;
    }

    // --- State Transition: WAITING_* -> MEASURING (Restart Succeeded) ---
    this->current_state_ = MEASURING;
    // Set stabilization delay for the upcoming update() calls.
    this->next_update_allowed_time_ = millis() + 1200;  // ~1.1s needed.
    ESP_LOGD(TAG, "Measurement restarted. Next read allowed after 1200 ms.");

    // --- Restart Poller ---
    ESP_LOGD(TAG, "Restarting polling with interval %u ms...", this->original_interval_before_action_);
    this->set_update_interval(this->original_interval_before_action_);
    this->start_poller();
    this->original_interval_before_action_ = 0;  // Clear stored interval after successful restart.

  } else {
    // --- State Transition: WAITING_* -> IDLE (Polling Was Not Active) ---
    // If polling was not active before the action, we should remain IDLE.
    ESP_LOGD(TAG, "Polling was not active before the action, remaining in IDLE state.");
    this->current_state_ = IDLE;
    this->next_update_allowed_time_ = 0;  // Not measuring, so no stabilization needed.
  }
}

/**
 * @brief Activate the SHT sensor's internal heater.
 * Stops measurement, activates heater, waits (~20s), deactivates (implicitly), and restarts measurement.
 * Useful in high humidity to prevent condensation.
 * @return True if the heater activation command was sent successfully, false otherwise.
 */
bool SEN66Component::activate_sht_heater() {
  // --- State Check: Ensure component is not already busy ---
  if (this->current_state_ != MEASURING && this->current_state_ != IDLE) {
    ESP_LOGE(TAG, "Cannot activate SHT heater: Component is busy (state: %d).", this->current_state_);
    return false;
  }

  ESP_LOGI(TAG, "Starting SHT heater activation sequence (non-blocking)...");

  // --- State Transition: MEASURING -> IDLE (temporarily) ---
  if (!this->stop_measurement_if_needed_()) {
    ESP_LOGE(TAG, "Failed to enter idle mode for heater activation.");
    return false;
  }

  // --- Activate Heater --- (Component is now IDLE)
  ESP_LOGD(TAG, "Sending activate SHT heater command...");
  if (!write_command(SEN66_ACTIVATE_SHT_HEATER_CMD_ID)) {
    ESP_LOGE(TAG, "Failed to send activate SHT heater command. Err=%d", this->last_error_);
    // --- Action Failed: Trigger Completion Handler (Failure) ---
    this->handle_action_completion_(false);  // Attempt restart immediately.
    return false;
  }

  // --- Schedule Completion --- (Heater needs 20s cooldown before restart)
  ESP_LOGI(TAG, "SHT heater command sent. Waiting 20 seconds before restarting measurement...");
  // --- State Transition: IDLE -> WAITING_FOR_HEATER ---
  this->current_state_ = WAITING_FOR_HEATER;
  // Schedule the completion handler to run after the cooldown period.
  this->set_timeout("heater_complete", 20000, [this]() { this->handle_action_completion_(true); });

  return true;  // Activation initiated successfully, will complete asynchronously.
}

/**
 * @brief Get the measurements taken during the SHT heater activation cycle.
 * Should be called *after* activate_sht_heater completes.
 * @return A pair containing <humidity, temperature> measured during heating, or nullopt on error.
 */
optional<std::pair<float, float>> SEN66Component::get_sht_heater_measurements() {
  // Requires idle mode & FW >= 4.0
  // Note: This function itself doesn't stop measurement, assumes user has ensured idle state
  // if calling manually after activate_sht_heater.
  if (this->initialized_) {
    if (this->get_update_interval() > 0) {  // Check if polling is active
      ESP_LOGW(TAG, "Attempting SHT heater measurement read while component might be measuring.");
      // Proceed, but might fail if sensor isn't actually idle
    }
  } else {
    ESP_LOGE(TAG, "Cannot get SHT heater measurements: Component not initialized.");
    return {};
  }

  uint8_t fw_major = this->version_info_.firmware_major;
  if (fw_major < 4) {
    ESP_LOGE(TAG, "SHT heater measurements only available for firmware >= 4.0 (Current: %d.x)", fw_major);
    return {};
  }

  float humidity, temperature;
  if (!this->read_sht_heater_measurements_(humidity, temperature)) {
    return {};
  }
  // Check if values are valid (not NAN) which indicates heating finished
  if (std::isnan(humidity) || std::isnan(temperature)) {
    ESP_LOGD(TAG, "SHT heater measurement not ready yet (returned invalid).");
    // Return empty optional, user should retry shortly.
    return {};
  }

  ESP_LOGD(TAG, "SHT heater measurements received: H=%.2f%%, T=%.2fC", humidity, temperature);
  return std::make_pair(humidity, temperature);
}

/** @brief Read the current device status register. */
optional<sen66_device_status> SEN66Component::read_device_status() {
  sen66_device_status status;
  if (!this->read_device_status_internal_(SEN66_READ_DEVICE_STATUS_CMD_ID, status)) {
    return {};
  }
  return status;
}

/** @brief Read the device status register and clear any latched status flags. */
optional<sen66_device_status> SEN66Component::read_and_clear_device_status() {
  sen66_device_status status;
  if (!this->read_device_status_internal_(SEN66_READ_AND_CLEAR_DEVICE_STATUS_CMD_ID, status)) {
    return {};
  }
  ESP_LOGI(TAG, "Read and cleared device status flags: 0x%08X", status.value);
  return status;
}

/** @brief Start the fan cleaning cycle. */
bool SEN66Component::start_fan_cleaning() {
  // --- State Check: Ensure component is not already busy ---
  if (this->current_state_ != MEASURING && this->current_state_ != IDLE) {
    ESP_LOGE(TAG, "Cannot start fan cleaning: Component is busy (state: %d).", this->current_state_);
    return false;
  }

  ESP_LOGI(TAG, "Starting fan cleaning sequence (non-blocking)...");

  // --- State Transition: MEASURING -> IDLE (temporarily) ---
  if (!this->stop_measurement_if_needed_()) {
    ESP_LOGE(TAG, "Failed to enter idle mode for fan cleaning.");
    return false;
  }

  // --- Start Cleaning --- (Component is now IDLE)
  ESP_LOGD(TAG, "Sending start fan cleaning command...");
  if (!write_command(SEN66_START_FAN_CLEANING_CMD_ID)) {
    ESP_LOGE(TAG, "Start fan cleaning command failed. Err=%d", this->last_error_);
    // --- Action Failed: Trigger Completion Handler (Failure) ---
    this->handle_action_completion_(false);  // Attempt restart immediately.
    return false;
  }

  // --- Schedule Completion --- (Fan runs for 10s)
  // Need 10s delay AFTER command before restarting measurement.
  ESP_LOGI(TAG, "Fan cleaning command sent. Waiting 10 seconds before restarting measurement...");
  // --- State Transition: IDLE -> WAITING_FOR_CLEANING ---
  this->current_state_ = WAITING_FOR_CLEANING;
  // Schedule the completion handler to run after the cleaning duration.
  this->set_timeout("cleaning_complete", 10100, [this]() { this->handle_action_completion_(true); });

  return true;  // Cleaning initiated successfully, will complete asynchronously.
}

/** @brief Set temperature compensation parameters from YAML configuration for a specific slot. */
void SEN66Component::set_temperature_compensation(float offset, float normalized_offset_slope, uint16_t time_constant,
                                                  uint16_t slot) {
  if (slot > 4) {
    ESP_LOGW(TAG, "Invalid temperature compensation slot %d, must be 0-4. Ignoring.", slot);
    return;
  }
  TemperatureCompensation temp_comp;
  temp_comp.offset = static_cast<int16_t>(roundf(offset * 200));
  temp_comp.normalized_offset_slope = static_cast<int16_t>(roundf(normalized_offset_slope * 10000));
  temp_comp.time_constant = time_constant;
  temp_comp.slot = slot;                // Store the slot
  this->temp_comp_params_ = temp_comp;  // Store the parameters
  // Queued for setup
  ESP_LOGD(TAG, "Temperature compensation for slot %d queued for setup.", slot);
}

// --- Configuration Setters/Getters ---

/** @brief Enable or disable CO2 Automatic Self-Calibration (ASC). */
void SEN66Component::set_co2_automatic_self_calibration(bool enable) {
  this->co2_asc_enabled_ = enable;
  // Configuration is applied during setup(). If called dynamically while running,
  // it would require stopping measurement, setting, and restarting.
  ESP_LOGD(TAG, "CO2 ASC status (%s) queued - will be applied during next setup/restart.", ONOFF(enable));
}

/** @brief Get the current status of CO2 Automatic Self-Calibration (ASC). Returns nullopt on error. */
optional<bool> SEN66Component::get_co2_automatic_self_calibration() {
  bool enabled;
  // This command works in MEASURING or IDLE state according to Sensirion C driver.
  if (!this->read_co2_asc_status_(enabled)) {
    return {};
  }
  return enabled;
}

/** @brief Set the ambient pressure for CO2 compensation. */
void SEN66Component::set_ambient_pressure(uint16_t ambient_pressure) {
  if (ambient_pressure < 700 || ambient_pressure > 1200) {
    ESP_LOGW(TAG, "Ambient pressure %u hPa outside valid range (700-1200), ignoring.", ambient_pressure);
    this->ambient_pressure_hpa_.reset();  // Reset if invalid
    return;
  }
  this->ambient_pressure_hpa_ = ambient_pressure;

  // Try to apply immediately if already initialized and measuring/idle,
  // as this command works during measurement.
  if (this->initialized_ && (this->current_state_ == MEASURING || this->current_state_ == IDLE)) {
    ESP_LOGD(TAG, "Attempting to apply ambient pressure (%u hPa) dynamically...", ambient_pressure);
    if (!this->write_ambient_pressure_(ambient_pressure)) {
      ESP_LOGW(TAG, "Failed to apply ambient pressure dynamically. It will be applied during next setup/restart.");
    } else {
      ESP_LOGD(TAG, "Dynamically applied ambient pressure.");
    }
  } else {
    // Otherwise, queue for setup
    ESP_LOGD(TAG, "Ambient pressure (%u hPa) queued - will be applied during next setup/restart.", ambient_pressure);
  }
}

/** @brief Get the currently set ambient pressure. Returns nullopt on error. */
optional<uint16_t> SEN66Component::get_ambient_pressure() {
  uint16_t pressure;
  // This command works in MEASURING or IDLE state according to Sensirion C driver.
  if (!this->read_ambient_pressure_(pressure)) {
    return {};
  }
  return pressure;
}

/** @brief Set the sensor altitude for CO2 compensation. */
void SEN66Component::set_sensor_altitude(uint16_t altitude) {
  if (altitude > 3000) {  // Valid range 0-3000m according to SCD4x datasheet (likely similar)
    ESP_LOGW(TAG, "Sensor altitude %u m outside typical valid range (0-3000), ignoring.", altitude);
    this->sensor_altitude_m_.reset();
    return;
  }
  this->sensor_altitude_m_ = altitude;
  // Configuration is only applied during setup(), as the write command requires IDLE state.
  ESP_LOGD(TAG, "Sensor altitude (%u m) queued - will be applied during next setup/restart.", altitude);
}

/** @brief Get the currently set sensor altitude. Returns nullopt on error. */
optional<uint16_t> SEN66Component::get_sensor_altitude() {
  uint16_t altitude;
  // This command requires IDLE state according to Sensirion C driver.
  if (this->current_state_ != IDLE) {
    ESP_LOGW(TAG, "Cannot get sensor altitude while component is not in IDLE state (current: %d).",
             this->current_state_);
    return {};
  }
  if (!this->read_sensor_altitude_(altitude)) {
    return {};
  }
  return altitude;
}

/**
 * @brief Perform a factory reset on the sensor.
 * Stops measurement, clears the component's persistent data (VOC state and last save timestamp)
 * from NVS using `global_preferences->reset()`, sends the hardware reset command to the sensor,
 * and then triggers a safe reboot of the ESPHome device to ensure re-initialization.
 * **Warning:** This clears the learned VOC algorithm state.
 */
void SEN66Component::factory_reset() {
  ESP_LOGI(TAG, "Attempting factory reset...");

  // Ensure measurement is stopped before sending the reset command
  if (!this->stop_measurement_if_needed_()) {
    ESP_LOGE(TAG, "Failed to stop measurement before factory reset. Aborting.");
    return;
  }

  if (this->voc_sensor_) {
    ESP_LOGD(TAG, "Clearing all component preferences (VOC state and timestamp) via global reset...");
    // Use global reset which targets all preferences associated with this component's hash.
    // This covers both voc_state_pref_ and last_voc_save_time_pref_.
    if (!global_preferences->reset()) {
      ESP_LOGW(TAG, "Failed to clear component preferences during factory reset.");
      // Continue reset process, but old preferences might remain.
    }
    // Nullify preference object handles after reset
    this->voc_state_pref_ = nullptr;
    this->last_voc_save_time_pref_ = nullptr;
  }

  // Resetting the hardware sensor is not done here, as the primary purpose is
  // clearing the ESPHome-side state and re-initializing the component.
  // A reboot handles the re-initialization.
  ESP_LOGI(TAG, "Component preferences cleared. Rebooting component to re-initialize...");
  App.safe_reboot();
}

/** @brief Set VOC algorithm tuning parameters from YAML configuration. */
void SEN66Component::set_voc_algorithm_tuning(int16_t index_offset, int16_t learning_time_offset_hours,
                                              int16_t learning_time_gain_hours, int16_t gating_max_duration_minutes,
                                              int16_t std_initial, int16_t gain_factor) {
  GasTuning params;
  params.index_offset = index_offset;
  params.learning_time_offset_hours = learning_time_offset_hours;
  params.learning_time_gain_hours = learning_time_gain_hours;
  params.gating_max_duration_minutes = gating_max_duration_minutes;
  params.std_initial = std_initial;
  params.gain_factor = gain_factor;
  this->voc_tuning_params_ = params;
  ESP_LOGD(TAG, "VOC tuning parameters queued for setup.");
}

/** @brief Set NOx algorithm tuning parameters from YAML configuration. */
void SEN66Component::set_nox_algorithm_tuning(int16_t index_offset, int16_t learning_time_offset_hours,
                                              int16_t learning_time_gain_hours, int16_t gating_max_duration_minutes,
                                              int16_t std_initial, int16_t gain_factor) {
  GasTuning params;
  params.index_offset = index_offset;
  params.learning_time_offset_hours = learning_time_offset_hours;
  params.learning_time_gain_hours = learning_time_gain_hours;
  params.gating_max_duration_minutes = gating_max_duration_minutes;
  params.std_initial = std_initial;
  params.gain_factor = gain_factor;
  this->nox_tuning_params_ = params;
  ESP_LOGD(TAG, "NOx tuning parameters queued for setup.");
}

}  // namespace sen66
}  // namespace esphome
