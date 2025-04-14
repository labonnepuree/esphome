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
template<typename T> float sensirion_invalid_to_nan(T value, T invalid_value) {
  return value == invalid_value ? NAN : static_cast<float>(value);
}

void SEN66Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up SEN66...");
  // --- Initial State Setup ---
  this->current_state_ = IDLE;                 // Start in IDLE state before initialization.
  this->original_interval_before_action_ = 0;  // Reset stored interval, not relevant here yet.
  this->next_update_allowed_time_ = 0;         // Reset stabilization timer.

  // Add explicit device reset based on official example
  ESP_LOGD(TAG, "Performing device reset...");
  if (!this->write_command(SEN66_DEVICE_RESET_CMD_ID)) {
    ESP_LOGE(TAG, "Device reset command failed!");
    this->mark_failed();
    return;
  }

  // Wait for reset to complete (official example uses 1.2 seconds)
  delay(1200);

  // Continue with setup logic directly (no more initial set_timeout)

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
      ESP_LOGE(TAG, "Failed to stop measurements during setup.");
      this->mark_failed();
      return;
    }
    // Wait state transition time (Datasheet doesn't specify for stop->config, using common 50ms)
    // Old code used 200ms. Official examples use ~20-50ms.
    delay(50);
  } else {
    ESP_LOGD(TAG, "Sensor is idle.");
  }

  // --- Read Static Information ---
  uint8_t serial_bytes[8];  // Read 8 bytes = 4 words for serial
  if (!this->get_register(SEN66_GET_SERIAL_NUMBER_CMD_ID, (uint16_t *) serial_bytes, 4, 50)) {
    ESP_LOGE(TAG, "Failed to read serial number");
    this->mark_failed();
    return;
  }
  // Store first 4 bytes as per original code structure, log all 8 for info
  memcpy(this->serial_number_, serial_bytes, 4);
  ESP_LOGD(TAG, "Read Serial number bytes: %02X%02X%02X%02X%02X%02X%02X%02X", serial_bytes[0], serial_bytes[1],
           serial_bytes[2], serial_bytes[3], serial_bytes[4], serial_bytes[5], serial_bytes[6], serial_bytes[7]);
  ESP_LOGI(TAG, "Using Serial number part: %02X%02X%02X%02X", serial_number_[0], serial_number_[1], serial_number_[2],
           serial_number_[3]);

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
  this->firmware_version_ = version_word;  // Store both bytes for now
  ESP_LOGI(TAG, "Firmware version: %d.%d", (uint8_t) (version_word >> 8), (uint8_t) (version_word & 0xFF));

  // --- Apply Configurations ---
  if (this->temperature_compensation_.has_value()) {
    if (!this->write_temperature_compensation_(this->temperature_compensation_.value(),
                                               this->temperature_compensation_slot_)) {
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

  if (this->temperature_acceleration_.has_value()) {
    if (!this->write_temperature_acceleration_(this->temperature_acceleration_.value())) {
      ESP_LOGW(TAG, "Failed to set temperature acceleration parameters.");
    }
    delay(20);
  }

  // Load VOC state from preferences if available, prepare it for writing
  if (this->voc_sensor_) {  // Only relevant if VOC sensor is configured
    uint32_t combined_serial = encode_uint32(this->serial_number_[0], this->serial_number_[1], this->serial_number_[2],
                                             this->serial_number_[3]);
    uint32_t hash = fnv1_hash(App.get_compilation_time() + "_sen66_" + std::to_string(combined_serial));
    this->pref_ = global_preferences->make_preference<uint8_t[8]>(hash);
    uint8_t loaded_state[8];
    if (this->pref_.load(&loaded_state)) {
      // Directly load into the restore buffer if successful
      ESP_LOGI(TAG, "Loaded saved VOC algorithm state.");
      this->voc_algorithm_state_to_restore_.assign(loaded_state, loaded_state + 8);
      // Debug log the loaded state
      ESP_LOGD(TAG, "Loaded state: %02X %02X %02X %02X %02X %02X %02X %02X", loaded_state[0], loaded_state[1],
               loaded_state[2], loaded_state[3], loaded_state[4], loaded_state[5], loaded_state[6], loaded_state[7]);
    } else {
      ESP_LOGD(TAG, "No saved VOC algorithm state found.");
    }
  }

  // Write VOC state BEFORE starting measurement if it was loaded
  if (!this->voc_algorithm_state_to_restore_.empty()) {
    ESP_LOGD(TAG, "Restoring VOC algorithm state...");
    if (!this->write_voc_algorithm_state_(this->voc_algorithm_state_to_restore_)) {
      ESP_LOGW(TAG, "Failed to restore VOC algorithm state.");
      // Continue anyway, sensor will start with default state
    } else {
      ESP_LOGI(TAG, "Successfully restored VOC algorithm state.");
    }
    // Clear the buffer after attempting write
    this->voc_algorithm_state_to_restore_.clear();
    delay(20);
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
  this->next_update_allowed_time_ =
      std::max(millis() + 1200, this->next_update_allowed_time_);  // ~1.1s needed for first data.
  ESP_LOGD(TAG, "Measurement started. Next read allowed after 1200 ms.");

  // Measurement start command needs ~1.1s until first data is ready.
  // Update interval should be longer than this.

  initialized_ = true;
  ESP_LOGI(TAG, "SEN66 initialized successfully.");
  // Poller is started automatically by PollingComponent::call_setup()
}

void SEN66Component::dump_config() {
  ESP_LOGCONFIG(TAG, "SEN66:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGW(TAG, "Component has failed setup and will not work!");
  }
  ESP_LOGCONFIG(TAG, "  Product Name: %s", this->product_name_.c_str());
  ESP_LOGCONFIG(TAG, "  Firmware Version: %d.%d", (uint8_t) (this->firmware_version_ >> 8),
                (uint8_t) (this->firmware_version_ & 0xFF));
  ESP_LOGCONFIG(TAG, "  Serial Number: %02X%02X%02X%02X...", serial_number_[0], serial_number_[1], serial_number_[2],
                serial_number_[3]);  // Show first 4 bytes

  // Log optional configurations if set
  if (this->temperature_compensation_.has_value()) {
    ESP_LOGCONFIG(TAG, "  Temperature Compensation: Slot %d, Offset %.2f, Slope %.4f, TC %u",
                  this->temperature_compensation_slot_, (float) this->temperature_compensation_.value().offset / 200.0f,
                  (float) this->temperature_compensation_.value().normalized_offset_slope / 10000.0f,
                  this->temperature_compensation_.value().time_constant);
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
  if (this->temperature_acceleration_.has_value()) {
    ESP_LOGCONFIG(TAG, "  Temperature Acceleration: K %.1f, P %.1f, T1 %.1fs, T2 %.1fs",
                  (float) this->temperature_acceleration_.value().k / 10.0f,
                  (float) this->temperature_acceleration_.value().p / 10.0f,
                  (float) this->temperature_acceleration_.value().t1 / 10.0f,
                  (float) this->temperature_acceleration_.value().t2 / 10.0f);
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

void SEN66Component::update() {
  if (!initialized_) {
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
  // After starting measurement (in setup or handle_action_completion_), a delay is needed
  // before the first valid reading. This check enforces that delay non-blockingly.
  if (millis() < this->next_update_allowed_time_) {
    ESP_LOGV(TAG, "Waiting for sensor stabilization period to complete... (Remaining: %u ms)",
             this->next_update_allowed_time_ - millis());
    return;  // Skip update cycle until stabilization time is reached
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
    if (this->consecutive_update_failures_ > 0) {
      ESP_LOGD(TAG, "Resetting failure counter as data is not ready (not a failure). Consecutive failures was: %d",
               this->consecutive_update_failures_);
      this->consecutive_update_failures_ = 0;
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
        float pm_1_0 = sensirion_invalid_to_nan(mass_gas_values[0], (uint16_t) 0xFFFF) / 10.0f;
        float pm_2_5 = sensirion_invalid_to_nan(mass_gas_values[1], (uint16_t) 0xFFFF) / 10.0f;
        float pm_4_0 = sensirion_invalid_to_nan(mass_gas_values[2], (uint16_t) 0xFFFF) / 10.0f;
        float pm_10_0 = sensirion_invalid_to_nan(mass_gas_values[3], (uint16_t) 0xFFFF) / 10.0f;
        float humidity = sensirion_invalid_to_nan((int16_t) mass_gas_values[4], (int16_t) 0x7FFF) / 100.0f;
        float temperature = sensirion_invalid_to_nan((int16_t) mass_gas_values[5], (int16_t) 0x7FFF) / 200.0f;
        float voc_index = sensirion_invalid_to_nan((int16_t) mass_gas_values[6], (int16_t) 0x7FFF) / 10.0f;
        float nox_index = sensirion_invalid_to_nan((int16_t) mass_gas_values[7], (int16_t) 0x7FFF) / 10.0f;
        float co2 = sensirion_invalid_to_nan(mass_gas_values[8], (uint16_t) 0xFFFF);  // CO2 is direct ppm

        // Publish Mass Concentration & Gas values
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
          float nc_0_5 = sensirion_invalid_to_nan(number_values[0], (uint16_t) 0xFFFF) / 10.0f;
          float nc_1_0 = sensirion_invalid_to_nan(number_values[1], (uint16_t) 0xFFFF) / 10.0f;
          float nc_2_5 = sensirion_invalid_to_nan(number_values[2], (uint16_t) 0xFFFF) / 10.0f;
          float nc_4_0 = sensirion_invalid_to_nan(number_values[3], (uint16_t) 0xFFFF) / 10.0f;
          float nc_10_0 = sensirion_invalid_to_nan(number_values[4], (uint16_t) 0xFFFF) / 10.0f;

          // Publish Number Concentration values
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
  }  // end if(update_successful) initial check

  // --- Handle success or failure ---
  if (update_successful) {
    // Reset counter on any successful update cycle
    if (this->consecutive_update_failures_ > 0) {
      ESP_LOGD(TAG, "Successful update, resetting failure counter. Consecutive failures was: %d",
               this->consecutive_update_failures_);
      this->consecutive_update_failures_ = 0;
    }
    // Clear warning if no issues occurred this cycle
    if (!this->status_has_warning()) {  // Check if *any* part set warning
      this->status_clear_warning();
    }
  } else {
    // Increment counter on failure
    this->consecutive_update_failures_++;
    ESP_LOGW(TAG, "Update failed. Consecutive failures: %d/%d", this->consecutive_update_failures_,
             this->max_consecutive_failures_);

    // Check if threshold is reached (and feature enabled, check > 0)
    if (this->max_consecutive_failures_ > 0 && this->consecutive_update_failures_ >= this->max_consecutive_failures_) {
      ESP_LOGE(TAG, "Reached maximum consecutive failures (%d). Triggering reboot!", this->max_consecutive_failures_);
      App.safe_reboot();  // Request a safe reboot
      // Optionally: delay slightly to allow log transmission
      delay(500);
      return;  // Stop further processing this cycle
    }
  }

  // This logic replaces the old baseline saving
  // Trigger e.g., every hour? This needs careful consideration regarding flash wear.
  // Example: save every 3600 seconds (1 hour)
  static uint32_t last_voc_save_time = 0;         // Use static to preserve value across calls
  const uint32_t voc_save_interval_ms = 3600000;  // 1 hour

  if (this->voc_sensor_ && (millis() - last_voc_save_time > voc_save_interval_ms || last_voc_save_time == 0)) {
    // Pref object is only created if voc_sensor_ exists, so this check is sufficient.
    ESP_LOGD(TAG, "Attempting periodic VOC algorithm state save (Interval: %u ms)...", voc_save_interval_ms);
    auto current_state_opt = this->get_voc_algorithm_state();
    if (current_state_opt.has_value()) {
      // Convert std::vector to uint8_t array for saving
      if (current_state_opt.value().size() == 8) {
        uint8_t state_to_save[8];
        memcpy(state_to_save, current_state_opt.value().data(), 8);

        // Debug log the state being saved
        ESP_LOGD(TAG, "State to save: %02X %02X %02X %02X %02X %02X %02X %02X", state_to_save[0], state_to_save[1],
                 state_to_save[2], state_to_save[3], state_to_save[4], state_to_save[5], state_to_save[6],
                 state_to_save[7]);

        if (this->pref_.save(&state_to_save)) {
          ESP_LOGI(TAG, "Periodically saved VOC algorithm state.");
          last_voc_save_time = millis();  // Update last save time only on success
        } else {
          ESP_LOGW(TAG, "Failed to periodically save VOC algorithm state (save operation failed).");
          // Optionally clear last_voc_save_time to retry sooner? Or keep it to avoid hammering flash? Keeping it for
          // now.
        }
      } else {
        ESP_LOGW(TAG, "Failed to save VOC state: Unexpected state size (%zu).", current_state_opt.value().size());
      }
    } else {
      ESP_LOGW(TAG, "Failed to save VOC state: Could not retrieve current state from sensor.");
      // Consider if we should update last_voc_save_time here to prevent retrying immediately.
      // Let's update it to avoid constant failed read attempts if the sensor is unresponsive.
      last_voc_save_time = millis();
    }
  }  // End periodic save logic
}

// ===================================
// Implementation of protected helpers
// ===================================

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

// Update signature to include slot
bool SEN66Component::write_temperature_compensation_(const TemperatureCompensation &compensation, uint16_t slot) {
  if (slot > 4) {
    ESP_LOGE(TAG, "Invalid temperature compensation slot: %d", slot);
    return false;
  }
  uint16_t params[4];  // Now 4 words: offset, slope, time_constant, slot
  params[0] = (uint16_t) compensation.offset;
  params[1] = (uint16_t) compensation.normalized_offset_slope;
  params[2] = compensation.time_constant;
  params[3] = slot;
  if (!write_command(SEN66_SET_TEMPERATURE_OFFSET_PARAMETERS_CMD_ID, params, 4)) {
    ESP_LOGE(TAG, "Set temperature compensation failed (slot %d). Err=%d", slot, this->last_error_);
    return false;
  }
  ESP_LOGD(TAG, "Successfully set temperature compensation for slot %d", slot);
  return true;
}

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

bool SEN66Component::write_voc_algorithm_state_(const std::vector<uint8_t> &state) {
  if (state.size() != 8) {
    ESP_LOGE(TAG, "Invalid VOC state size (%zu bytes), expected 8.", state.size());
    return false;
  }
  // Convert byte vector to uint16_t array (4 words)
  uint16_t state_words[SEN66_VOC_ALGORITHM_STATE_SIZE];
  memcpy(state_words, state.data(), 8);

  // Need to potentially byte swap if SensirionI2CDevice handles it automatically
  // Assuming write_command expects host byte order and handles swapping if needed
  if (!write_command(SEN66_SET_VOC_ALGORITHM_STATE_CMD_ID, state_words, SEN66_VOC_ALGORITHM_STATE_SIZE)) {
    ESP_LOGE(TAG, "Set VOC algorithm state failed. Err=%d", this->last_error_);
    return false;
  }
  ESP_LOGD(TAG, "Successfully wrote VOC algorithm state.");
  return true;
}

bool SEN66Component::write_co2_asc_status_(bool enable) {
  uint16_t status = enable ? 0x0001 : 0x0000;
  if (!write_command(SEN66_SET_CO2_SENSOR_AUTOMATIC_SELF_CALIBRATION_CMD_ID, &status, 1)) {
    ESP_LOGE(TAG, "Set CO2 ASC status failed. Err=%d", this->last_error_);
    return false;
  }
  ESP_LOGD(TAG, "Successfully set CO2 ASC status to %s.", ONOFF(enable));
  return true;
}

bool SEN66Component::write_ambient_pressure_(uint16_t pressure) {
  if (!write_command(SEN66_SET_AMBIENT_PRESSURE_CMD_ID, &pressure, 1)) {
    ESP_LOGE(TAG, "Set ambient pressure failed. Err=%d", this->last_error_);
    return false;
  }
  ESP_LOGD(TAG, "Successfully set ambient pressure to %u hPa.", pressure);
  return true;
}

bool SEN66Component::write_sensor_altitude_(uint16_t altitude) {
  if (!write_command(SEN66_SET_SENSOR_ALTITUDE_CMD_ID, &altitude, 1)) {
    ESP_LOGE(TAG, "Set sensor altitude failed. Err=%d", this->last_error_);
    return false;
  }
  ESP_LOGD(TAG, "Successfully set sensor altitude to %u m.", altitude);
  return true;
}

// Read helpers are needed for the getter methods
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

bool SEN66Component::read_voc_algorithm_state_(std::vector<uint8_t> &state) {
  uint16_t state_words[SEN66_VOC_ALGORITHM_STATE_SIZE];
  if (!this->get_register(SEN66_GET_VOC_ALGORITHM_STATE_CMD_ID, state_words, SEN66_VOC_ALGORITHM_STATE_SIZE, 50)) {
    ESP_LOGW(TAG, "Failed to read VOC algorithm state.");
    return false;
  }
  state.resize(8);
  memcpy(state.data(), state_words, 8);
  // Handle potential byte swapping if needed (depends on get_register implementation)
  return true;
}

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

bool SEN66Component::read_ambient_pressure_(uint16_t &pressure) {
  if (!this->get_register(SEN66_GET_AMBIENT_PRESSURE_CMD_ID, &pressure, 1, 50)) {
    ESP_LOGW(TAG, "Failed to read ambient pressure.");
    return false;
  }
  return true;
}

bool SEN66Component::read_sensor_altitude_(uint16_t &altitude) {
  if (!this->get_register(SEN66_GET_SENSOR_ALTITUDE_CMD_ID, &altitude, 1, 50)) {
    ESP_LOGW(TAG, "Failed to read sensor altitude.");
    return false;
  }
  return true;
}

bool SEN66Component::read_sht_heater_measurements_(float &humidity, float &temperature) {
  int16_t values[2];  // Returns two int16_t
  if (!this->get_register(SEN66_GET_SHT_HEATER_MEASUREMENTS_CMD_ID, (uint16_t *) values, 2, 50)) {
    ESP_LOGW(TAG, "Failed to read SHT heater measurements.");
    return false;
  }
  humidity = sensirion_invalid_to_nan(values[0], (int16_t) 0x7FFF) / 100.0f;
  temperature = sensirion_invalid_to_nan(values[1], (int16_t) 0x7FFF) / 200.0f;
  return true;
}

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

void SEN66Component::set_max_consecutive_failures(uint8_t max_failures) {
  this->max_consecutive_failures_ = max_failures;
  ESP_LOGD(TAG, "Set max consecutive failures before reboot to: %d", max_failures);
}

optional<GasTuning> SEN66Component::get_voc_algorithm_tuning() {
  GasTuning tuning;
  if (!this->read_tuning_parameters_(SEN66_GET_VOC_ALGORITHM_TUNING_PARAMETERS_CMD_ID, tuning)) {
    return {};  // Return empty optional on failure
  }
  return tuning;
}

optional<GasTuning> SEN66Component::get_nox_algorithm_tuning() {
  GasTuning tuning;
  if (!this->read_tuning_parameters_(SEN66_GET_NOX_ALGORITHM_TUNING_PARAMETERS_CMD_ID, tuning)) {
    return {};  // Return empty optional on failure
  }
  return tuning;
}

void SEN66Component::set_temperature_acceleration_parameters(uint16_t k, uint16_t p, uint16_t t1, uint16_t t2) {
  TemperatureAcceleration params;
  params.k = k;
  params.p = p;
  params.t1 = t1;
  params.t2 = t2;
  this->temperature_acceleration_ = params;
  // Actual writing happens during setup() if called before, or needs separate trigger if called after.
  // For simplicity, we assume it's set in YAML and applied during setup.
  // If dynamic setting is needed, a separate action/service would be required.
  ESP_LOGD(TAG, "Temperature acceleration parameters queued for setup.");
}

bool SEN66Component::set_voc_algorithm_state(const std::vector<uint8_t> &state) {
  // This function is intended to be called dynamically (e.g., from a service call)
  // It needs to handle stopping measurement, writing state, restarting measurement.
  // This is complex and might be better handled via a dedicated ESPHome "action".
  // For now, provide a basic implementation that queues the state for the *next* setup.
  if (state.size() != 8) {
    ESP_LOGE(TAG, "Invalid VOC state size provided (%zu bytes), expected 8.", state.size());
    return false;
  }
  ESP_LOGI(TAG, "Queuing VOC algorithm state to be restored on next setup/restart.");
  this->voc_algorithm_state_to_restore_ = state;
  // Note: This doesn't apply the state immediately. Sensor needs restart or setup rerun.
  return true;  // Return true indicating state was queued
}

optional<std::vector<uint8_t>> SEN66Component::get_voc_algorithm_state() {
  std::vector<uint8_t> state;
  if (!this->read_voc_algorithm_state_(state)) {
    return {};  // Return empty optional on failure
  }
  return state;
}

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

// --- Timeout Callback: Read FRC Result ---
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

// Internal helper to stop measurement if needed.
// Manages state transitions: MEASURING -> IDLE.
// Stops the ESPHome poller if it was running.
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

// Internal helper to handle completion of asynchronous actions.
// Manages state transitions back to MEASURING or IDLE.
// Restarts the ESPHome poller if it was previously active.
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

  uint8_t fw_major = this->firmware_version_ >> 8;
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

optional<sen66_device_status> SEN66Component::read_device_status() {
  sen66_device_status status;
  if (!this->read_device_status_internal_(SEN66_READ_DEVICE_STATUS_CMD_ID, status)) {
    return {};
  }
  return status;
}

optional<sen66_device_status> SEN66Component::read_and_clear_device_status() {
  sen66_device_status status;
  if (!this->read_device_status_internal_(SEN66_READ_AND_CLEAR_DEVICE_STATUS_CMD_ID, status)) {
    return {};
  }
  ESP_LOGI(TAG, "Read and cleared device status flags: 0x%08X", status.value);
  return status;
}

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

void SEN66Component::set_temperature_compensation(float offset, float normalized_offset_slope, uint16_t time_constant,
                                                  uint16_t slot) {
  if (slot > 4) {
    ESP_LOGW(TAG, "Invalid temperature compensation slot %d, must be 0-4. Ignoring.", slot);
    return;
  }
  TemperatureCompensation temp_comp;
  temp_comp.offset = offset * 200;
  temp_comp.normalized_offset_slope = normalized_offset_slope * 10000;
  temp_comp.time_constant = time_constant;
  temperature_compensation_slot_ = slot;  // Store the slot
  temperature_compensation_ = temp_comp;
  // Queued for setup
  ESP_LOGD(TAG, "Temperature compensation for slot %d queued for setup.", slot);
}

// --- Configuration Setters/Getters ---

void SEN66Component::set_co2_automatic_self_calibration(bool enable) {
  this->co2_asc_enabled_ = enable;
  // Configuration is applied during setup(). If called dynamically while running,
  // it would require stopping measurement, setting, and restarting.
  ESP_LOGD(TAG, "CO2 ASC status (%s) queued - will be applied during next setup/restart.", ONOFF(enable));
}

optional<bool> SEN66Component::get_co2_automatic_self_calibration() {
  bool enabled;
  // This command works in MEASURING or IDLE state according to Sensirion C driver.
  if (!this->read_co2_asc_status_(enabled)) {
    return {};
  }
  return enabled;
}

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

optional<uint16_t> SEN66Component::get_ambient_pressure() {
  uint16_t pressure;
  // This command works in MEASURING or IDLE state according to Sensirion C driver.
  if (!this->read_ambient_pressure_(pressure)) {
    return {};
  }
  return pressure;
}

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

}  // namespace sen66
}  // namespace esphome
