#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "sen66.h"

namespace esphome {
/**
 * @brief Namespace for the SEN66 sensor component.
 */
namespace sen66 {

/**
 * @brief Action to start the fan cleaning process on the SEN66 sensor.
 *
 * This action triggers the fan to run at maximum speed for a short period
 * to clean the sensor chamber of dust and particles.
 */
template<typename... Ts> class StartFanAction : public Action<Ts...> {
 public:
  explicit StartFanAction(SEN66Component *sen66) : sen66_(sen66) {}

  void play(Ts... x) override { this->sen66_->start_fan_cleaning(); }

 protected:
  SEN66Component *sen66_;
};

/**
 * @brief Action to activate the SHT heater on the SEN66 sensor.
 *
 * This action turns on the heater element near the humidity sensor
 * to drive off condensation in high humidity environments.
 */
template<typename... Ts> class ActivateShtHeaterAction : public Action<Ts...> {
 public:
  explicit ActivateShtHeaterAction(SEN66Component *sen66) : sen66_(sen66) {}

  void play(Ts... x) override { this->sen66_->activate_sht_heater(); }

 protected:
  SEN66Component *sen66_;
};

/**
 * @brief Action to perform a forced CO2 recalibration (FRC) on the SEN66 sensor.
 *
 * This action allows calibrating the CO2 sensor to a known reference concentration.
 * The target CO2 concentration is provided as a parameter from the automation.
 */
template<typename... Ts> class PerformForcedCo2RecalibrationAction : public Action<Ts...> {
 public:
  explicit PerformForcedCo2RecalibrationAction(SEN66Component *sen66) : sen66_(sen66) {}

  // Use TEMPLATABLE_VALUE to accept the target concentration from YAML/automation
  TEMPLATABLE_VALUE(uint16_t, target_co2)

  void play(Ts... x) override {
    // Evaluate the template to get the target CO2 value
    uint16_t target = this->target_co2_.value(x...);
    // Call the component method
    this->sen66_->perform_forced_co2_recalibration(target);
    // Note: Return value (correction factor) is logged in C++ but not directly usable in automation here
  }

 protected:
  SEN66Component *sen66_;
};

/**
 * @brief Action to perform a factory reset on the SEN66 sensor.
 *
 * This action resets the sensor to factory defaults, clearing all calibration
 * data and learned parameters. Use with caution as this will require the sensor
 * to relearn environmental conditions.
 */
template<typename... Ts> class FactoryResetAction : public Action<Ts...> {
 public:
  explicit FactoryResetAction(SEN66Component *sen66) : sen66_(sen66) {}

  void play(Ts... x) override {
    ESP_LOGD("factory_reset_action", "Triggering factory reset via action.");
    this->sen66_->factory_reset();
  }

 protected:
  SEN66Component *sen66_;
};

}  // namespace sen66
}  // namespace esphome
