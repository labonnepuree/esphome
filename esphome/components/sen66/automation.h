#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "sen66.h"

namespace esphome {
namespace sen66 {

template<typename... Ts> class StartFanAction : public Action<Ts...> {
 public:
  explicit StartFanAction(SEN66Component *sen66) : sen66_(sen66) {}

  void play(Ts... x) override { this->sen66_->start_fan_cleaning(); }

 protected:
  SEN66Component *sen66_;
};

template<typename... Ts> class ActivateShtHeaterAction : public Action<Ts...> {
 public:
  explicit ActivateShtHeaterAction(SEN66Component *sen66) : sen66_(sen66) {}

  void play(Ts... x) override { this->sen66_->activate_sht_heater(); }

 protected:
  SEN66Component *sen66_;
};

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

}  // namespace sen66
}  // namespace esphome
