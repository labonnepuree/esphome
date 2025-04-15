# ESPHome SEN66 Component

This is a component for the Sensirion SEN66 all-in-one air quality sensor module for ESPHome. It supports reading various particulate matter (PM), number concentration (NC), environmental (Temperature, Humidity, CO2), and gas index (VOC, NOx) measurements via I2C.

## Supported Sensors

The component can expose the following sensor entities:

| Sensor                      | YAML Key                    | Unit  | Icon                  | Device Class                         |
| :-------------------------- | :-------------------------- | :---- | :-------------------- | :----------------------------------- |
| PM1.0                       | `pm_1_0`                    | µg/m³ | `mdi:chemical-weapon` | `pm1`                                |
| PM2.5                       | `pm_2_5`                    | µg/m³ | `mdi:chemical-weapon` | `pm25`                               |
| PM4.0                       | `pm_4_0`                    | µg/m³ | `mdi:chemical-weapon` | `pm4` (Requires HA 2022.7+)          |
| PM10.0                      | `pm_10_0`                   | µg/m³ | `mdi:chemical-weapon` | `pm10`                               |
| Number Concentration 0.5µm  | `number_concentration_0_5`  | #/cm³ | `mdi:counter`         | `None`                               |
| Number Concentration 1.0µm  | `number_concentration_1_0`  | #/cm³ | `mdi:counter`         | `None`                               |
| Number Concentration 2.5µm  | `number_concentration_2_5`  | #/cm³ | `mdi:counter`         | `None`                               |
| Number Concentration 4.0µm  | `number_concentration_4_0`  | #/cm³ | `mdi:counter`         | `None`                               |
| Number Concentration 10.0µm | `number_concentration_10_0` | #/cm³ | `mdi:counter`         | `None`                               |
| Temperature                 | `temperature`               | °C    | `mdi:thermometer`     | `temperature`                        |
| Humidity                    | `humidity`                  | %     | `mdi:water-percent`   | `humidity`                           |
| VOC Index                   | `voc`                       | Index | `mdi:chemical-weapon` | `volatile_organic_compounds_index`   |
| NOx Index                   | `nox`                       | Index | `mdi:chemical-weapon` | `nitrogen_oxide_index` (HA 2023.12+) |
| CO2                         | `co2`                       | ppm   | `mdi:molecule-co2`    | `carbon_dioxide`                     |

**Note:** VOC and NOx sensors report an _index_ value from 1 to 500, scaled logarithmically based on the recent history of measured values. They do not represent absolute concentrations.

## Configuration Variables

```yaml
sen66:
  id: sen66_sensor
  address: 0x6b # Optional, defaults to 0x6b
  update_interval: 60s # Optional, defaults to 60s

  # -- Sensor definitions (Optional) --
  pm_1_0:
    name: "SEN66 PM1.0"
  pm_2_5:
    name: "SEN66 PM2.5"
  pm_4_0:
    name: "SEN66 PM4.0"
  pm_10_0:
    name: "SEN66 PM10.0"
  number_concentration_0_5:
    name: "SEN66 NC0.5"
  number_concentration_1_0:
    name: "SEN66 NC1.0"
  number_concentration_2_5:
    name: "SEN66 NC2.5"
  number_concentration_4_0:
    name: "SEN66 NC4.0"
  number_concentration_10_0:
    name: "SEN66 NC10.0"
  temperature:
    name: "SEN66 Temperature"
  humidity:
    name: "SEN66 Humidity"
  voc:
    name: "SEN66 VOC Index"
    # Optional VOC algorithm tuning
    algorithm_tuning:
      index_offset: 100
      learning_time_offset_hours: 12
      learning_time_gain_hours: 12
      gating_max_duration_minutes: 180
      std_initial: 50
      gain_factor: 230
  nox:
    name: "SEN66 NOx Index"
    # Optional NOx algorithm tuning (some values are fixed by Sensirion)
    algorithm_tuning:
      index_offset: 1
      learning_time_offset_hours: 12
      # learning_time_gain_hours: 12   # Fixed at 12 for NOx
      gating_max_duration_minutes: 720
      # std_initial: 50                # Fixed at 50 for NOx
      gain_factor: 230
  co2:
    name: "SEN66 CO2"

  # -- Component-Wide Settings (Optional) --
  co2_automatic_self_calibration: true # default: true. Enable/disable CO2 ASC.
  ambient_pressure: 1013 # Optional, Ambient pressure in hPa for CO2 compensation.
  # You can also use a sensor ID:
  # ambient_pressure: !lambda |-
  #   return id(pressure_sensor).state;
  sensor_altitude: 50 # Optional, Sensor altitude in meters above sea level for CO2 compensation.

  # Optional Temperature Compensation (see Sensirion docs for details)
  temperature_compensation:
    offset: 0.0 # Default: 0.0
    normalized_offset_slope: 0.0 # Default: 0.0
    time_constant: 0 # Default: 0 (immediate application)
    slot: 0 # Default: 0 (0-4)

  # Optional Temperature Acceleration (see Sensirion docs for details)
  temperature_acceleration:
    k: 0 # Default: 0
    p: 0 # Default: 0
    t1: 0 # Default: 0
    t2: 0 # Default: 0

  # Optional: Set max communication errors before rebooting ESP (Default: 5)
  max_errors_before_reboot: 10
```

- **id** (Optional, ID): Manually specify the ID used for code generation.
- **address** (Optional, int): Specify the I2C address of the sensor. Defaults to `0x6b`.
- **update_interval** (Optional, Time): Specify the interval for sensor updates. Defaults to `60s`. Minimum recommended interval is >1s due to sensor processing time.
- **Sensor Blocks** (Optional): Define sensor entities for the measurements you want to expose. Each block accepts standard [Sensor variables](https://esphome.io/components/sensor/index.html).
  - `voc` and `nox` blocks can contain an optional `algorithm_tuning` sub-block with parameters:
    - `index_offset` (int): VOC/NOx index representing typical environment conditions. Default: 100 (VOC), 1 (NOx). Range: 1-250.
    - `learning_time_offset_hours` (int): Time constant to estimate the VOC/NOx algorithm offset. Default: 12h. Range: 1-1000. (Fixed to 12 for NOx).
    - `learning_time_gain_hours` (int): Time constant to estimate the VOC/NOx algorithm gain. Default: 12h. Range: 1-1000. (Fixed to 12 for NOx).
    - `gating_max_duration_minutes` (int): Maximum duration of index gating (holding output constant) in minutes. Default: 180 (VOC), 720 (NOx). Range: 0-3000 (0 = disable).
    - `std_initial` (int): Initial estimate for standard deviation. Default: 50. Range: 10-5000. (Fixed to 50 for NOx).
    - `gain_factor` (int): Factor applied to the Index output. Default: 230. Range: 1-1000.
- **co2_automatic_self_calibration** (Optional, boolean): Enable or disable the CO2 sensor's Automatic Self-Calibration (ASC). Defaults to `true`. It's recommended to leave this enabled unless the sensor is never exposed to fresh air (~400 ppm CO2).
- **ambient_pressure** (Optional, int or float): Ambient pressure in hPa (hectopascals/millibars) used for CO2 compensation. Can be a fixed value or a lambda referencing another sensor's state. Defaults to `0` (compensation disabled).
- **sensor_altitude** (Optional, int): Altitude of the sensor in meters above sea level, used for CO2 compensation. Defaults to `0` (compensation disabled).
- **temperature_compensation** (Optional, map): Configuration block for temperature compensation of the RHT sensor. See Sensirion documentation for details on tuning these parameters.
  - `offset` (float): Temperature offset °C. Default: 0.0.
  - `normalized_offset_slope` (float): Normalized offset slope. Default: 0.0.
  - `time_constant` (int): Time constant in seconds for applying the offset/slope. Default: 0 (immediate).
  - `slot` (int): Storage slot for the parameters (0-4). Default: 0.
- **temperature_acceleration** (Optional, map): Configuration block for temperature acceleration mode. Only useful in specific applications where rapid temperature changes need compensation. See Sensirion documentation.
  - `k`, `p`, `t1`, `t2` (float): Tuning parameters. Default: 0 for all (disabled).
- **max_errors_before_reboot** (Optional, int): Set the maximum number of consecutive I2C communication failures before the component requests an ESP reboot. Helps recover from hung I2C states. Defaults to `5`. Set to `0` to disable.

## Automation Actions

The component provides several actions that can be called from automations or services:

### Start Fan Cleaning

Starts the sensor's internal fan cleaning cycle. This runs the fan at high speed for ~10 seconds to dislodge dust. Measurement is paused during this time.

```yaml
# Example: Button to trigger fan cleaning
button:
  - platform: template
    name: "SEN66 Fan Clean"
    on_press:
      - sen66.start_fan_cleaning: sen66_sensor # Use the ID of your sen66 component
```

- **sen66.start_fan_cleaning:** `id` (Required, ID): The ID of the SEN66 component.

### Activate SHT Heater

Activates the internal heater near the Temperature/Humidity sensor for ~20 seconds. This can help drive off condensation in high humidity environments (>90% RH) that might affect readings. Measurement is paused during this time.

```yaml
# Example: Automation to activate heater if humidity is high
on_value:
  then:
    - if:
        condition:
          for:
            time: 6h
            condition:
              lambda: |-
                return id(sen66_humidity_id).state > 95;
        then:
          - sen66.activate_sht_heater: sen66_id
```

- **sen66.activate_sht_heater:** `id` (Required, ID): The ID of the SEN66 component.

### Perform Forced CO2 Recalibration (FRC)

Performs a Forced Recalibration (FRC) of the CO2 sensor against a known target concentration. This procedure takes approximately 3 minutes, during which measurements are paused. **Use with caution:** Only perform FRC if you know the ambient CO2 concentration accurately (e.g., by placing the sensor outdoors in fresh air, typically ~400-420 ppm).

```yaml
# Example: Service call to trigger FRC with 415 ppm target
# Assuming the sen66 component has id: sen66_sensor
# In Home Assistant Developer Tools -> Services:
# Service: esphome.<your_node_name>_sen66_perform_forced_co2_recalibration
# Data:
#   target_co2_concentration: 415
```

```yaml
# Example: Button in ESPHome to trigger FRC with a fixed 410ppm target
button:
  - platform: template
    name: "SEN66 Calibrate CO2"
    on_press:
      - sen66.perform_forced_co2_recalibration:
          id: sen66_sensor
          target_co2_concentration: 410
```

- **sen66.perform_forced_co2_recalibration:**
  - `id` (Required, ID): The ID of the SEN66 component.
  - `target_co2_concentration` (Required, int): The target CO2 concentration in ppm to calibrate against.

### Factory Reset

Performs a factory reset. This action does the following:

1. Stops sensor measurements.
2. Clears the component's persistent data (VOC state and last save timestamp) from the ESP's non-volatile storage (NVS) using `global_preferences->reset()`.
3. Sends the hardware reset command to the SEN66 sensor itself.
4. Triggers a safe reboot of the ESPHome device.

**Warning:** This clears the learned VOC algorithm state stored on the ESP. The hardware reset also clears any learned state on the sensor itself. After the reboot, the component will re-initialize and re-apply YAML configurations, but the VOC algorithm will need to re-adapt to its environment.

```yaml
# Example: Button to trigger factory reset (Use with extreme caution!)
button:
  - platform: template
    name: "SEN66 Factory Reset"
    entity_category: "diagnostic"
    on_press:
      - sen66.factory_reset: sen66_sensor
```

- **sen66.factory_reset:** `id` (Required, ID): The ID of the SEN66 component.

## Advanced Features

### VOC State Persistence

If a `voc` sensor is configured, the component automatically saves the current state of the Sensirion VOC algorithm to the ESP's non-volatile storage (NVS) periodically (every hour by default). The timestamp of the last save attempt is also saved.

This state and timestamp are restored during startup.

This allows the VOC index algorithm to resume from its previous state more quickly after a reboot, rather than restarting the learning process from default values.

Persistence requires the `esp_idf` framework on ESP32.

## Dependencies

- **I2C Bus:** This component requires an I2C bus to be configured in your ESPHome YAML.
- **Sensirion Common:** The `sensirion_common` component is automatically loaded.
