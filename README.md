
# Kegerator Control Unit

This project is a custom Kegerator control system, which monitors temperature, fan, cooling status, and door sensor. It uses an ESP32 microcontroller and DallasTemperature sensors to manage the cooling system and fan within a kegerator. 

## Features
- Monitors inside and outside temperature using DallasTemperature sensors.
- Automatic cooling control based on temperature set points.
- Fan control based on set thresholds.
- Door sensor monitoring with automatic cooling and fan shutdown if the door is left open.
- Logging and real-time feedback through Telnet.
- Non-blocking temperature readings and task management using FreeRTOS.

## Components
1. **ESP32**: Microcontroller for managing sensors and controlling the system.
2. **DallasTemperature Sensors**: DS18B20 sensors for monitoring inside and outside kegerator temperatures.
3. **Relays**: Used to control the cooling and fan systems.
4. **Door Sensor**: Monitors the door state to control the system accordingly.

## Files and Structure
- **config.h**: Configuration file for pin definitions, sensor addresses, and threshold settings.
- **control.cpp & control.h**: Core logic for temperature monitoring, fan and cooling control, and door sensor management.
- **utils.cpp & utils.h**: Utility functions, including Telnet communications and message logging.
- **ota.cpp & ota.h**: Over-the-Air (OTA) updates functionality.
- **Kegeretor_control_unit.ino**: The main file for Arduino, orchestrating the initialization and task execution.
  
## Setup and Installation

### Hardware Setup
1. Connect the **DS18B20 temperature sensors** to the ESP32, using the OneWire protocol.
2. Use relays to control the **cooling system** and **fan**.
3. Connect the **door sensor** to monitor the state of the kegerator door.

### Software Setup
1. Clone this repository:
   ```bash
   git clone https://github.com/danhajduk/Kegeretor_control_unit.git
   ```
2. Add your WiFi credentials and temperature sensor addresses in the `secrets.h` file:
   ```cpp
    #ifndef SECRETS_H
    #define SECRETS_H
    
    // WiFi credentials
    #define WIFI_SSID <ssid>
    #define WIFI_PASSWORD <password>
    
    // MQTT settings
    #define MQTT_USER <user>
    #define MQTT_PASSWORD <password>
    
    #define OTA_PASSWORD <password>
    
    #endif

   ```
3. Upload the code to your **ESP32** using Arduino IDE or PlatformIO.

4. Set up the correct pins for your ESP32 in `config.h`:
   ```cpp
    // Pin where the data line is connected for temperature sensors
    #define ONE_WIRE_BUS 15  // Define OneWire bus pin for DS18B20 temperature sensors
    
    // Create a OneWire instance to communicate with OneWire devices
    extern OneWire oneWire;
    
    // Pass OneWire reference to DallasTemperature library
    extern DallasTemperature sensors;
    
    #define TELNET_PORT 23  // Standard Telnet port
    
    // MQTT settings
    #define MQTT_SERVER <mwtt server>
    #define MQTT_PORT 1883
    #define MQTT_MAX_PACKET_SIZE 1024
    
    // MQTT discovery topic base
    #define DISCOVERY_PREFIX "homeassistant"
    #define DEVICE_NAME "kegerator"
    #define DEVICE_FRIENDLY_NAME "Kegerator"
    
    // Cooling system settings
    #define COOLING_RELAY_PIN 13  
    #define COOLING_THRESHOLD 5.0  // Default cooling threshold (Celsius)
    #define COOLING_HYSTERESIS 0.5  // Hysteresis for cooling
    
    // Fan system settings
    #define FAN_RELAY_PIN 12  
    #define FAN_THRESHOLD 2.0  // Default fan threshold (Celsius)
    #define FAN_HYSTERESIS 1.0  // Hysteresis for fan control
    
    // External fan system settings
    #define EXTERNAL_FAN_RELAY_PIN 14  
    #define EXTERNAL_FAN_THRESHOLD 27.0  // Default threshold for external fan (Celsius)
    
    // Device information for Home Assistant MQTT Discovery
    #define DEVICE_IDENTIFIERS <your_ID>
    #define DEVICE_MODEL "Kegerator Controller ESP32"
    #define DEVICE_MANUFACTURER "Dan Hajduk"
    
    // Other pins
    #define DOOR_SENSOR_PIN 5
    #define POWER_RELAY 27  
    
    #endif  // CONFIG_H

   ```

### Running the System
1. Once uploaded, the ESP32 will initialize and start monitoring the temperatures, controlling the fan, and the cooling system.
2. Connect to the ESP32 via **Telnet** to monitor real-time logs and change settings (e.g., temperature set point).

### OTA (Over-the-Air) Updates
You can perform OTA updates to the ESP32 using the defined OTA functionality in `ota.cpp` and `ota.h`.

## Telnet Commands
- **status**: Shows the current status of the system.
- **setpoint <value>**: Sets a new cooling set point.
- **cool**: Manually activate the cooling system.
  
## License
This project is licensed under the MIT License.
