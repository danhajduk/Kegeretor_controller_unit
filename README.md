
# Kegerator Control Unit

This project is a custom **Kegerator Control System**, designed to manage temperature, cooling, fan, and door monitoring for a kegerator. Built on an **ESP32 microcontroller**, it uses **DallasTemperature sensors** to automate the cooling process and log real-time data via **Telnet**. The system is also capable of **Over-the-Air (OTA)** updates for remote firmware management.

## Features
- **Temperature Monitoring:** Tracks both inside and outside kegerator temperatures using DS18B20 sensors.
- **Automated Cooling Control:** Dynamically adjusts cooling based on set temperature points.
- **Fan Control:** Automatically activates based on temperature thresholds.
- **Door Monitoring:** Detects door status and shuts down cooling/fan systems if the door is left open.
- **Telnet Logging:** Provides real-time feedback and control through a Telnet interface.
- **Non-blocking Temperature Readings:** Employs **FreeRTOS** for task management and non-blocking I/O.
- **OTA Updates:** Supports Over-the-Air updates to upgrade firmware without manual intervention.

## Components
1. **ESP32**: The microcontroller that manages sensors and controls the cooling/fan system.
2. **DallasTemperature Sensors**: DS18B20 sensors that monitor the inside and outside temperatures of the kegerator.
3. **Relays**: Used to control the cooling and fan systems based on temperature and door status.
4. **Door Sensor**: Monitors the door's state to prevent unnecessary cooling/fan operation if left open.

## Files and Structure
- **config.h**: Configuration file for pin definitions, sensor addresses, and system thresholds.
- **control.cpp & control.h**: Core logic for temperature monitoring, cooling/fan control, and door management.
- **utils.cpp & utils.h**: Utility functions for Telnet communication and logging.
- **ota.cpp & ota.h**: Implements Over-the-Air (OTA) updates functionality.
- **Kegeretor_control_unit.ino**: The main Arduino sketch that initializes and orchestrates all system components.

## Setup and Installation

### Hardware Setup
1. Connect the **DS18B20 temperature sensors** to the ESP32 using the OneWire protocol.
2. Wire relays to control the **cooling system** and **fan**.
3. Attach a **door sensor** to monitor the state of the kegerator door.

### Software Setup
1. Clone this repository:
   ```bash
   git clone https://github.com/danhajduk/Kegeretor_control_unit.git
   ```

2. Add your WiFi credentials and sensor addresses in the `secrets.h` file:
   ```cpp
   #ifndef SECRETS_H
   #define SECRETS_H
   
   // WiFi credentials
   #define WIFI_SSID "<your-ssid>"
   #define WIFI_PASSWORD "<your-password>"
   
   // OTA settings
   #define OTA_PASSWORD "<your-ota-password>"
   
   #endif
   ```

3. Modify pin settings in `config.h` according to your hardware setup:
   ```cpp
   // Pin definitions
   #define ONE_WIRE_BUS 15  // Pin for DS18B20 temperature sensors
   #define COOLING_RELAY_PIN 13  // Pin for cooling system relay
   #define FAN_RELAY_PIN 12  // Pin for fan system relay
   #define DOOR_SENSOR_PIN 5  // Pin for door sensor
   ```

4. Upload the code to your **ESP32** using **Arduino IDE** or **PlatformIO**.

### Running the System
1. After uploading, the ESP32 will start monitoring the temperatures, controlling the fan, and managing the cooling system.
2. Connect to the ESP32 via **Telnet** to view real-time logs and adjust settings (e.g., temperature set point).

### OTA (Over-the-Air) Updates
You can update the ESP32 firmware remotely using OTA functionality defined in `ota.cpp` and `ota.h`.

## Telnet Commands
- **status**: Displays the current system status.
- **setpoint <value>**: Adjusts the cooling set point to a new value.
- **cool**: Manually activates the cooling system.

## License
This project is licensed under the MIT License.
