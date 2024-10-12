
# Kegerator Control Unit

This repository contains the code for an ESP32-based Kegerator control unit. The system monitors and controls temperature inside the kegerator, manages cooling and fan systems, provides Over-the-Air (OTA) update functionality, and supports remote debugging via Telnet.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Installation](#installation)
- [Configuration](#configuration)
- [Usage](#usage)
  - [Over-the-Air (OTA) Updates](#ota-updates)
  - [Telnet Debugging](#telnet-debugging)
  - [Temperature Control](#temperature-control)
- [Pin Configuration](#pin-configuration)
- [Contributing](#contributing)
- [License](#license)

## Overview

The Kegerator Control Unit is designed to maintain the temperature of a kegerator by controlling the cooling system and fans. It is based on the ESP32 microcontroller and includes additional features like NTP time synchronization, OTA updates, and Telnet for remote diagnostics.

## Features

- **Temperature Monitoring:** Tracks the temperature inside and outside the kegerator using DS18B20 sensors.
- **Cooling Control:** Automatically controls the cooling relay based on temperature readings.
- **Fan Management:** Controls both internal and external fans.
- **Over-the-Air (OTA) Updates:** Allows for remote firmware updates.
- **Telnet Debugging:** Provides remote access for diagnostics via Telnet.
- **NTP Time Synchronization:** Automatically syncs with an NTP server to maintain accurate time.

## Hardware Requirements

- ESP32 Microcontroller
- DS18B20 temperature sensors
- Relays for controlling cooling and fan systems
- Power supply (5V/3.3V)
- Cat 5 cable for sensor and relay connections (optional)

## Software Requirements

- Arduino IDE (or PlatformIO)
- ESP32 Board Manager
- Arduino Libraries:
  - `WiFi.h` (WiFi support for ESP32)
  - `PubSubClient.h` (MQTT for communication)
  - `OneWire.h` (For temperature sensor communication)
  - `DallasTemperature.h` (For DS18B20 temperature sensors)
  - `Preferences.h` (For non-volatile memory storage)
  - `ArduinoOTA.h` (For Over-the-Air updates)

## Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/yourusername/kegerator-control-unit.git
   cd kegerator-control-unit
   ```

2. Open the project in Arduino IDE or PlatformIO.

3. Install required libraries from the Library Manager (in Arduino IDE) or `platformio.ini` (in PlatformIO).

4. Flash the firmware to your ESP32 by connecting it via USB and uploading the `Kegeretor_control_unit.ino` file.

## Configuration

1. **WiFi and MQTT Credentials:**

   Your WiFi and MQTT credentials are stored in the `secrets.h` file. Ensure that this file contains your network details:

   ```cpp
   #define WIFI_SSID "your_wifi_ssid"
   #define WIFI_PASSWORD "your_wifi_password"
   #define MQTT_SERVER "your_mqtt_server_ip"
   ```

2. **Pin Definitions:**

   Pin mappings for the ESP32 can be found in `config.h`. Review and adjust the pin definitions according to your hardware setup.

   ```cpp
   #define ONE_WIRE_BUS 15         // Pin for DS18B20 sensors
   #define COOLING_RELAY_PIN 13    // Pin for cooling relay
   #define FAN_RELAY_PIN 12        // Pin for internal fan
   ```

## Usage

### OTA Updates

The project supports OTA (Over-the-Air) updates, allowing you to upload new firmware without physically connecting the ESP32 via USB.

To use OTA:
1. Set up your code to include `setupOTA()`.
2. Once the device is running, you can upload new code via the Arduino IDE's OTA functionality.

### Telnet Debugging

The Kegerator Control Unit supports remote debugging via Telnet. By connecting to the ESP32's IP address through Telnet, you can monitor real-time status updates and send commands to control various aspects of the system.

#### Setting up Telnet

1. **Enable Telnet Server:**  
   The Telnet server is initialized in the `setup()` function with `setupTelnet()`. By default, the server runs on port 23.

2. **Connect via Telnet:**  
   You can connect to the ESP32 via Telnet using the device's IP address:

   ```bash
   telnet [ESP32_IP] 23
   ```

   Replace `[ESP32_IP]` with the actual IP address assigned to your ESP32 device.

#### Available Telnet Commands

Once connected via Telnet, you can issue the following commands to interact with the system:

| Command                 | Description                                                     |
|-------------------------|-----------------------------------------------------------------|
| `stat`                  | Displays the current status of the kegerator (temperature, relays, fans). |
| `set <value>`           | Updates the cooling setpoint temperature. Example: `set 4.5`    |
| `cool`                  | Manually trigger the cooling system.                            |
| `cooloff`               | Manually turn off the cooling system.                           |
| `reboot`                | Reboots the ESP32 device.                                       |
| `crush <temp> <time>`   | Starts a "cold crush" process for a specific temperature and duration. Example: `crush 2.0 24` |
| `help`                  | Displays a list of available commands.                          |

#### Example Telnet Session

1. **Connect to the Device**:  
   Open your terminal and connect to the ESP32 using its IP address:

   ```bash
   telnet 192.168.1.100 23
   ```

2. **Check Status**:  
   After connecting, type `stat` to see the current state of the system:

   ```
   > stat
   Inside Temp: 4.2°C
   Outside Temp: 22.3°C
   Cooling: ON
   Fan: OFF
   ```

3. **Change Cooling Setpoint**:  
   You can update the cooling setpoint using the `set` command:

   ```
   > set 3.5
   Cooling setpoint updated to 3.5°C
   ```

4. **Start Cold Crush Process**:  
   If you want to start a cold crush process, use the `crush` command with the desired temperature and duration:

   ```
   > crush 2.0 24
   Cold crush process started at 2.0°C for 24 hours
   ```

5. **Reboot the System**:  
   If you need to reboot the device, simply issue the `reboot` command:

   ```
   > reboot
   Device restarting...
   ```

6. **Disconnect from Telnet**:  
   When you're done, type `exit` to disconnect:

   ```
   > exit
   Connection closed.
   ```

#### Command Error Handling

If an invalid command is entered, the system will return an error message and display the list of available commands:

```bash
Unknown command: [command]
Available commands: stat, set <value>, cool, cooloff, crush <temp> <time>
```

### Temperature Control

The cooling system and fans are controlled based on the readings from DS18B20 temperature sensors. The logic is implemented in `control.cpp`, which handles activation and deactivation based on predefined thresholds.

- **Activate/Deactivate Cooling:**
  The cooling system is activated if the temperature exceeds the set point and deactivated when the set point is reached.

- **Fan Management:**
  Fans are activated when cooling is enabled or when specific thresholds are exceeded.

## Pin Configuration

The pin mappings for various components can be found in the `config.h` file. Below is a summary of the main pin definitions:

| Pin                | Function               |
|--------------------|------------------------|
| `ONE_WIRE_BUS`      | DS18B20 Temperature Sensors |
| `COOLING_RELAY_PIN` | Relay for cooling system |
| `FAN_RELAY_PIN`     | Relay for internal fan |
| `EXTERNAL_FAN_RELAY_PIN` | Relay for external fan |
| `DOOR_SENSOR_PIN`   | Door sensor input |

Refer to the full pin configuration in the `config.h` file for further details.

## Contributing

Contributions are welcome! Please follow these steps:
1. Fork the repository.
2. Create a new branch with a descriptive name.
3. Submit a pull request with your changes.

## License

This project is licensed under the MIT License. See the `LICENSE` file for more details.
