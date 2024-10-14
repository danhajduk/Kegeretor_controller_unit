#ifndef CONFIG_H
#define CONFIG_H

// Include necessary libraries and sensitive credentials
#include "secrets.h"      // Contains WiFi and MQTT credentials
#include <WiFi.h>         // WiFi library for ESP32
#include <PubSubClient.h> // MQTT client library
#include <OneWire.h>      // OneWire protocol for temperature sensors
#include <DallasTemperature.h> // Library for handling Dallas temperature sensors
#include <Preferences.h>  // Preferences library for non-volatile storage

#define VERSION "0.1.1"

// Uncomment the following line to enable debugging mode
#define DEBUG_MODE

// External declarations (variables shared across multiple files)
extern WiFiClient espClient;        // WiFi client object
extern float coolingSetPoint;       // Cooling setpoint for temperature control
extern Preferences preferences;     // Preferences object for saving settings
extern bool restartForOTA;          // Flag to signal OTA update restart

// Pin Definitions
#define ONE_WIRE_BUS 15             // Pin for OneWire data (DS18B20 temperature sensors)
#define COOLING_RELAY_PIN 13        // Pin for controlling cooling relay
#define FAN_RELAY_PIN 12            // Pin for internal fan relay
#define EXTERNAL_FAN_RELAY_PIN 14   // Pin for external fan relay
#define DOOR_SENSOR_PIN 5           // Pin for door sensor (detects open/close)
#define POWER_RELAY 27              // Pin to control main power relay

// OneWire and Temperature Sensor Setup
extern OneWire oneWire;             // OneWire instance for communicating with DS18B20 sensors
extern DallasTemperature sensors;   // DallasTemperature object for managing temperature sensors

// Telnet and Network Settings
#define TELNET_PORT 23              // Standard Telnet port for remote monitoring
extern WiFiServer telnetServer;     // Telnet server object
extern WiFiClient telnetClient;     // Telnet client object

// MQTT Settings
#define MQTT_SERVER "10.0.0.100"    // MQTT broker IP address
#define MQTT_PORT 1883              // MQTT port (1883 is the default)
#define MQTT_MAX_PACKET_SIZE 1024   // Maximum packet size for MQTT messages

// MQTT Discovery Settings for Home Assistant
#define DISCOVERY_PREFIX "homeassistant" // Discovery topic base
#define DEVICE_NAME "kegerator"          // Device name used in MQTT topics
#define DEVICE_FRIENDLY_NAME "Kegerator" // Friendly device name

// Device Info for Home Assistant MQTT Discovery
#define DEVICE_IDENTIFIERS "kegerator_id"
#define DEVICE_MODEL "Kegerator Controller ESP32"
#define DEVICE_MANUFACTURER "Dan Hajduk"

// Cooling System Configuration
#define COOLING_THRESHOLD 5.0       // Temperature threshold for cooling (in Celsius)
#define COOLING_HYSTERESIS 0.5      // Hysteresis for cooling control (prevents rapid toggling)
#define TURN_ON 0
#define TURN_OFF 1

// Fan System Configuration
#define FAN_THRESHOLD 2.0           // Temperature threshold for fan control (in Celsius)
#define FAN_HYSTERESIS 1.0          // Hysteresis for fan control
  
// External Fan System Configuration
#define EXTERNAL_FAN_THRESHOLD 27.0 // Threshold for external fan (in Celsius)

// Sensor Addresses for DS18B20 (inside and outside Kegerator)
extern uint8_t sensor1Address[8];   // Address of the temperature sensor inside the Kegerator
extern uint8_t sensor2Address[8];   // Address of the temperature sensor outside the Kegerator

// Global Variables for System State
extern bool coolingOn;              // Flag to indicate if cooling is active
extern bool fanOn;                  // Flag to indicate if the fan is active
extern bool internalFanOn;          // Flag for internal fan state
extern bool externalFanOn;          // Flag for external fan state

// Timing Variables
extern unsigned long lastCoolingOffTime; // Stores the last time cooling was turned off
extern unsigned long coolingDelay;       // Minimum delay before re-enabling cooling

// Time Variables for NTP
extern String currentDate;          // Current date (updated via NTP)
extern String currentTimeHHMM;      // Current time in HH:MM format (from NTP)
extern int totalReadings;           // Total number of temperature readings taken

// Temperature Readings
extern float insideTemp;            // Current temperature inside the Kegerator
extern float outsideTemp;           // Current temperature outside (behind Kegerator)

#endif // CONFIG_H

// --- Cat 5 Cable Configuration ---
// This section outlines how to connect sensors and components using a Cat 5 cable
// Pin Number | Function           | Color   | ESP32 Pin
// ----------------------------------------------------
// 1          | 5V Power           | RED     | NA (Not used by ESP32)
// 2          | 3V3 Power          | ORANGE  | 3V3 (ESP32 power)
// 3          | Temp Data          | YELLOW  | D15 (ONE_WIRE_BUS pin for temperature sensors)
// 4          | Cooling Relay       | BLUE    | D13 (COOLING_RELAY_PIN)
// 5          | Fan Control        | GREEN   | (Fan pin not connected in diagram)
// 6          | Door Sensor        | PURPLE  | D5 (DOOR_SENSOR_PIN)
// 7          | Not Connected (NC) | WHITE   | NA
// 8          | Ground (GND)       | BLACK   | NA (Ground pin)