#ifndef CONFIG_H
#define CONFIG_H

#include "secrets.h"  // Ensure secrets.h is available for sensitive credentials
#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Preferences.h>  // For using Preferences library

// Uncomment to enable debugging mode
#define DEBUG_MODE

// External declarations
extern WiFiClient espClient;
extern float coolingSetPoint;
// extern float externalFanThreshold;
extern Preferences preferences;  
extern bool restartForOTA;

// Pin where the data line is connected for temperature sensors
#define ONE_WIRE_BUS 15  // Define OneWire bus pin for DS18B20 temperature sensors

// Create a OneWire instance to communicate with OneWire devices
extern OneWire oneWire;

// Pass OneWire reference to DallasTemperature library
extern DallasTemperature sensors;

#define TELNET_PORT 23  // Standard Telnet port

// MQTT settings
#define MQTT_SERVER "10.0.0.100"
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
#define DEVICE_IDENTIFIERS "kegerator_id"
#define DEVICE_MODEL "Kegerator Controller ESP32"
#define DEVICE_MANUFACTURER "Dan Hajduk"

// Other pins
#define DOOR_SENSOR_PIN 5
#define POWER_RELAY 27  

// External temperature sensor addresses (inside and outside the Kegerator)
extern uint8_t sensor1Address[8];  // Inside Kegerator
extern uint8_t sensor2Address[8];  // Outside Kegerator

// External control state flags
extern bool coolingOn;  
extern bool fanOn;      
extern bool internalFanOn;
extern bool externalFanOn;

// Additional external declarations for networking
extern WiFiServer telnetServer;
extern WiFiClient telnetClient;

// External timing variables
extern unsigned long lastCoolingOffTime;
extern unsigned long coolingDelay;

// External variables for time tracking
extern String currentDate;
extern String currentTimeHHMM;
extern int totalReadings;

// External declarations
extern WiFiClient espClient;

// Pin where the data line is connected for temperature sensors
#define ONE_WIRE_BUS 15

// Create a OneWire instance to communicate with OneWire devices
extern OneWire oneWire;

// Pass OneWire reference to DallasTemperature library
extern DallasTemperature sensors;

// Global variables to store temperature readings
extern float insideTemp;   // Temperature inside the Kegerator
extern float outsideTemp;  // Temperature outside, behind the Kegerator

// Sensor addresses (inside and outside the Kegerator)
extern uint8_t sensor1Address[8];
extern uint8_t sensor2Address[8];


#endif  // CONFIG_H


// Cat 5 Cable configp 
// pin #   |  Pin Name        Color main    ESP Pin
//   1     |  5V            | RED           NA
//   2     |  3V3           | ORANGE        3V3
//   3     |  Temp Data     | YELLOW        D15
//   4     |  Cooling Relay | BLUE          D13         
//   5     |  Fans          | GREEN         
//   6     |  Door sensor   | PURPLE        D5
//   7     |  NC            | WHITE
//   8     |  GND           | BLACK         NA