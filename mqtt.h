#ifndef MQTT_H
#define MQTT_H

#include <WiFi.h>
#include <PubSubClient.h>

// Declare the external variables defined in control.cpp that are used in mqtt.cpp
extern float insideTemp;
extern float outsideTemp;
extern bool coolingOn;
extern bool doorOpen;
extern bool fanOn;        // Use fanOn instead of internal/external fan
extern float avgTemp;
extern float coolingSetPoint;

// Callback function for handling incoming MQTT messages
void mqttCallback(char* topic, byte* payload, unsigned int length);

// Sets up the MQTT connection and subscribes to topics
void setupMQTT();

// Publishes the MQTT auto-discovery configuration for Home Assistant
void publishDiscoveryConfig();

// Reconnects to the MQTT broker in case of disconnection
void reconnect();

// Sends sensor data (temperature, cooling state, door status, fan statuses, etc.) to the MQTT broker
void sendSensorData(float insideTemp, float outsideTemp, bool coolingOn);

// Handles the MQTT loop, ensuring the connection stays alive and periodically publishes sensor data
void handleMQTT();

#endif // MQTT_H
