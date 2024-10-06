#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>

void updateNTPTask(void *parameter);
String getCurrentTime();
void initNTP();

void connectToWiFi();    // Function to connect to WiFi
void setupTelnet();      // Function to initialize the Telnet server
void handleTelnet();     // Function to handle Telnet connections
void setupSensors();     // Function to initialize DS18B20 sensors

void printToTelnetErr(const String &msg);
void printToTelnet(const String &msg);
void printToTelnet(const char *msg);
void printKegeratorStatus() ;
void processTelnetCommand(const String &command);

void printDashboard();
#endif
