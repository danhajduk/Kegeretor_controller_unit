#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>

// NTP Functions
void updateNTPTask(void *parameter);   // Function to update time via NTP
String getCurrentTime();               // Function to get the current time as a string
void initNTP();                        // Function to initialize NTP

// WiFi and Telnet Functions
void connectToWiFi();                  // Function to connect to WiFi
void setupTelnet();                    // Function to initialize the Telnet server
void handleTelnet();                   // Function to handle Telnet connections

// Sensor Functions
void setupSensors();                   // Function to initialize DS18B20 sensors

// Telnet Messaging Functions
void printToTelnetFormatted(const String& msg, const String& color = "white", int row = 1, int column = 1, bool blink = false, bool underline = false);
void printToTelnetErr(const String &msg);   // Function to print an error message to Telnet
void printToTelnet(const String &msg);      // Function to print a message to Telnet (String)
void printToTelnet(const char *msg);        // Function to print a message to Telnet (char*)
void printKegeratorStatus();                // Function to print the Kegerator's current status
void processTelnetCommand(const String &command); // Function to process Telnet commands
bool isTelnetAvailable();                   // Function checks if the Telnet client is available and connected.
// Dashboard Functions
void printDashboard();                 // Function to print the dashboard to Telnet

String formatTime(unsigned long seconds); 

#endif // UTILS_H
