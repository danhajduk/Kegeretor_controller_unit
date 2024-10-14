#ifndef OTA_H
#define OTA_H

// Include necessary libraries and sensitive credentials
#include <ArduinoOTA.h>

// External variables declerations
extern String newVersion;

// Function Declarations
void setupOTA(const char* hostname);  // Function to set up OTA updates
void downloadAndUpdate();
void checkForUpdates();
#endif // OTA_H
