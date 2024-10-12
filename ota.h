#ifndef OTA_H
#define OTA_H

// Include necessary libraries and sensitive credentials
#include <ArduinoOTA.h>

// Function Declarations
void setupOTA(const char* hostname);  // Function to set up OTA updates
void downloadAndUpdate(const char* url);
#endif // OTA_H
