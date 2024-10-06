#include "ota.h"
#include "secrets.h"
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "utils.h"
#include <Arduino.h>  // Ensure Arduino types like String are available

/**
 * @brief Initializes the OTA (Over-the-Air) update service.
 * 
 * This function sets up the OTA service with a given hostname and password. 
 * It defines various event handlers to manage the OTA process, including 
 * start, progress, error handling, and completion of the update.
 * 
 * @param hostname The desired hostname for the OTA service.
 */
void setupOTA(const char* hostname) {
    // Set the hostname for OTA updates
    ArduinoOTA.setHostname(hostname);

    // Set OTA password from secrets.h for authentication
    ArduinoOTA.setPassword(OTA_PASSWORD);

    // Define the OTA event handlers
    ArduinoOTA.onStart([]() {
        // Identify the type of update: sketch or filesystem
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
        printToTelnet("Updating " + type + " now, Please standby....");
    });

    ArduinoOTA.onEnd([]() {
        // Print end message to telnet upon OTA completion
        printToTelnet("\nEnd");
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        // Display the progress of the OTA update
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });

    ArduinoOTA.onError([](ota_error_t error) {
        // Handle OTA errors and print corresponding messages
        printToTelnetErr(String(error));
        switch (error) {
            case OTA_AUTH_ERROR:
                printToTelnet("Auth Failed");
                break;
            case OTA_BEGIN_ERROR:
                printToTelnet("Begin Failed");
                break;
            case OTA_CONNECT_ERROR:
                printToTelnet("Connect Failed");
                break;
            case OTA_RECEIVE_ERROR:
                printToTelnet("Receive Failed");
                break;
            case OTA_END_ERROR:
                printToTelnet("End Failed");
                break;
        }
    });

    // Start the OTA service
    ArduinoOTA.begin();
    printToTelnet("OTA Ready");
}
