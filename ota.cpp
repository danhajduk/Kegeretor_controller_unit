#include "ota.h"
#include "secrets.h"
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "utils.h"
#include <Arduino.h>  // Ensure Arduino types like String are available

void setupOTA(const char* hostname) {
    // Set the hostname for the OTA
    ArduinoOTA.setHostname(hostname);

    // Set OTA password from secrets.h
    ArduinoOTA.setPassword(OTA_PASSWORD);

    // Define the OTA event handlers
    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else {  // U_SPIFFS
            type = "filesystem";
        }
        printToTelnet("Updating " + type + " now, Please standby....");
    });

    ArduinoOTA.onEnd([]() {
        printToTelnet("\nEnd");
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });

    ArduinoOTA.onError([](ota_error_t error) {
        printToTelnetErr(String (error));
        if (error == OTA_AUTH_ERROR) {
            printToTelnet("Auth Failed");
        } else if (error == OTA_BEGIN_ERROR) {
            printToTelnet("Begin Failed");
        } else if (error == OTA_CONNECT_ERROR) {
            printToTelnet("Connect Failed");
        } else if (error == OTA_RECEIVE_ERROR) {
            printToTelnet("Receive Failed");
        } else if (error == OTA_END_ERROR) {
            printToTelnet("End Failed");
        }
    });

    // Start OTA service
    ArduinoOTA.begin();
    printToTelnet("OTA Ready");
}
