#include "ota.h"
#include "secrets.h"
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <HTTPClient.h>
#include <Update.h>
#include "utils.h"
#include <Arduino.h>  // Ensure Arduino types like String are available
#include "config.h"

const char* firmwareVersion = VERSION;  // Current firmware version
// URL tox version file
const char* versionURL = "https://raw.githubusercontent.com/danhajduk/Kegeretor_controller_unit/refs/heads/main/version.txt";  
const char* firmwareURL = "https://github.com/your-repo/releases/download/v1.1/firmware.bin";  // URL to binary file

/**
 * @brief Initializes the OTA (Over-the-Air) update service.
 * 
 * This function sets up the OTA service with a given hostname and password.
 * It defines various event handlers to manage the OTA process.
 * 
 * @param hostname The desired hostname for the OTA service.
 */
void setupOTA(const char* hostname) {
    ArduinoOTA.setHostname(hostname);
    ArduinoOTA.setPassword(OTA_PASSWORD);

    ArduinoOTA.onStart([]() {
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
        printToTelnet("Updating " + type + " now, Please standby....");
    });

    ArduinoOTA.onEnd([]() {
        printToTelnet("\nEnd");
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });

    ArduinoOTA.onError([](ota_error_t error) {
        printToTelnetErr(String(error));
        switch (error) {
            case OTA_AUTH_ERROR: printToTelnet("Auth Failed"); break;
            case OTA_BEGIN_ERROR: printToTelnet("Begin Failed"); break;
            case OTA_CONNECT_ERROR: printToTelnet("Connect Failed"); break;
            case OTA_RECEIVE_ERROR: printToTelnet("Receive Failed"); break;
            case OTA_END_ERROR: printToTelnet("End Failed"); break;
        }
    });

    ArduinoOTA.begin();
    printToTelnet("OTA Ready");
}

/**
 * @brief Check for a new firmware version by comparing the current version with the hosted version.
 */
void checkForUpdates() {
    HTTPClient http;

    // Check the version file
    http.begin(versionURL);
    int httpCode = http.GET();

    if (httpCode == 200) {
        String newVersion = http.getString();
        newVersion.trim();  // Remove any extra whitespace

        // If a new version is available, start the update
        if (newVersion != firmwareVersion) {
            printToTelnet("New firmware version detected: " + newVersion);
        } else {
            printToTelnet("Firmware is up to date.");
        }
    } else {
        printToTelnet("Error checking firmware version: " + String(httpCode));
    }

    http.end();
}

/**
 * @brief Download and perform OTA update with the new firmware binary.
 * 
 * @param url The URL to the new firmware binary.
 */
void downloadAndUpdate(const char* url) {
    WiFiClient client;
    HTTPClient http;

    printToTelnet("Starting OTA update from: " + String(url));

    // Connect to the server hosting the firmware binary
    http.begin(client, url);
    int httpCode = http.GET();

    if (httpCode == 200) {
        int contentLength = http.getSize();
        if (contentLength > 0) {
            if (Update.begin(contentLength)) {
                printToTelnet("Downloading firmware...");
                WiFiClient* stream = http.getStreamPtr();
                size_t written = Update.writeStream(*stream);

                if (written == contentLength) {
                    printToTelnet("Firmware downloaded successfully.");
                    if (Update.end()) {
                        printToTelnet("OTA update complete.");
                        if (Update.isFinished()) {
                            printToTelnet("Rebooting...");
                            ESP.restart();  // Reboot to apply the new firmware
                        } else {
                            printToTelnet("OTA update failed.");
                        }
                    } else {
                        printToTelnet("Error during OTA update: " + String(Update.getError()));
                    }
                } else {
                    printToTelnet("Failed to write all the bytes.");
                }
            } else {
                printToTelnet("Not enough space for OTA update.");
            }
        } else {
            printToTelnet("No content in the firmware file.");
        }
    } else {
        printToTelnet("Failed to download firmware: " + String(httpCode));
    }

    http.end();
}
