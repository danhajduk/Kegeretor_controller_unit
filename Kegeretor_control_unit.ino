#include "ota.h"
#include "utils.h"
#include "control.h"
#include "config.h"

void setup() {
    Serial.begin(115200);
    Serial.println("Starting setup ...");
    connectToWiFi();
    initNTP();
    Serial.println("Debug");
    setupOTA(DEVICE_NAME);
    setupTelnet();
    setupControl();  // Initialize control logic (temperature sensors, etc.)
    printToTelnet("Kegerator Control System started.");

    digitalWrite(POWER_RELAY, LOW);  // Start with power ON
    //Create a task to periodically update time from NTP
    xTaskCreatePinnedToCore(
        updateNTPTask,     // Task function
        "NTPUpdateTask",   // Task name
        4096,              // Stack size
        NULL,              // Parameter
        1,                 // Priority
        NULL,              // Task handle
        0                  // Core number
    );
}
int lastStat = 0;
void loop() {

    ArduinoOTA.handle();
    handleTelnet();
    handleControl();  // Handle temperature readings and other control logic
}
