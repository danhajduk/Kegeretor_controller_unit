#include "ota.h"       // OTA (Over-the-Air) update functionality
#include "utils.h"     // Utility functions (WiFi, time sync, etc.)
#include "control.h"   // Control logic for sensors and relays
#include "config.h"    // Configuration file for constants and settings

// Function: setup()
// Initializes the system, sets up network connectivity, OTA updates, control logic, etc.
void setup() {
    Serial.begin(115200); // Start serial communication at 115200 baud rate for debugging
    printToTelnet("Starting setup ...");

    connectToWiFi();  // Connect to WiFi network
    initNTP();        // Initialize Network Time Protocol (NTP) for time synchronization

    setupOTA(DEVICE_NAME); // Setup OTA updates (useful for remote firmware updates)
    setupTelnet();         // Setup Telnet for remote debugging and monitoring
    setupControl();        // Initialize control logic (e.g., temperature sensors, relays)
    
    printToTelnet("Kegerator Control System started."); // Log message to Telnet

    // Ensure the power relay is initially ON (LOW may represent ON state)
    digitalWrite(POWER_RELAY, TURN_ON); 

    // Create a FreeRTOS task to periodically update time from NTP server
    // This task runs on core 0 of the ESP32.
    xTaskCreatePinnedToCore(
        updateNTPTask,    // Task function that updates NTP
        "NTPUpdateTask",  // Task name (for debugging purposes)
        4096,             // Stack size for the task (4 KB)
        NULL,             // No parameters are passed to the task
        1,                // Task priority (1 is a normal priority)
        NULL,             // Task handle (not required here)
        0                 // Core number (0 means run on the first core)
    );
}

// Global variable to track last status
int lastStat = 0;

// Function: loop()
// Continuously runs the main control logic, handles OTA, Telnet, and sensors.
void loop() {
    ArduinoOTA.handle();  // Handle Over-the-Air updates
    handleTelnet();       // Process Telnet commands for remote debugging
    handleControl();      // Main control loop for managing temperature, relays, etc.
}