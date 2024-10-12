#include "ota.h"       // OTA (Over-the-Air) update functionality
#include "utils.h"     // Utility functions (WiFi, time sync, etc.)
#include "control.h"   // Control logic for sensors and relays
#include "config.h"    // Configuration file for constants and settings

// Function: setup()
// Initializes the system, sets up network connectivity, OTA updates, control logic, etc.
void setup() {
    Serial.begin(115200); // Start serial communication
    delay(100);           // Allow some time for Serial to initialize

    printToTelnet("Serial Initialized");
    printToTelnet("Starting setup ...");

    printToTelnet("Connecting to WiFi...");
    connectToWiFi();  // Connect to WiFi network
    printToTelnet("WiFi connected.");

    Serial.println("Initializing NTP...");
    initNTP();        // Initialize Network Time Protocol (NTP) for time synchronization
    printToTelnet("NTP Initialized.");

    printToTelnet("Setting up OTA...");
    setupOTA(DEVICE_NAME); // Setup OTA updates
    printToTelnet("OTA Ready.");

    printToTelnet("Setting up Telnet...");
    setupTelnet();         // Setup Telnet for remote debugging
    printToTelnet("Telnet Server Ready.");

    printToTelnet("Setting up control...");
    setupControl();        // Initialize control logic (temperature sensors, relays)
    printToTelnet("Control system initialized.");

    printToTelnet("Kegerator Control System started.");

    // Ensure the power relay is initially ON
    printToTelnet("Turning on Power Relay...");
    digitalWrite(POWER_RELAY, TURN_ON); 
    printToTelnet("Power Relay ON");

    xTaskCreatePinnedToCore(
        updateNTPTask,    // Task function
        "NTPUpdateTask",  // Task name
        4096,             // Stack size
        NULL,             // No parameters passed to the task
        1,                // Task priority
        NULL,             // Task handle
        0                 // Core number
    );
    printToTelnet("NTP Update Task created.");
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