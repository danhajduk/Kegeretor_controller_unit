#include "utils.h"
#include "config.h"
#include "control.h" 
#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFiUdp.h>
#include <time.h>

WiFiServer telnetServer(TELNET_PORT);  // Create a Telnet server on the defined port
WiFiClient telnetClient;  // Create a Telnet client


const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -28800;  // Offset for Pacific Standard Time (UTC -8 hours)
const int daylightOffset_sec = 3600;  // Change for daylight savings if applicable

unsigned long lastNtpUpdateTime = 0;  // Track the last time the NTP update was performed
unsigned long ntpUpdateInterval = 300000;  // 5 minutes (in milliseconds)

// Function to initialize time from NTP server
void initNTP() {
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    printToTelnet("NTP initialized.");
}

// Function to get the current time
String getCurrentTime() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        printToTelnetErr("Failed to obtain time");
        return "N/A";
    }
    char timeStringBuff[50];
    strftime(timeStringBuff, sizeof(timeStringBuff), "%m-%d-%Y %H:%M:%S", &timeinfo);
    return String(timeStringBuff);  // Return current time as a string
}

// Task to update NTP time every 5 minutes
void updateNTPTask(void *parameter) {
    while (true) {
        unsigned long currentMillis = millis();

        // If 5 minutes have passed, update the time from NTP
        if (currentMillis - lastNtpUpdateTime >= ntpUpdateInterval) {
            printToTelnet("Updating time from NTP...");
            initNTP();
            lastNtpUpdateTime = currentMillis;
        }

        // Delay task for 1 second to keep things responsive
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// Circular buffer for storing debug messages
String debugMSG[14];
int debugIndex = 0;  // Circular buffer index

void setupSensors() {
    sensors.begin();  // Start the DS18B20 temperature sensors

    // Check if the sensors are connected properly
    if (!sensors.getAddress(sensor1Address, 0)) {
        printToTelnetErr("Could not find sensor 1 inside the Kegerator.");
    } else {
        printToTelnet("Sensor 1 (inside) found and initialized.");
    }

    if (!sensors.getAddress(sensor2Address, 1)) {
        printToTelnetErr("Could not find sensor 2 outside the Kegerator.");
    } else {
        printToTelnet("Sensor 2 (outside) found and initialized.");
    }
}

void connectToWiFi() {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);  // Start WiFi connection

    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    // printToTelnet("\nWiFi connected");
    // printToTelnet("IP Address: " + String(WiFi.localIP()));
}

// Setup the Telnet server
void setupTelnet() {
    telnetServer.begin();  // Start the Telnet server
    telnetServer.setNoDelay(true);  // Ensure data is sent without delay
    printToTelnet("Telnet server started on port " + String(TELNET_PORT));
}

void handleTelnet() {
    static String commandBuffer = "";  // Buffer to store incoming commands

    // Accept new client
    if (telnetServer.hasClient()) {
        if (!telnetClient || !telnetClient.connected()) {
            if (telnetClient) telnetClient.stop();  // Disconnect old client
            telnetClient = telnetServer.available();  // Assign new client
            telnetClient.println("New Telnet client connected");
            telnetClient.println("Welcome to the Kegerator Control Telnet!");
            printKegeratorStatus();  // Print the status upon connection
        } else {
            telnetServer.available().stop();  // Reject new connection if client is already connected
        }
    }

    // Handle incoming data from Telnet client
    if (telnetClient && telnetClient.connected()) {
        while (telnetClient.available()) {
            char c = telnetClient.read();
            Serial.write(c);  // Echo Telnet input to Serial

            // Add received characters to the command buffer
            if (c == '\n' || c == '\r') {
                processTelnetCommand(commandBuffer);  // Process the command
                commandBuffer = "";  // Clear buffer after processing
            } else {
                commandBuffer += c;  // Accumulate the command characters
            }
        }
    }
}

void processTelnetCommand(const String &command) {
    String trimmedCommand = command;
    trimmedCommand.trim();  // Remove leading and trailing whitespace

    if (trimmedCommand.equalsIgnoreCase("stat")) {
        printDashboard();  // Display the dashboard
    } 
    else if (trimmedCommand.startsWith("set ")) {
        String valueStr = trimmedCommand.substring(4);  // Extract value
        float newSetPoint = valueStr.toFloat();
        if (newSetPoint > 0) {
            updateCoolingSetPoint(newSetPoint);  // Update set point
            printToTelnet("Set point updated to: " + String(newSetPoint) + " °C");
        } else {
            printToTelnetErr("Invalid set point value.");
        }
    }
    else if (trimmedCommand.equalsIgnoreCase("cool")) {
        // Manually trigger the cooling system
        if (!coolingOn) {
            digitalWrite(COOLING_RELAY_PIN, LOW);  // Activate cooling
            coolingOn = true;
            printToTelnet("Cooling system manually activated.");
        } else {
            printToTelnet("Cooling is already ON.");
        }
    }
    else {
        // Unknown command
        printToTelnetErr("Unknown command: " + trimmedCommand);
        printToTelnet("Available commands: stat, set <value>, cool");
    }
}

void printToTelnetErr(const String &msg) {
    String errMsg = "\033[31mERROR : \033[0m" + msg;
    debugMSG[debugIndex] = getCurrentTime() + "> " + errMsg;
    Serial.println(debugMSG[debugIndex]);
    // telnetClient.println(debugMSG[debugIndex]);
    debugIndex = (debugIndex + 1) % 10;  // Update debugIndex, looping it within the array bounds
    printDashboard();
}

void printToTelnet(const String &msg) {
    debugMSG[debugIndex] = getCurrentTime() + "> " + msg;
    Serial.println(debugMSG[debugIndex]);
    // telnetClient.println(debugMSG[debugIndex]);
    debugIndex = (debugIndex + 1) % 10;  // Update debugIndex, looping it within the array bounds
    printDashboard();
}

void printToTelnet(const char* msg) {
    debugMSG[debugIndex] = getCurrentTime() + "> " + String(msg);
    Serial.println(debugMSG[debugIndex]);
    // telnetClient.println(debugMSG[debugIndex]);
    debugIndex = (debugIndex + 1) % 10;  // Update debugIndex, looping it within the array bounds
    printDashboard();
}

void printKegeratorStatus() {
    printDashboard();  // Call the function to print the dashboard
}

void printDashboard() {
    // Clear the screen
    telnetClient.print("\033[2J");

    // Move cursor to top left (home)
    telnetClient.print("\033[H");

    // Print header with color
    telnetClient.print("\033[1;1H=========================================== Kegerator Dashboard ===============================================\n");
    if (insideTemp == DEVICE_DISCONNECTED_C) telnetClient.print("\033[2;1HInside Temperature : \033[33m N/A \033[0m\n");
    else telnetClient.print("\033[2;1HInside Temperature : \033[33m" + String(insideTemp) + " °C\033[0m\n");
    if (outsideTemp == DEVICE_DISCONNECTED_C) telnetClient.print("\033[3;1HOutside Temperature: \033[33m N/A \033[0m\n");
    else telnetClient.print("\033[3;1HInside Temperature: \033[33m" + String(outsideTemp) + " °C\033[0m\n");
    telnetClient.print("\033[4;1HCooling Set Point  : \033[36m" + String(coolingSetPoint) + " °C\033[0m\n");
    telnetClient.print("\033[2;45HAvg. Temp.  :   \033[33m" + String(calculateAverageTemp()) + " °C\033[0m\n");
    
    String Stat;
    // Display cooling status
    Stat = coolingOn ? "\033[32mON\033[0m" : "\033[31mOFF\033[0m";
    telnetClient.print("\033[5;1HCooling Status :" + Stat + "\n");
    // Disply fan status
    Stat = fanOn ? "\033[32mON\033[0m" : "\033[31mOFF\033[0m";
    telnetClient.print("\033[4;45HExternal Fan:   " + Stat + "\n");
    Stat = fanOn ? "\033[32mON\033[0m" : "\033[31mOFF\033[0m";
    telnetClient.print("\033[5;45HInternal Fan:   " + Stat + "\n");
    // Dsiplay door status
    Stat = doorOpen ? "\033[31mOpen\033[0m" : "\033[32mClosed\033[0m";
    telnetClient.print("\033[3;45HDoor status :   " + Stat + "\n");

    Stat = getCurrentTime();
    Stat = Stat.substring(Stat.length() - 8, Stat.length() - 3);
    telnetClient.print("\033[3;85HTime    :   " +  Stat + "\n");

    Stat = getCurrentTime();
    Stat = Stat.substring(0, 10);
    telnetClient.print("\033[2;85HDate    :   " +  Stat + "\n");

    Stat = coolingOnDuration;
    telnetClient.print("\033[4;85HAvg. On :   " +  Stat + "\n");

    Stat = coolingOffDuration;
    telnetClient.print("\033[5;85HAvg. Off:   " +  Stat + "\n");

    telnetClient.print("\033[6;1H===============================================================================================================\n");
    telnetClient.print("\033[9;1H===============================================================================================================\n");

    // Time since last cooling
        if (!coolingOn) telnetClient.print("\033[8;1HTime Since Last Cooling Off: " + String((millis() - lastCoolingOffTime) / 1000) + " seconds\n");
        if (coolingOn) telnetClient.print("\033[8;1HTime Since Last Cooling On : " + String((millis() - lastCoolingOnTime) / 1000) + " seconds\n");

    // Display last 10 debug messages from the circular buffer
    for (int i = 0; i < 14; i++) {
        int msgIndex = (debugIndex + i) % 10;  // Get message index from the buffer
        telnetClient.print("\033[" + String(10 + i) + ";1H" + debugMSG[msgIndex] + "\n");
    }

    // Reset text formatting
    telnetClient.print("\033[0m");
}
