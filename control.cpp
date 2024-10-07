#include "control.h"
#include "config.h"
#include "utils.h"
#include <Arduino.h>

// Create a OneWire instance for temperature sensors
OneWire oneWire(ONE_WIRE_BUS);

// Pass the OneWire instance to DallasTemperature library
DallasTemperature sensors(&oneWire);

// Global variables for storing temperature readings
float insideTemp = 0.0;
float outsideTemp = 0.0;

// Sensor addresses
uint8_t sensor1Address[8] = {0x28, 0xCC, 0x3F, 0x6B, 0x00, 0x00, 0x00, 0x13};  // Inside Kegerator
uint8_t sensor2Address[8] = {0x28, 0x77, 0xFA, 0x6A, 0x00, 0x00, 0x00, 0x93};  // Outside, Behind Kegerator

// Task handler
TaskHandle_t temperatureTaskHandle;

// Global variables for cooling control
bool coolingOn = false;
float coolingSetPoint = 4.0;  // Default set point for cooling
unsigned long lastCoolingOffTime = 0;  // Time when cooling was last turned off
unsigned long lastCoolingOnTime = 0;   // Time when cooling was last turned on
unsigned long coolingDelay = 180000;  // Define cooling delay as 3 minutes (180000 milliseconds)

// Preferences object
Preferences preferences;

// Global variables for door monitoring
bool doorOpen = false;
unsigned long doorOpenedTime = 0;  // Time when the door was opened
const unsigned long doorOpenCoolingThreshold = 60000;  // 60 seconds (for cooling)
const unsigned long doorOpenFanThreshold = 30000;  // 30 seconds (for fan)

// Global variables for tracking on/off durations
unsigned long coolingOnDuration = 0;   // How long the cooling was ON
unsigned long coolingOffDuration = 0;  // How long the cooling was OFF

// Global variable to track fan status
bool fanOn = false;

const int bufferSize = 120;  // Buffer to store 60 temperature readings (1 per minute for an hour)
float insideTempBuffer[bufferSize];  // Array to store the inside temperature readings
int bufferIndex = 0;  // Tracks the current position in the buffer
bool bufferFilled = false;  // Tracks if the buffer has been filled with 60 values yet

/**
 * @brief Main control function that handles cooling, fan, and door monitoring.
 * 
 * This function is called repeatedly in the main loop. It manages the cooling, fan, 
 * and door monitoring functionalities by calling the respective functions.
 */
void handleControl() {
    cooling();
    controlFan();
    monitorDoor();
}

/**
 * @brief Initializes the door sensor.
 * 
 * This function sets up the door sensor, configuring the pin mode 
 * and reading the initial door state. The door sensor uses an input pull-up resistor.
 */
void setupDoorSensor() {
    pinMode(DOOR_SENSOR_PIN, INPUT_PULLUP);  // Assuming the sensor pulls the pin LOW when the door is open
    doorOpen = digitalRead(DOOR_SENSOR_PIN) == LOW;  // Set initial door state
    printToTelnet("Door sensor initialized.");
}

/**
 * @brief Monitors the door state and controls cooling and fan based on the door status.
 * 
 * This function constantly checks the state of the door sensor and takes 
 * appropriate action. If the door is open for too long, it will turn off the 
 * fan and cooling system to prevent unnecessary energy consumption. It also logs 
 * when the door is opened or closed, and how long it was open.
 */
void monitorDoor() {
    bool currentDoorState = digitalRead(DOOR_SENSOR_PIN) == HIGH;  // Read the current door state (LOW = closed, HIGH = open)
    
    if (currentDoorState != doorOpen) {
        // Door state has changed
        if (currentDoorState) {
            doorOpenedTime = millis();  // Record the time when the door was opened
            printToTelnet("Door opened.");
        } else {
            unsigned long doorOpenDuration = millis() - doorOpenedTime;  // Calculate how long the door was open
            printToTelnet("Door closed.");
            printToTelnet("Door was open for " + String(doorOpenDuration / 1000) + " seconds.");
        }
        doorOpen = currentDoorState;  // Update the door state
    }

    // If the door is open, check the duration
    if (doorOpen) {
        unsigned long currentOpenDuration = millis() - doorOpenedTime;

        // Turn off the fan if the door has been open for more than 30 seconds
        if (currentOpenDuration >= doorOpenFanThreshold && fanOn) {
            digitalWrite(FAN_RELAY_PIN, TURN_OFF);  // Turn the fan OFF
            fanOn = false;
            printToTelnet("Fan turned OFF due to door being open for more than 30 seconds.");
        }

        // Turn off the cooling if the door has been open for more than 60 seconds
        if (currentOpenDuration >= doorOpenCoolingThreshold && coolingOn) {
            deactivateCooling("door open for more than 60 seconds");  // Use the new deactivateCooling function
        }
    }
}

/**
 * @brief Initializes control systems for cooling, fan, and door sensors.
 * 
 * This function sets up the relay pins for controlling the cooling and fan systems.
 * It also initializes the temperature sensors and retrieves the saved cooling 
 * set point from non-volatile storage (Preferences).
 */
void setupControl() {
    // Set pin modes for cooling and fan relays
    pinMode(COOLING_RELAY_PIN, OUTPUT);
    digitalWrite(COOLING_RELAY_PIN, TURN_OFF);  // Start with cooling OFF
    pinMode(FAN_RELAY_PIN, OUTPUT);
    digitalWrite(FAN_RELAY_PIN, TURN_OFF);  // Start with fan OFF
    pinMode(EXTERNAL_FAN_RELAY_PIN, OUTPUT);
    digitalWrite(EXTERNAL_FAN_RELAY_PIN, TURN_OFF);  // Start with external fan OFF
    pinMode(POWER_RELAY, OUTPUT);
    digitalWrite(POWER_RELAY, TURN_OFF);  // Start with power OFF

    // Initialize the temperature sensors
    sensors.begin();
    printToTelnet("Temperature sensors initialized.");

    // Initialize Preferences and retrieve the saved set point
    preferences.begin("kegerator", false);  // Open namespace "kegerator" for read/write
    coolingSetPoint = preferences.getFloat("setPoint", 4.0);  // Retrieve saved set point or default to 4.0°C
    printToTelnet("Cooling set point loaded: " + String(coolingSetPoint) + " °C");

    setupDoorSensor();
    
    // Create the task for temperature readings in parallel
    xTaskCreatePinnedToCore(
        temperatureTask,       // Task function
        "TemperatureTask",     // Task name
        4096,                  // Stack size (in bytes)
        NULL,                  // Parameter
        1,                     // Priority
        &temperatureTaskHandle,// Task handle
        0                      // Core number (0 or 1)
    );
}

/**
 * @brief Turns the cooling system on and logs the activation time.
 */
void activateCooling() {
    digitalWrite(COOLING_RELAY_PIN, TURN_ON);  // Activate the cooling system
    coolingOn = true;
    lastCoolingOnTime = millis();  // Track the time cooling was turned on
    printToTelnet("Cooling system activated.");
    printToTelnet("Cooling was OFF for: " + String(coolingOffDuration / 1000) + " seconds.");
}

/**
 * @brief Turns the cooling system off and logs the deactivation time.
 * 
 * @param reason Reason for deactivation (e.g., reached set point, max time limit)
 */
void deactivateCooling(const String& reason) {
    digitalWrite(COOLING_RELAY_PIN, TURN_OFF);  // Deactivate the cooling system
    coolingOn = false;
    lastCoolingOffTime = millis();  // Record the time cooling was turned off

    // Calculate how long the cooling system was ON
    coolingOnDuration = lastCoolingOffTime - lastCoolingOnTime;  // Calculate on duration
    printToTelnet("Cooling system deactivated (" + reason + ").");
    printToTelnet("Cooling was ON for: " + String(coolingOnDuration / 1000) + " seconds.");
}

/**
 * @brief Checks if the cooling system has been on for too long (2 hours).
 */
bool isCoolingOnTooLong() {
    return (coolingOn && (millis() - lastCoolingOnTime >= 7200000));  // 7200000 ms = 2 hours
}

/**
 * @brief Determines if cooling should be activated based on temperature.
 */
bool shouldActivateCooling() {
    unsigned long currentMillis = millis();
    return (!doorOpen && insideTemp >= (coolingSetPoint + COOLING_HYSTERESIS) && 
            !coolingOn && (currentMillis - lastCoolingOffTime >= coolingDelay));
}

/**
 * @brief Determines if cooling should be deactivated based on reaching the set point.
 */
bool shouldDeactivateCooling() {
    return (insideTemp <= (coolingSetPoint - COOLING_HYSTERESIS) && coolingOn);
}

/**
 * @brief Controls the cooling system based on temperature readings and door status.
 * 
 * This function monitors the inside temperature and determines whether the cooling 
 * system should be turned on or off. It also respects the cooling delay and turns 
 * off the cooling if it has been running for more than 2 hours or if the set point is reached.
 */
void cooling() {
    if (insideTemp == DEVICE_DISCONNECTED_C) return;  // No valid temperature reading

    // Turn off cooling if it's been on for too long (2 hours)
    if (isCoolingOnTooLong()) {
        deactivateCooling("it was on for more than 2 hours");
        return;
    }

    // Turn on cooling if needed (based on temperature and delay)
    if (shouldActivateCooling()) {
        activateCooling();
    }

    // Turn off cooling if the temperature is below the set point
    if (shouldDeactivateCooling()) {
        deactivateCooling("Reached set point");
    }
}

/**
 * @brief Controls the fan based on temperature readings and door status.
 * 
 * This function manages the fan operation based on the temperature inside the Kegerator. 
 * The fan will be turned on if the temperature exceeds the set point plus a threshold 
 * and turned off when the temperature drops below the set point minus the threshold.
 */
void controlFan() {
    if (insideTemp == DEVICE_DISCONNECTED_C) return;

    // Turn the fan on if the inside temperature is higher than the cooling set point plus the threshold
    if (!doorOpen && insideTemp > (coolingSetPoint + FAN_THRESHOLD + FAN_HYSTERESIS) && !fanOn) {
        digitalWrite(FAN_RELAY_PIN, TURN_ON);  // Turn the fan ON
        fanOn = true;
        printToTelnet("Fan turned ON.");
    }
    // Turn the fan off if the inside temperature is lower than the cooling set point minus the threshold
    else if (insideTemp < (coolingSetPoint + FAN_THRESHOLD - FAN_HYSTERESIS) && fanOn) {
        digitalWrite(FAN_RELAY_PIN, TURN_OFF);  // Turn the fan OFF
        fanOn = false;
        printToTelnet("Fan turned OFF.");
    }
}

/**
 * @brief Calculates the average temperature from the temperature buffer.
 * 
 * This function calculates the average temperature using the temperature readings 
 * stored in the buffer over a period of time (typically one hour).
 * 
 * @return float The calculated average temperature.
 */
float calculateAverageTemp() {
    float sum = 0.0;
    int count = bufferFilled ? bufferSize : bufferIndex;  // Use the full buffer if filled, otherwise use up to the current index
    for (int i = 0; i < count; i++) {
        sum += insideTempBuffer[i];
    }
    return (count > 0) ? (sum / count) : 0.0;
}

/**
 * @brief Task that periodically reads temperature data and stores it in a buffer.
 * 
 * This FreeRTOS task periodically reads the temperature from the sensors. It retries 
 * up to 3 times if a sensor read fails and logs any errors. The temperature data is 
 * also stored in a buffer to allow for the calculation of the average temperature.
 * 
 * @param parameter A pointer to task parameters (unused in this case).
 */
void temperatureTask(void *parameter) {
    while (true) {
        bool validInsideTemp = false;
        bool validOutsideTemp = false;

        // Retry up to 3 times for inside and outside temperature readings
        for (int retry = 0; retry < 3; retry++) {
            // Request temperature readings from all sensors
            sensors.requestTemperatures();
            vTaskDelay(750 / portTICK_PERIOD_MS);  // Wait for the sensor reading
            
            insideTemp = sensors.getTempC(sensor1Address);
            outsideTemp = sensors.getTempC(sensor2Address);

            if (insideTemp != DEVICE_DISCONNECTED_C) {
                validInsideTemp = true;
                break;  // Exit loop if we get a valid reading
            }
        }

        for (int retry = 0; retry < 3; retry++) {
            if (outsideTemp != DEVICE_DISCONNECTED_C) {
                validOutsideTemp = true;
                break;  // Exit loop if we get a valid reading
            }
        }

        // Check if inside temperature is valid after 3 retries
        if (!validInsideTemp) {
            printToTelnetErr("Inside sensor disconnected or not found after 3 retries!");
        } else {
            // printToTelnet("Inside Temp: " + String(insideTemp) + " °C");

            // Store the inside temperature in the buffer
            insideTempBuffer[bufferIndex] = insideTemp;
            bufferIndex++;

            // If we fill the buffer, wrap the index around (circular buffer behavior)
            if (bufferIndex >= bufferSize) {
                bufferIndex = 0;
                bufferFilled = true;
            }
        }

        // Check if outside temperature is valid after 3 retries
        if (!validOutsideTemp) {
            // printToTelnetErr("Outside sensor disconnected or not found after 3 retries!");
        } else {
            // printToTelnet("Outside Temp: " + String(outsideTemp) + " °C");
        }

        // Calculate and print the average temperature after the buffer has been filled
        if (bufferFilled) {
            float avgTemp = calculateAverageTemp();
            printToTelnet("Average Inside Temp (Last Hour): " + String(avgTemp) + " °C");
        }

        // Delay the task for 20 seconds (non-blocking delay in FreeRTOS)
        vTaskDelay(20000 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Updates the cooling set point and saves it in non-volatile memory.
 * 
 * This function updates the cooling set point and stores the new value in 
 * non-volatile memory (Preferences) so that it persists across device reboots.
 * 
 * @param newSetPoint The new cooling set point to be saved.
 */
void updateCoolingSetPoint(float newSetPoint) {
    coolingSetPoint = newSetPoint;
    preferences.putFloat("setPoint", coolingSetPoint);  // Save the new set point to Preferences
    printToTelnet("Cooling set point updated: " + String(coolingSetPoint) + " °C");
}
