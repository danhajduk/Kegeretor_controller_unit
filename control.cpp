#include "control.h"
#include "config.h"
#include "utils.h"
#include <Arduino.h>
#include "time.h"

// Create a OneWire instance for temperature sensors
OneWire oneWire(ONE_WIRE_BUS);

// Pass the OneWire instance to DallasTemperature library
DallasTemperature sensors(&oneWire);

// Global variables for storing temperature readings
float insideTemp = 0.0;
float outsideTemp = 0.0;

int invalidInsideTempCount = 0;       // Counter for consecutive invalid inside temperature readings
int invalidOutsideTempCount = 0;      // Counter for consecutive invalid outside temperature readings

unsigned long lastInsideRecheckTime = 0;  // Last time inside sensor was rechecked
unsigned long lastOutsideRecheckTime = 0; // Last time outside sensor was rechecked

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

// Global variables for skewed activation and deactivation points
float skewedActivationPoint;
float skewedDeactivationPoint;

// Global variable to store the last time the skewed points were updated.
unsigned long lastSkewUpdateTime = 0;
const unsigned long skewUpdateInterval = 300000; // 10 minutes in milliseconds

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

unsigned long lastCrushCheck = 0;  // Keeps track of the last time monitorCrush was called
const unsigned long crushCheckInterval = 20000;  // 20 seconds in milliseconds

/**
 * @brief Initializes the skewed points based on the set point.
 * 
 * This should be called during setup or whenever the cooling set point is changed.
 */
void initializeSkewedPoints(float setPoint) {
    skewedActivationPoint = setPoint + COOLING_HYSTERESIS;
    skewedDeactivationPoint = setPoint - COOLING_HYSTERESIS;
    printToTelnet("Initial Skewed Activation Point: " + String(skewedActivationPoint) + " °C");
    printToTelnet("Initial Skewed Deactivation Point: " + String(skewedDeactivationPoint) + " °C");
}

/**
 * @brief Updates the skewed activation and deactivation points based on the deviation.
 * 
 * This function adjusts the skewed points by adding/subtracting a calculated deviation
 * derived from the difference between the average temperature and the set point.
 * 
 * @param setPoint The target set point for cooling.
 * @param avgTemp The average temperature of the buffer.
 * @param skewFactor The factor to apply for the skewing.
 */
void updateSkewedPoints(float setPoint, float avgTemp, float skewFactor = SKEW_FACTOR) {
    // Get the current time
    unsigned long currentMillis = millis();

    // Ensure a minimum interval between updates
    if (currentMillis - lastSkewUpdateTime < skewUpdateInterval) {
        return; // Skip the update if the interval hasn't elapsed
    }
    // Update the last time the skew points were adjusted
    lastSkewUpdateTime = currentMillis;

    // Calculate the deviation between average temperature and set point
    float deviation = avgTemp - setPoint;

    // Only adjust if the deviation is greater than a small threshold (e.g., 0.1°C)
    if (abs(deviation) > 0.05) {
        // Adjust the skewed points based on the deviation.
        skewedActivationPoint -= skewFactor * deviation;
        skewedDeactivationPoint -= skewFactor * deviation;

        printToTelnet("Updated Skewed Activation Point: " + String(skewedActivationPoint) + " °C ");
        printToTelnet("Updated Skewed Deactivation Point: " + String(skewedDeactivationPoint) + " °C");

    } else {
        printToTelnet("Deviation too small for skew adjustment: " + String(deviation) + " °C.");
        printToTelnet("Activation Point: " + String(skewedActivationPoint) + " °C" + 
                      "Deactivation Point: " + String(skewedDeactivationPoint) + " °C");

    }
}

/**
 * @brief Main control function that handles cooling, fan, and door monitoring.
 * 
 * This function is called repeatedly in the main loop. It manages the cooling, fan, 
 * and door monitoring functionalities by calling the respective functions.
 */
void handleControl() {
    cooling();         // Handle cooling logic
    controlFan();      // Handle fan control logic
    monitorDoor();     // Monitor door sensor and manage door-related logic

    // Calculate the average temperature periodically and update skewed points if needed
    float avgTemp = calculateAverageTemp();
    updateSkewedPoints(coolingSetPoint, avgTemp);

    // Check if 20 seconds have passed since the last time we called monitorCrush()
    unsigned long currentMillis = millis();
    if (currentMillis - lastCrushCheck >= crushCheckInterval) {
        monitorCrush();                  // Call the crush monitoring function
        lastCrushCheck = currentMillis;  // Update the last time it was called
        printDashboard();
    }
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
    bool currentDoorState = digitalRead(DOOR_SENSOR_PIN) == HIGH;

    if (currentDoorState != doorOpen) {
        unsigned long currentMillis = millis();
        if (currentDoorState) {
            doorOpenedTime = currentMillis;
            printToTelnet("Door opened.");
        } else {
            unsigned long doorOpenDuration = currentMillis - doorOpenedTime;
            printToTelnet("Door closed.");
            printToTelnet("Door was open for " + String(doorOpenDuration / 1000) + " seconds.");

            if (doorOpenDuration < doorOpenCoolingThreshold) {
                printToTelnet("Short door open event detected. Resuming normal operation.");
            }
        }
        doorOpen = currentDoorState;
    }

    // If the door is open, check the duration for fan and cooling control
    if (doorOpen) {
        unsigned long currentOpenDuration = millis() - doorOpenedTime;
        if (currentOpenDuration >= doorOpenFanThreshold && fanOn) {
            digitalWrite(FAN_RELAY_PIN, TURN_OFF);
            fanOn = false;
            printToTelnet("Fan turned OFF due to door being open for more than 30 seconds.");
        }
        if (currentOpenDuration >= doorOpenCoolingThreshold && coolingOn) {
            deactivateCooling("door open for more than 60 seconds");
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
    printToTelnet("I/O pins setup completed");

    // Initialize the temperature sensors
    sensors.begin();
    printToTelnet("Temperature sensors initialized.");

    // Initialize Preferences and retrieve the saved set point
    preferences.begin("kegerator", false);  // Open namespace "kegerator" for read/write
    coolingSetPoint = preferences.getFloat("setPoint", 4.0);  // Retrieve saved set point or default to 4.0°C
    printToTelnet("Cooling set point loaded: " + String(coolingSetPoint) + " °C");
    preferences.putBool("in_sens_avail", true);  // Mark inside sensor as available
    preferences.putBool("out_sens_avail", true);  // Mark inside sensor as available
    initializeSkewedPoints(coolingSetPoint);  // Initialize the skewed points.

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
    printToTelnet("Temperature reading task created");
}

/**
 * @brief Turns the cooling system on and logs the activation time.
 */
void activateCooling() {
    digitalWrite(COOLING_RELAY_PIN, TURN_ON);  // Activate the cooling system
    coolingOn = true;
    lastCoolingOnTime = millis();  // Track the time cooling was turned on
    coolingOffDuration = lastCoolingOnTime - lastCoolingOffTime;
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
 * @brief Determines if cooling should be activated with skewed thresholds.
 * 
 * Uses the calculated average temperature to adjust the cooling threshold,
 * skewing the cooling set point dynamically.
 */
bool shouldActivateCooling() {
    return (!doorOpen && insideTemp >= skewedActivationPoint && !coolingOn &&
            (millis() - lastCoolingOffTime >= coolingDelay));
}

/**
 * @brief Determines if cooling should be deactivated with skewed thresholds.
 * 
 * Uses the calculated average temperature to adjust the cooling threshold,
 * skewing the cooling set point dynamically.
 */
bool shouldDeactivateCooling() {
    return (insideTemp <= skewedDeactivationPoint && coolingOn);
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
        deactivateCooling("it was ON for more than 2 hours");
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
 * @brief Reads temperature from a given sensor and checks its validity.
 * 
 * @param address The sensor address.
 * @param temp Reference to store the read temperature.
 * @param minTemp Minimum valid temperature.
 * @param maxTemp Maximum valid temperature.
 * @return bool True if the temperature is valid, otherwise false.
 */
bool readTemperature(uint8_t* address, float& temp, float minTemp, float maxTemp) {
    for (int retry = 0; retry < 3; retry++) {
        sensors.requestTemperatures();
        vTaskDelay(750 / portTICK_PERIOD_MS);  // Wait for the sensor reading
        temp = sensors.getTempC(address);
        
        if (temp != DEVICE_DISCONNECTED_C && temp >= minTemp && temp <= maxTemp) {
            return true;  // Valid temperature reading
        }
    }
    return false;  // Invalid temperature after retries
}

/**
 * @brief Updates sensor availability based on validity and logs changes.
 * If the temperature reading is invalid, it keeps the last valid reading.
 * Only writes to EEPROM if the value changes to avoid unnecessary writes.
 * 
 * @param validTemp Indicates if the sensor has a valid reading.
 * @param sensorAvailable Reference to the sensor availability status.
 * @param sensorType A string identifier for the sensor (e.g., "inside" or "outside").
 * @param lastValidTemp The last valid temperature reading to retain if the current reading is invalid.
 * @param currentTemp Reference to the variable storing the current temperature.
 */
void updateSensorAvailability(bool validTemp, bool& sensorAvailable, const char* sensorType, float lastValidTemp, float& currentTemp) {
    String key = String(sensorType) + "_sens_avail";
    bool previousState = preferences.getBool(key.c_str(), true); // Retrieve the previously stored value

    if (!validTemp) {
        sensorAvailable = false;
        printToTelnetErr(String(sensorType) + " sensor marked as unavailable. Keeping last valid reading: " + String(lastValidTemp) + " °C.");
        currentTemp = lastValidTemp; // Retain the last valid reading
        
        // Only write to preferences if the value has changed
        if (previousState != sensorAvailable) {
            preferences.putBool(key.c_str(), sensorAvailable);
        }
    } else {
        sensorAvailable = true;        
        // Only write to preferences if the value has changed
        if (previousState != sensorAvailable) {
            preferences.putBool(key.c_str(), sensorAvailable);
            printToTelnet(String(sensorType) + " sensor marked as available again.");
        }
    }
}

/**
 * @brief Updates the temperature buffer for averaging purposes.
 * Checks for outliers before adding the new reading to maintain stability.
 * 
 * @param newTemp The new temperature reading to add to the buffer.
 */
void updateTemperatureBuffer(float newTemp) {
    if (newTemp >= MIN_VALID_TEMP && newTemp <= MAX_INSIDE_TEMP) {
        insideTempBuffer[bufferIndex] = newTemp;
        bufferIndex = (bufferIndex + 1) % bufferSize;
        bufferFilled = bufferFilled || (bufferIndex == 0);
    } else {
        printToTelnetErr("Skipped outlier temperature reading: " + String(newTemp) + " °C.");
    }
}

/**
 * @brief Task that periodically reads temperature data, validates sensors, and manages availability.
 *  Use previous valid reading if the current one is invalid.
 *  Adjust delay time for more frequent checks when issues are detected.
 * 
 * @param parameter A pointer passed to the task (not used here).
 */
void temperatureTask(void *parameter) {
  int inSensorCounter = 0;
  int outSensorCounter = 0;
  int inSesnsorTimer = 0;
  int outSensorTimer = 0;
  float previousReading = DEVICE_DISCONNECTED_C;

    while (true) {
        // Retrieve initial sensor availability from Preferences
        bool insideSensorAvailable = preferences.getBool("in_sens_avail", true);
        bool outsideSensorAvailable = preferences.getBool("out_sens_avail", true);

        int now = millis();

        if (inSensorCounter < MAX_INVALID_RETRIES || now - inSesnsorTimer > RECHECK_INTERVAL) {
            inSesnsorTimer = now; // Reset the timer
            previousReading = insideTemp;
            // Read and validate inside temperature
            bool validInsideTemp = insideSensorAvailable 
                ? readTemperature(sensor1Address, insideTemp, MIN_VALID_TEMP, MAX_INSIDE_TEMP)
                : false;

            // Update inside sensor availability with last known valid inside temperature
            updateSensorAvailability(validInsideTemp, insideSensorAvailable, "inside", previousReading, insideTemp);
            if (validInsideTemp) {
                updateTemperatureBuffer(insideTemp);
                inSensorCounter = 0;
            } else inSensorCounter ++;
        } else insideTemp = DEVICE_DISCONNECTED_C;

        previousReading = DEVICE_DISCONNECTED_C;

        if (outSensorCounter < MAX_INVALID_RETRIES || now - outSensorTimer > RECHECK_INTERVAL) {
            outSensorTimer = now; // Reset the timer
            previousReading = outsideTemp;
            bool validOutsideTemp = outsideSensorAvailable 
                ? readTemperature(sensor2Address, outsideTemp, MIN_VALID_TEMP, MAX_OUTSIDE_TEMP)
                : false;
            updateSensorAvailability(validOutsideTemp, outsideSensorAvailable, "outside", previousReading, outsideTemp);
            if (validOutsideTemp) outSensorCounter = 0;
            else outSensorCounter++;
        } else outsideTemp = DEVICE_DISCONNECTED_C;
        // Determine the appropriate delay time
        int delayTime = ((inSensorCounter > 0 && inSensorCounter < MAX_INVALID_RETRIES) ||
                        (outSensorCounter > 0 && outSensorCounter < MAX_INVALID_RETRIES)) 
                         ? SHORT_DELAY : LONG_DELAY;

        // Delay the task for 20 seconds (non-blocking delay in FreeRTOS)
        vTaskDelay(delayTime / portTICK_PERIOD_MS);
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
    initializeSkewedPoints(coolingSetPoint);  // Initialize the skewed points.
    preferences.putFloat("setPoint", coolingSetPoint);  // Save the new set point to Preferences
    printToTelnet("Cooling set point updated: " + String(coolingSetPoint) + " °C");
}

/**
 * @brief Sets up the cold crush process with the specified temperature and time.
 * 
 * This function stores the crush temperature and duration, saves the current 
 * cooling set point, and updates the cooling set point to the crush temperature.
 * 
 * @param temp The temperature to maintain during the crush process (in °C).
 *             It must be greater than or equal to 1°C.
 * @param time The duration for the crush process (in hours).
 *             It must be greater than zero.
 */
void setupCrush(float temp, int time) {
    preferences.begin("kegerator", false);  // Open preferences namespace for read/write
    // Validate parameters (temperature must be >= 1°C and time must be > 0 hours)
    if (temp < 1) {
        printToTelnetErr("Invalid temperature: Must be 1°C or higher.");
        return;  // Exit the function if the temperature is invalid
    }
    if (time <= 0) {
        printToTelnetErr("Invalid time: Must be greater than zero.");
        return;  // Exit the function if the time is invalid
    }

    // Store crush settings in preferences
    preferences.putFloat("crush_temp", temp);          // Store crush temperature
    preferences.putInt("crush_time", time);            // Store crush duration (in hours)
    preferences.putBool("crush_setup", true);          // Flag: crush setup initiated
    preferences.putBool("crush_started", false);       // Flag: crush process hasn't started

    // Store the current cooling set point (save the original temp)
    float currentSetPoint = coolingSetPoint;           // Retrieve current cooling set point
    printToTelnet("Current cooling set point: " + String(currentSetPoint) + " °C");
    preferences.putFloat("original_temp", currentSetPoint);  // Store original set point

    preferences.putFloat("original_temp", currentSetPoint);  // Store original set point
    float storedSetPoint = preferences.getFloat("original_temp", -1.0);
    printToTelnet("Stored original cooling set point: " + String(storedSetPoint) + " °C");

    // Update the cooling set point to the crush temperature
    updateCoolingSetPoint(temp);                       // Set the cooling point to the crush temperature

    // Feedback to Telnet
    printToTelnet("Crush setup initiated.");
    printToTelnet("Target temperature: " + String(temp) + " °C for " + String(time) + " hours.");
    printToTelnet("Original cooling set point: " + String(currentSetPoint) + " °C saved.");
//    printCrushPreferences();
}

/**
 * @brief Monitors the cold crush process to ensure the target temperature is reached
 *        and checks if the crush process has completed based on the end time.
 * 
 * This function checks two things:
 * 1. If the current temperature has reached or gone below the specified crush temperature.
 *    Once the temperature is reached, it stores the start time and calculates the end time.
 * 2. If the current time has passed the crush end time, it restores the original cooling 
 *    set point and marks the crush process as completed.
 */
void monitorCrush() {
    // Retrieve the crush parameters and status from preferences
    float crushTemp = preferences.getFloat("crush_temp", 0.0);
    int crushTime = preferences.getInt("crush_time", 0);  // Duration in hours
    bool crushSetup = preferences.getBool("crush_setup", false);
    bool crushStarted = preferences.getBool("crush_started", false);

    // Retrieve the original cooling set point and crush end timestamp
    float originalSetPoint = preferences.getFloat("original_temp", coolingSetPoint);
    time_t crushEndTime = preferences.getULong("crush_end_time", 0);  // Retrieve the end time as a timestamp

    // Get the current time as a `time_t` object (seconds since epoch)
    time_t currentTime;
    time(&currentTime);

    // If the crush process hasn't started but the setup is in progress
    if (crushSetup && !crushStarted) {
        if (insideTemp <= crushTemp) {
            // Calculate the end time by adding the crush duration (in hours, converted to seconds)
            crushEndTime = currentTime + (crushTime * 3600);  // Add hours to current time in seconds

            // Store the start and end times as timestamps in Preferences
            preferences.putULong("crush_time", currentTime);   // Store the start time as a timestamp
            preferences.putULong("crush_end_time", crushEndTime);    // Store the end time as a timestamp

            // Mark the crush process as started
            preferences.putBool("crush_started", true);
            preferences.putBool("crush_setup", false);  // Crush setup is complete

            // Provide feedback via Telnet with formatted times
            printToTelnet("Crush temperature of " + String(crushTemp) + " °C reached.");
            printToTelnet("Crush started at: " + String(ctime(&currentTime)));  // Convert start time to readable format
            printToTelnet("Crush will end at: " + String(ctime(&crushEndTime)));  // Convert end time to readable format
        } else {
            printToTelnet("Crush setting up... Current temp: " + String(insideTemp) + " °C. Target: " + String(crushTemp) + " °C.");
        }
    }

    // If the crush process has started, monitor its completion
    if (crushStarted) {
        // Check if the current time has passed the crush end time
        if (currentTime >= crushEndTime) {
            // Restore the original cooling set point
            updateCoolingSetPoint(originalSetPoint);

            // Mark the crush process as complete
            preferences.putBool("crush_started", false);  // Mark crush as finished

            // Provide feedback via Telnet
            printToTelnet("Crush process complete.");
            printToTelnet("Cooling set point restored to: " + String(originalSetPoint) + " °C.");
        } else {
            // Provide status update if the process is still running
            printToTelnet("Crush in progress. End time: " + String(ctime(&crushEndTime)));
        }
    }
}

/**
 * @brief Converts a Unix timestamp (time_t) to a formatted string in "MM/DD/YYYY HH:MM" format.
 * 
 * @param timestamp The Unix timestamp to be converted.
 * @return String The formatted date and time as a string.
 */
String convertTimestampToDateTime(time_t timestamp) {
    struct tm *timeinfo = localtime(&timestamp);  // Convert timestamp to local time
    char buffer[20];  // Buffer to hold the formatted date and time string
    
    // Format the time into "MM/DD/YYYY HH:MM"
    strftime(buffer, sizeof(buffer), "%m/%d/%Y %H:%M", timeinfo);

    return String(buffer);  // Return the formatted string
}

/**
 * @brief Pulls all crush-related preferences and prints them to the Telnet client.
 * 
 * This function retrieves the crush temperature, crush time, original cooling set point, 
 * crush start time, and crush end time from Preferences, then prints the values to the Telnet client.
 */
void printCrushPreferences() {
    float crushTemp = preferences.getFloat("crush_temp", 0.0);
    int crushTime = preferences.getInt("crush_time", 0);
    bool crushSetup = preferences.getBool("crush_setup", false);
    bool crushStarted = preferences.getBool("crush_started", false);
    float originalSetPoint = preferences.getFloat("original_temp", 0.0);
    time_t crushStartTime = preferences.getULong("crush_time", 0);
    time_t crushEndTime = preferences.getULong("crush_end_time", 0);

    printToTelnet("Crush Preferences:");
    printToTelnet("  Crush Temp: " + String(crushTemp, 2) + " °C");
    printToTelnet("  Crush Time: " + String(crushTime) + " hours");
    printToTelnet("  Crush Setup: " + String(crushSetup ? "True" : "False"));
    printToTelnet("  Crush Started: " + String(crushStarted ? "True" : "False"));
    printToTelnet("  Original Cooling Set Point: " + String(originalSetPoint, 2) + " °C");

    if (crushStartTime != 0) {
        printToTelnet("  Crush Start Time: " + String(ctime(&crushStartTime)));
    } else {
        printToTelnet("  Crush Start Time: N/A");
    }

    if (crushEndTime != 0) {
        printToTelnet("  Crush End Time: " + String(ctime(&crushEndTime)));
    } else {
        printToTelnet("  Crush End Time: N/A");
    }
}
