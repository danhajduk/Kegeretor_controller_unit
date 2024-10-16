#ifndef CONTROL_H
#define CONTROL_H


#include <Arduino.h>  // Include Arduino library for String type and other utilities

// Temperature reading defenitions
#define MIN_VALID_TEMP -10.0          // Minimum valid temperature for both inside and outside
#define MAX_INSIDE_TEMP 45.0          // Maximum valid temperature for inside
#define MAX_OUTSIDE_TEMP 70.0         // Maximum valid temperature for outside
#define MAX_INVALID_RETRIES 5         // Maximum consecutive invalid readings before marking the sensor as unavailable
#define RECHECK_INTERVAL 300000       // 5 minutes in milliseconds
#define SHORT_DELAY 5000            // 5 seconds in milliseconds
#define LONG_DELAY 20000            // 20 seconds in milliseconds

// Cooling control sefenitions
#define SKEW_FACTOR 0.1


// Function declarations
void initializeSkewedPoints(float setPoint); // Initializes the skewed points based on the set point.
void updateSkewedPoints(float setPoint, float avgTemp, float skewFactor);
                                             // Updates the skewed activation and deactivation points based on the deviation.
void setupControl();                         // Initialize control logic
void handleControl();                        // Handle control logic (temperature, relays, etc.)
void temperatureTask(void *parameter);       // Update temperature readings in the background
void updateCoolingSetPoint(float newSetPoint);  // Update the cooling set point
float calculateAverageTemp();                // Calculate average temperature from buffer
void cooling();                              // Control the cooling system logic
void controlFan();                           //Controls the fan based on temperature readings and door status.
void activateCooling();                      // Function to activate cooling
void deactivateCooling(const String& reason); // Function to deactivate cooling
bool isCoolingOnTooLong();                   // Check if cooling is on for too long
bool shouldActivateCooling();                // Check if cooling should be activated
bool shouldDeactivateCooling();              // Check if cooling should be deactivated
void monitorDoor();                          // Monitors the door state and controls cooling and fan based on the door status.
void setupCrush(float temp, int time);         // Sets up the cold crush process with the specified temperature and time.
void monitorCrush();                         // Monitor cold crush process
String convertTimestampToDateTime(time_t timestamp);//Converts a Unix timestamp (time_t) to a formatted string in "MM/DD/YYYY HH:MM" format.
void printCrushPreferences();

// Global variable declarations
extern bool doorOpen;                        // Tracks if the door is currently open
extern bool coolingOn;                       // Tracks if the cooling system is currently ON
extern unsigned long coolingOnDuration;      // Duration of the cooling being ON
extern unsigned long coolingOffDuration;     // Duration of the cooling being OFF
extern unsigned long lastCoolingOnTime;      // Timestamp of the last cooling ON event
extern unsigned long lastCoolingOffTime;     // Timestamp of the last cooling OFF event
extern float coolingSetPoint;                // The cooling set point temperature
extern float insideTemp;                     // The current inside temperature
extern unsigned long coolingDelay;           // Minimum delay between cooling cycles
// Global variables for skewed activation and deactivation points
extern float skewedActivationPoint;
extern float skewedDeactivationPoint;

#endif  // CONTROL_H
