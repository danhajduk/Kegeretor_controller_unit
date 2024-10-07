#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>  // Include Arduino library for String type and other utilities

// Function declarations
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

#endif  // CONTROL_H
