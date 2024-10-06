#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>  // Include Arduino library for String type and other utilities

// Function declarations
void setupControl();                         // Initialize control logic
void handleControl();                        // Handle control logic (temperature, relays, etc.)
void temperatureTask(void *parameter);       // Update temperature readings in the background
void updateCoolingSetPoint(float newSetPoint);
float calculateAverageTemp();

// Global variable declarations
extern bool doorOpen;                        // Tracks if the door is currently open
extern unsigned long coolingOnDuration;      // Duration of the cooling being ON
extern unsigned long coolingOffDuration;     // Duration of the cooling being OFF
extern unsigned long lastCoolingOnTime;      // Timestamp of the last cooling ON event

#endif  // CONTROL_H
