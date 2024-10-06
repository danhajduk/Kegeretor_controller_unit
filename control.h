#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>  // Include Arduino library for String type and other utilities

void setupControl();     // Function to initialize control logic
void handleControl();    // Function to handle control logic (temperature, relays, etc.)

// Function to update temperature readings in the background
void temperatureTask(void *parameter);
void updateCoolingSetPoint(float newSetPoint);
float calculateAverageTemp();


extern bool doorOpen;  // Tracks if the door is currently open
extern unsigned long coolingOnDuration;   // How long the cooling was ON
extern unsigned long coolingOffDuration;  // How long the cooling was OFF
extern unsigned long lastCoolingOnTime;   // Time when cooling was last turned on
#endif
