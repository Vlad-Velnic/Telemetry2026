#ifndef FRONT_MODULE_LOOP
#define FRONT_MODULE_LOOP

#include "includes.h"

void sendCanMessage(int id, uint8_t* data, size_t length);
void onCanReceive(int packetSize);
void updateDisplay(u_int8_t currentGear, unsigned long lastLapTime, float currentTemp, float currentBatteryVoltage, int currentRPM);

#endif