#ifndef FRONT_MODULE_LOOP
#define FRONT_MODULE_LOOP

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"


void sendCanMessage(uint32_t id, uint8_t* data, size_t length);
void updateDisplay(uint8_t currentGear, unsigned long lastLapTime, float currentTemp, float currentBatteryVoltage, int currentRPM);
void readMPUData();
void readMPUData2();

void CAN_Task(void *pvParameters);
void SD_Task(void *pvParameters);
void broadcastData(uint32_t id, uint8_t* data, size_t len);

#endif