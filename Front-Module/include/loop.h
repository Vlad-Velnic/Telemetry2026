#ifndef FRONT_MODULE_LOOP
#define FRONT_MODULE_LOOP

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Task definitions
void CAN_Task(void *pvParameters);
void SD_Task(void *pvParameters);

// Utility functions
void broadcastData(uint32_t id, uint8_t* data, size_t len);
void sendHealthFrame();
void updateDisplay(uint8_t currentGear, unsigned long lastLapTime,
                   float gpsSpeed, uint16_t brakePressure);

#endif
