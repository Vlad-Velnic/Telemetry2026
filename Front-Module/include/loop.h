#ifndef FRONT_MODULE_LOOP
#define FRONT_MODULE_LOOP

#include "includes.h"

struct LogMessage {
    uint32_t id;
    uint8_t len;
    uint8_t data[8];
    unsigned long timestamp;
    bool isRx;
};

extern QueueHandle_t canQueue; 
extern volatile int currentRPM;
extern volatile float currentTemp;
extern volatile float currentBat;

void sendCanMessage(uint32_t id, uint8_t* data, size_t length);
void updateDisplay(u_int8_t currentGear, unsigned long lastLapTime, float currentTemp, float currentBatteryVoltage, int currentRPM);

void CAN_Task(void *pvParameters);
void SD_Task(void *pvParameters);

#endif