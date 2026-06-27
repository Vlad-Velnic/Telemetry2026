#ifndef FRONT_MODULE_INCLUDES
#define FRONT_MODULE_INCLUDES

// Libraries
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Preferences.h>
#include "driver/twai.h" 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// Local Modules
#include "pinout.h"
#include "debug.h"
#include "setup.h"
#include "loop.h"
#include "canIDs.h"

// Timing Constants
#define LOGGING_FREQ_HZ 25
#define DISPLAY_FREQ_HZ 10
#define LOGGING_PERIOD_MS (1000 / LOGGING_FREQ_HZ)
#define DISPLAY_PERIOD_MS (1000 / DISPLAY_FREQ_HZ)
#define HEALTH_PERIOD_MS 5000

// Health frame node IDs
#define HEALTH_NODE_FRONT 1
#define HEALTH_NODE_REAR 2

// --- DATA GLOBALS (Shared) ---
extern volatile int currentRPM;
extern volatile float currentTemp;
extern volatile float currentBat;
extern volatile int currentGear;
extern volatile unsigned long lastLapTime;
extern volatile uint32_t frontCanQueueDrops;
extern volatile uint32_t frontCanTxFailures;

extern bool NO_REAR, NO_WIFI, NO_ECU;


// Dedicated SPI instance for SD Card (HSPI)
extern SPIClass sdSPI;

// Structure for message queue
struct LogMessage {
    uint32_t id;
    uint8_t len;
    uint8_t data[8];
    unsigned long timestamp;
    bool isRx; // true = RX (received), false = TX (sent by us)
};

extern QueueHandle_t canQueue;
extern Adafruit_SSD1306 display;
extern Adafruit_MPU6050 mpu;
extern char logFileName[32];

#endif
