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

// --- GLOBALE PENTRU DATE (Shared) ---
extern volatile int currentRPM;
extern volatile float currentTemp;
extern volatile float currentBat;
extern volatile int currentGear;
extern volatile unsigned long lastLapTime;

extern bool NO_REAR, NO_WIFI, NO_ECU;


// Instanță SPI dedicată pentru SD Card (HSPI)
extern SPIClass sdSPI;

// Structura pentru coada de mesaje
struct LogMessage {
    uint32_t id;
    uint8_t len;
    uint8_t data[8];
    unsigned long timestamp;
    bool isRx; // true = RX (primit), false = TX (trimis de noi)
};

extern QueueHandle_t canQueue;
extern Adafruit_SSD1306 display;
extern Adafruit_MPU6050 mpu;

#endif