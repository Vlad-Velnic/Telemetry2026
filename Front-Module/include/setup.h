#ifndef FRONT_MODULE_SETUP
#define FRONT_MODULE_SETUP

#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>

extern Adafruit_SSD1306 display;
extern Adafruit_MPU6050 mpu;
extern bool NO_REAR, NO_WIFI, NO_ECU;

void setupPins();
void setupOLED();
void setupSD();
void setupMPU();
void setupCAN();

#endif