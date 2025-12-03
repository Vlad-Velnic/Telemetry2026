#ifndef FRONT_MODULE_SETUP
#define FRONT_MODULE_SETUP

#include "includes.h"

Adafruit_SSD1306 display(128, 64, PIN_OLED_MOSI, PIN_OLED_CLK, PIN_OLED_DC, PIN_OLED_RESET, PIN_OLED_CS);
Adafruit_MPU6050 mpu;
bool NO_REAR, NO_WIFI, NO_ECU;

void setupPins();
void setupOLED();
void setupSD();
void setupMPU();
void setupCAN();

#endif