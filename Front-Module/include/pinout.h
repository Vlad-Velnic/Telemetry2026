#ifndef FRONT_MODULE_PINS_H
#define FRONT_MODULE_PINS_H

#include <Arduino.h>

// ================================================================
// OLED DISPLAY
// ================================================================
constexpr int8_t PIN_OLED_MOSI  = 13;
constexpr int8_t PIN_OLED_CLK   = 14;
constexpr int8_t PIN_OLED_DC    = 33;
constexpr int8_t PIN_OLED_CS    = 32;
constexpr int8_t PIN_OLED_RESET = -1;

// ================================================================
// SD CARD
// ================================================================
constexpr int8_t PIN_SD_MOSI = 23;
constexpr int8_t PIN_SD_MISO = 19;
constexpr int8_t PIN_SD_SCK  = 18;
constexpr int8_t PIN_SD_CS   = 22;

// ================================================================
// ANALOG SENSORS
// ================================================================
constexpr int8_t PIN_DAMPER_1    = 36;
constexpr int8_t PIN_DAMPER_2    = 39;
constexpr int8_t PIN_STEERING    = 34;

// ================================================================
// MPU 6050
// ================================================================
constexpr int8_t PIN_MPU_SDA = 21;
constexpr int8_t PIN_MPU_SCL = 25;

// ================================================================
// CAN BUS (TJA1050)
// ================================================================
constexpr int8_t PIN_CAN_TX = 26;
constexpr int8_t PIN_CAN_RX = 27;

#endif