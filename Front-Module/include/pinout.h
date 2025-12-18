#ifndef FRONT_MODULE_PINS_H
#define FRONT_MODULE_PINS_H

#include <Arduino.h>

// ================================================================
// OLED DISPLAY
// ================================================================
constexpr int8_t PIN_OLED_MOSI  = GPIO_NUM_23;
constexpr int8_t PIN_OLED_CLK   = GPIO_NUM_18;
constexpr int8_t PIN_OLED_DC    = GPIO_NUM_33;
constexpr int8_t PIN_OLED_CS    = GPIO_NUM_4;
constexpr int8_t PIN_OLED_RESET = -1;

// ================================================================
// SD CARD
// ================================================================
constexpr int8_t PIN_SD_MOSI = GPIO_NUM_13;
constexpr int8_t PIN_SD_MISO = GPIO_NUM_12;
constexpr int8_t PIN_SD_SCK  = GPIO_NUM_14;
constexpr int8_t PIN_SD_CS   = GPIO_NUM_15; // 22 in pinout

// ================================================================
// ANALOG SENSORS
// ================================================================
constexpr int8_t PIN_DAMPER_1    = GPIO_NUM_36;
constexpr int8_t PIN_DAMPER_2    = GPIO_NUM_39;
constexpr int8_t PIN_STEERING    = GPIO_NUM_34;

// ================================================================
// MPU 6050
// ================================================================
constexpr int8_t PIN_MPU_SDA = GPIO_NUM_21;
constexpr int8_t PIN_MPU_SCL = GPIO_NUM_25;

// ================================================================
// CAN BUS (TJA1050)
// ================================================================
constexpr int8_t PIN_CAN_TX = GPIO_NUM_26;
constexpr int8_t PIN_CAN_RX = GPIO_NUM_27;

#endif