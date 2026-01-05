#ifndef PINOUT_H
#define PINOUT_H

// --- MODEM PINS (LilyGO T-Call A7670) ---
#define MODEM_TX            26
#define MODEM_RX            25
#define MODEM_PWRKEY        4
#define MODEM_POWER_ON      12

// --- CAN BUS PINS (TWAI) ---
#define PIN_CAN_TX          21
#define PIN_CAN_RX          22

// --- ANALOG SENSORS ---
// Make sure these are ADC1 pins if using WiFi (ADC2 is restricted)
#define PIN_DAMPER_RL       32
#define PIN_DAMPER_RR       33
#define PIN_BRAKE_PRESS     34

// --- GEAR SENSORS (Digital Inputs) ---
#define PIN_GEAR_1          13
#define PIN_GEAR_2          14
#define PIN_GEAR_3          27
#define PIN_GEAR_4          35 
#define PIN_GEAR_5          36
#define PIN_GEAR_0          39

#endif