#ifndef INCLUDES_H
#define INCLUDES_H

#include <Arduino.h>
#include "pinout.h"
#include "debug.h"
#include "canIDs.h"

// --- LIBRARIES ---
#define TINY_GSM_MODEM_A7670
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include "driver/twai.h" // Native CAN driver

// --- CONFIGURATION ---
extern const char apn[];
extern const char user[];
extern const char pass[];
extern const char* mqtt_server;
extern const char* mqtt_topic;

// --- GLOBAL OBJECTS ---
extern TinyGsm modem;
extern TinyGsmClient client;
extern PubSubClient mqtt;

// --- SHARED VARIABLES ---
extern volatile float gps_lat;
extern volatile float gps_lon;
extern volatile float gps_speed;
extern volatile int currentGear;

// --- DATA STRUCTURES ---
struct TelemetryMessage {
    uint32_t id;
    uint8_t len;
    uint8_t data[8];
    unsigned long timestamp;
};

// --- FUNCTION PROTOTYPES ---
// Setup
void setupPins();
void setupModem();
void setupMQTT();
void setupCAN();

// Loop / Tasks
void MQTT_And_Sensor_Loop();
void CAN_RX_Task(void *pvParameters);
void Sensor_Task(void *pvParameters);
void MQTT_Task(void *pvParameters); // Unified MQTT handler
bool getFastGPS();

#endif