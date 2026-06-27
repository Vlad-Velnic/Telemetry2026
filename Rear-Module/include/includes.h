#ifndef INCLUDES_H
#define INCLUDES_H

#include <Arduino.h>
#include "pinout.h"
#include "debug.h"
#include "canIDs.h"

// --- LIBRARIES ---
#define TINY_GSM_MODEM_A7670
#define ENABLE_OTA 1

#if ENABLE_OTA
#include <WiFi.h>
#include <ArduinoOTA.h>
#endif

#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include "driver/twai.h" // Native CAN driver

// --- CONFIGURATION ---
extern const char apn[];
extern const char user[];
extern const char pass[];
extern const char* mqtt_server;
extern const char* mqtt_topic;

#if ENABLE_OTA
// --- OTA CONFIGURATION ---
// Create a phone/laptop hotspot with these credentials, then upload to
// "rear-module-ota" through PlatformIO/Arduino OTA when the module is online.
static constexpr char OTA_WIFI_SSID[] = "TelemetryOTA";
static constexpr char OTA_WIFI_PASSWORD[] = "telemetry2026";
static constexpr char OTA_HOSTNAME[] = "rear-module-ota";
static constexpr char OTA_UPDATE_PASSWORD[] = "rear2026ota";
static constexpr uint32_t OTA_CONNECT_TIMEOUT_MS = 8000;
static constexpr uint32_t OTA_RECONNECT_INTERVAL_MS = 10000;
#endif

// --- TIMING CONSTANTS ---
#define SENSOR_FREQ_HZ 25
#define SENSOR_PERIOD_MS (1000 / SENSOR_FREQ_HZ)

// --- GLOBAL OBJECTS ---
extern TinyGsm modem;
extern TinyGsmClient client;
extern PubSubClient mqtt;

// --- SHARED VARIABLES ---
extern volatile float gps_lat;
extern volatile float gps_lon;
extern volatile float gps_speed;
extern volatile int currentGear;
#if ENABLE_OTA
extern volatile bool otaReady;
#endif

// --- QUEUES ---
extern QueueHandle_t mqttQueue;

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
#if ENABLE_OTA
void setupOTA();
#endif

// Loop / Tasks
void CAN_RX_Task(void *pvParameters);
void Sensor_Task(void *pvParameters);
void MQTT_Task(void *pvParameters); // Unified MQTT handler
#if ENABLE_OTA
void OTA_Task(void *pvParameters);
#endif
bool getFastGPS();
void broadcastData(uint32_t id, uint8_t* data, size_t len);
int getGear();
void sendCanMessage(uint32_t id, uint8_t* data, size_t length);

#endif
