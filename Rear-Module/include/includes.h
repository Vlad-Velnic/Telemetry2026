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
#define HEALTH_PERIOD_MS 5000

// --- LAP TIMING CONFIGURATION ---
// Set this to 1 after replacing the four gate coordinates below.
// Coordinates must use the same decimal-degree convention as gps_lat/gps_lon.
#define LAP_TIMING_ENABLED 0
static constexpr double LAP_GATE_LEFT_LAT = 0.0;
static constexpr double LAP_GATE_LEFT_LON = 0.0;
static constexpr double LAP_GATE_RIGHT_LAT = 0.0;
static constexpr double LAP_GATE_RIGHT_LON = 0.0;
static constexpr uint32_t LAP_MIN_TIME_MS = 10000;
static constexpr uint32_t LAP_MAX_SAMPLE_GAP_MS = 2500;
static constexpr float LAP_MIN_CROSSING_SPEED_KMH = 5.0f;
static constexpr uint8_t LAP_GPS_QUEUE_LENGTH = 8;

// Health frame node IDs
#define HEALTH_NODE_FRONT 1
#define HEALTH_NODE_REAR 2

// --- GLOBAL OBJECTS ---
extern TinyGsm modem;
extern TinyGsmClient client;
extern PubSubClient mqtt;

// --- SHARED VARIABLES ---
extern volatile float gps_lat;
extern volatile float gps_lon;
extern volatile float gps_speed;
extern volatile int currentGear;
extern volatile uint32_t rearMqttQueueDrops;
extern volatile uint32_t rearMqttPublishFailures;
extern volatile uint32_t rearCanTxFailures;
extern volatile bool rearCanReady;
extern volatile bool rearMqttConnected;
#if ENABLE_OTA
extern volatile bool otaReady;
#endif

// --- QUEUES ---
extern QueueHandle_t mqttQueue;
extern QueueHandle_t lapGpsQueue;

// --- DATA STRUCTURES ---
struct TelemetryMessage {
    uint32_t id;
    uint8_t len;
    uint8_t data[8];
    unsigned long timestamp;
};

struct GPSPoint {
    double lat;
    double lon;
    float speed;
    uint32_t timestamp;
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
void LapTime_Task(void *pvParameters); // Calculate lap time based on CAN messages
#if ENABLE_OTA
void OTA_Task(void *pvParameters);
#endif
void sendHealthFrame();
bool getFastGPS();
void broadcastData(uint32_t id, uint8_t* data, size_t len);
int getGear();
void sendCanMessage(uint32_t id, uint8_t* data, size_t length);

#endif
