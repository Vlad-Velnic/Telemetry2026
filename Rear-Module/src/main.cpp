#include "includes.h"

void setup() {
    DEBUG_BEGIN(115200);
    DEBUG_PRINTLN("Booting Rear Module...");

    // Increased queue size to handle bursty CAN traffic and high latency
    mqttQueue = xQueueCreate(200, sizeof(TelemetryMessage));
    lapGpsQueue = xQueueCreate(LAP_GPS_QUEUE_LENGTH, sizeof(GPSPoint));
    if (!mqttQueue || !lapGpsQueue) {
        DEBUG_PRINTLN("Telemetry queue allocation failed");
    }

    setupPins();
    setupCAN();

    // Start vehicle-critical acquisition before any potentially slow network
    // initialization. CAN and local sensing must work with the modem absent.
    // Core 0 Task: Listen to external CAN messages
    if (rearCanReady && mqttQueue) {
        xTaskCreatePinnedToCore(CAN_RX_Task, "CAN_RX", 4096, NULL, 4, NULL, 0);
    }

    // Core 1 Task: Dedicated sensor reading at SENSOR_FREQ_HZ (Higher priority)
    if (mqttQueue) {
        xTaskCreatePinnedToCore(Sensor_Task, "Sensor", 4096, NULL, 4, NULL, 1);
    }

    setupModem();
    setupMQTT();
#if ENABLE_OTA
    setupOTA();
#endif

    // Core 1 Task: MQTT connection, upload processing, and GPS (Blocking/Lower priority)
    if (mqttQueue && lapGpsQueue) {
        xTaskCreatePinnedToCore(MQTT_Task, "MQTT", 8192, NULL, 2, NULL, 1);
    }

    // Core 0 Task: Calculate lap time from GPS gate crossings
    if (lapGpsQueue) {
        xTaskCreatePinnedToCore(LapTime_Task, "LapTime", 4096, NULL, 3, NULL, 0);
    }

#if ENABLE_OTA
    // Core 0 Task: Handle Wi-Fi OTA updates without blocking vehicle telemetry tasks
    xTaskCreatePinnedToCore(OTA_Task, "OTA", 4096, NULL, 1, NULL, 0);
#endif
}

void loop() {
    // Delete the Arduino loop task, as we're using custom FreeRTOS tasks now
    vTaskDelete(NULL);
}
