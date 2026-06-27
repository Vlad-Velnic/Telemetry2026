#include "includes.h"

void setup() {
    DEBUG_BEGIN(115200);
    DEBUG_PRINTLN("Booting Rear Module...");

    // Increased queue size to handle bursty CAN traffic and high latency
    mqttQueue = xQueueCreate(200, sizeof(TelemetryMessage));

    setupPins();
    setupModem();
    setupMQTT();
    setupCAN();
#if ENABLE_OTA
    setupOTA();
#endif
    
    // Core 0 Task: Listen to external CAN messages
    xTaskCreatePinnedToCore(CAN_RX_Task, "CAN_RX", 4096, NULL, 5, NULL, 0);

    // Core 1 Task: Dedicated sensor reading at SENSOR_FREQ_HZ (Higher priority)
    xTaskCreatePinnedToCore(Sensor_Task, "Sensor", 4096, NULL, 4, NULL, 1);

    // Core 1 Task: MQTT connection, upload processing, and GPS (Blocking/Lower priority)
    xTaskCreatePinnedToCore(MQTT_Task, "MQTT", 8192, NULL, 2, NULL, 1);

#if ENABLE_OTA
    // Core 0 Task: Handle Wi-Fi OTA updates without blocking vehicle telemetry tasks
    xTaskCreatePinnedToCore(OTA_Task, "OTA", 4096, NULL, 1, NULL, 0);
#endif
}

void loop() {
    // Delete the Arduino loop task, as we're using custom FreeRTOS tasks now
    vTaskDelete(NULL);
}
