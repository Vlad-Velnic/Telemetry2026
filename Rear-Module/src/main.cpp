#include "includes.h"

// Queue Definition
extern QueueHandle_t mqttQueue;

void setup() {
    DEBUG_BEGIN(115200);
    DEBUG_PRINTLN("Booting Rear Module...");

    mqttQueue = xQueueCreate(50, sizeof(TelemetryMessage));

    setupPins();
    setupModem();
    setupMQTT();
    setupCAN();
    
    xTaskCreatePinnedToCore(CAN_RX_Task, "CAN_RX", 4096, NULL, 5, NULL, 0);
}

void loop() {
    MQTT_And_Sensor_Loop();
}