#include "includes.h"

// Centralized function for CAN sending and SD logging
void broadcastData(uint32_t id, uint8_t* data, size_t len) {
    if (len > 8) len = 8;

    // 1. Physically send on the CAN bus
    twai_message_t txMsg;
    txMsg.identifier = id;
    txMsg.extd = 0;
    txMsg.data_length_code = len;
    memcpy(txMsg.data, data, len);
    if (twai_transmit(&txMsg, pdMS_TO_TICKS(5)) != ESP_OK) {
        frontCanTxFailures++;
    }

    // 2. Send to queue for SD logging
    LogMessage log;
    log.id = id;
    log.len = len;
    log.timestamp = millis();
    log.isRx = false; // Message sent locally
    memcpy(log.data, data, len);
    
    if (xQueueSend(canQueue, &log, 0) != pdTRUE) {
        frontCanQueueDrops++;
    }
}

static uint16_t saturateU16(uint32_t value) {
    return value > 0xFFFF ? 0xFFFF : (uint16_t)value;
}

void sendHealthFrame() {
    static uint8_t heartbeat = 0;

    uint16_t queueDrops = saturateU16(frontCanQueueDrops);
    uint16_t txFailures = saturateU16(frontCanTxFailures);
    uint8_t queueFree = canQueue ? min((UBaseType_t)255, uxQueueSpacesAvailable(canQueue)) : 0;
    uint8_t flags = 0;

    if (frontCanQueueDrops > 0) flags |= 0x01;
    if (frontCanTxFailures > 0) flags |= 0x02;

    uint8_t healthMsg[8] = {
        HEALTH_NODE_FRONT,
        flags,
        (uint8_t)((queueDrops >> 8) & 0xFF),
        (uint8_t)(queueDrops & 0xFF),
        (uint8_t)((txFailures >> 8) & 0xFF),
        (uint8_t)(txFailures & 0xFF),
        queueFree,
        heartbeat++
    };

    broadcastData(CAN_ID_SYSTEM_HEALTH, healthMsg, 8);
}

// --- TASK: CAN RECEIVER ---
void CAN_Task(void *pvParameters) {
    twai_message_t rxMsg;
    LogMessage log;

    while (1) {
        if (twai_receive(&rxMsg, portMAX_DELAY) == ESP_OK) {
            
            // 1. Log EVERYTHING received
            log.id = rxMsg.identifier;
            log.len = rxMsg.data_length_code;
            log.timestamp = millis();
            log.isRx = true;
            memcpy(log.data, rxMsg.data, rxMsg.data_length_code);
            if (xQueueSend(canQueue, &log, 0) != pdTRUE) {
                frontCanQueueDrops++;
            }

            // 2. Update Display Variables (Only if relevant)
            if (rxMsg.identifier == CAN_ID_RPM && rxMsg.data_length_code >= 2) {
                currentRPM = (rxMsg.data[0] << 8) | rxMsg.data[1];
            }
            else if (rxMsg.identifier == CAN_ID_VOLTAGE) {
                currentBat = (float)rxMsg.data[0] / 10.0;
            }
            else if (rxMsg.identifier == CAN_ID_WATER_TEMP) {
                currentTemp = (float)rxMsg.data[0];
            }
            else if (rxMsg.identifier == CAN_ID_GEAR) {
                currentGear = rxMsg.data[0];
            }
        }
    }
}

// --- TASK: SD WRITER ---
void SD_Task(void *pvParameters) {
    LogMessage msg;
    char hexData[20];
    const int BATCH_SIZE = 20; 
    int batchCount = 0;

    // Wait for setupSD to set the filename
    vTaskDelay(pdMS_TO_TICKS(1000));

    File logFile = SD.open(logFileName, FILE_APPEND);

    while (1) {
        if (xQueueReceive(canQueue, &msg, pdMS_TO_TICKS(500))) {
            if (!logFile) logFile = SD.open(logFileName, FILE_APPEND);
            
            if (logFile) {
                // Convert data to Hex String
                hexData[0] = '\0';
                for (int i = 0; i < msg.len; i++) {
                    sprintf(hexData + (i*2), "%02X", msg.data[i]);
                }

                // Format: timestamp, id, data
                logFile.printf("%lu,%X,%s\n", msg.timestamp, msg.id, hexData);

                batchCount++;
                if (batchCount >= BATCH_SIZE) {
                    logFile.flush();
                    batchCount = 0;
                }
            }
        } else {
            // Periodic flush if idle
            if (logFile) {
                logFile.flush();
            }
        }
    }
}

void updateDisplay(u_int8_t currentGear, unsigned long lastLapTime, float currentTemp, float currentBatteryVoltage, int currentRPM)
{
    display.clearDisplay();

    // Large gear number on left
    display.setTextSize(8);
    display.setCursor(0, 5);
    if (currentGear == 0 && !NO_REAR)
        display.print("N");
    else
        display.print(currentGear);

    // Right side time
    display.setTextSize(2);

    // Time at top right
    unsigned long totalMs = lastLapTime;
    unsigned int mins = (totalMs / 60000) % 60;
    unsigned int secs = (totalMs / 1000) % 60;
    unsigned int tenths = (totalMs / 100) % 10;

    display.setCursor(50, 8);
    display.printf("%01d:%02d:%d", mins, secs, tenths);

    // Right side temp and voltage
    display.setTextSize(1);

    if (NO_WIFI)
    {
        display.setCursor(55, 30);
        display.printf("W");
    }

    if (NO_REAR)
    {
        display.setCursor(65, 30);
        display.printf("R");
    }

    if (NO_ECU)
    {
        display.setCursor(75, 30);
        display.printf("M");
    }

    // Temperature
    display.setCursor(100, 42);
    display.printf("%.0f%cC", currentTemp, 247);

    // RPM
    display.setCursor(51, 42);
    display.printf("%d", currentRPM);

    // Battery voltage
    display.setCursor(95, 57);
    display.printf("%.1fV", currentBatteryVoltage);

    display.display();
}
