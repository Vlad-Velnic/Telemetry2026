#include "includes.h"

// Centralized function for CAN sending and SD logging
void broadcastData(uint32_t id, uint8_t* data, size_t len) {
    if (len > 8) len = 8;

    // 1. Physically send on the CAN bus
    twai_message_t txMsg = {};
    txMsg.identifier = id;
    txMsg.extd = 0;
    txMsg.data_length_code = len;
    memcpy(txMsg.data, data, len);
    if (!frontCanReady || twai_transmit(&txMsg, pdMS_TO_TICKS(5)) != ESP_OK) {
        frontCanTxFailures++;
    }

    // 2. Send to queue for SD logging
    LogMessage log;
    log.id = id;
    log.len = len;
    log.timestamp = millis();
    log.isRx = false; // Message sent locally
    memcpy(log.data, data, len);
    
    if (!canQueue || xQueueSend(canQueue, &log, 0) != pdTRUE) {
        frontCanQueueDrops++;
    }
}

// --- TASK: CAN RECEIVER ---
void CAN_Task(void *pvParameters) {
    twai_message_t rxMsg;
    LogMessage log;

    while (1) {
        const esp_err_t receiveResult = twai_receive(&rxMsg, pdMS_TO_TICKS(100));

        uint32_t alerts = 0;
        if (twai_read_alerts(&alerts, 0) == ESP_OK) {
            if (alerts & TWAI_ALERT_RX_QUEUE_FULL) frontCanQueueDrops++;
            if (alerts & TWAI_ALERT_TX_FAILED) frontCanTxFailures++;
            if (alerts & TWAI_ALERT_BUS_OFF) {
                frontCanReady = false;
                if (twai_initiate_recovery() != ESP_OK) frontCanTxFailures++;
            }
            if (alerts & TWAI_ALERT_BUS_RECOVERED) {
                if (twai_start() == ESP_OK) frontCanReady = true;
            }
        }

        if (receiveResult == ESP_OK) {
            // This project uses only standard Classical-CAN data frames.
            if (rxMsg.extd || rxMsg.rtr || rxMsg.dlc_non_comp ||
                rxMsg.data_length_code > 8) {
                frontCanQueueDrops++;
                continue;
            }
            const unsigned long rxTimestamp = millis();
            
            // 1. Log EVERYTHING received
            log.id = rxMsg.identifier;
            log.len = rxMsg.data_length_code;
            log.timestamp = rxTimestamp;
            log.isRx = true;
            memcpy(log.data, rxMsg.data, rxMsg.data_length_code);
            if (xQueueSend(canQueue, &log, 0) != pdTRUE) {
                frontCanQueueDrops++;
            }

            if (rxMsg.identifier == CAN_ID_GEAR && rxMsg.data_length_code >= 1)
                lastGearCanRxMs = rxTimestamp;
            if (rxMsg.identifier == CAN_ID_GPS_POS && rxMsg.data_length_code >= 8) {
                lastGpsPositionCanRxMs = rxTimestamp;
            }

            // 2. Update Display Variables (Only if relevant)
            if (rxMsg.identifier == CAN_ID_GEAR && rxMsg.data_length_code >= 1) {
                currentGear = rxMsg.data[0];
            }
            else if (rxMsg.identifier == CAN_ID_GPS_SPD &&
                     rxMsg.data_length_code >= sizeof(float)) {
                float speed = 0.0f;
                memcpy(&speed, rxMsg.data, sizeof(speed));
                currentGpsSpeed = speed;
            }
            else if (rxMsg.identifier == CAN_ID_LAPTIME && rxMsg.data_length_code >= 4) {
                lastLapTime = ((uint32_t)rxMsg.data[0] << 24) | ((uint32_t)rxMsg.data[1] << 16) | ((uint32_t)rxMsg.data[2] << 8) | (uint32_t)rxMsg.data[3];
            }
        } else if (receiveResult != ESP_ERR_TIMEOUT) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// --- TASK: SD WRITER ---
void SD_Task(void *pvParameters) {
    LogMessage msg;
    char hexData[20];
    char line[64];
    const int BATCH_SIZE = 20; 
    int batchCount = 0;
    unsigned long lastMountAttempt = 0;

    File logFile;
    if (frontSdReady) logFile = SD.open(logFileName, FILE_APPEND);

    while (1) {
        if (xQueueReceive(canQueue, &msg, pdMS_TO_TICKS(500))) {
            if (!frontSdReady && millis() - lastMountAttempt >= 2000) {
                lastMountAttempt = millis();
                mountSDStorage();
            }
            if (frontSdReady && !logFile) {
                logFile = SD.open(logFileName, FILE_APPEND);
                if (!logFile) frontSdReady = false;
            }
            
            if (logFile) {
                // Convert data to Hex String
                hexData[0] = '\0';
                for (int i = 0; i < msg.len; i++) {
                    sprintf(hexData + (i*2), "%02X", msg.data[i]);
                }

                // Format: timestamp, id, data
                const int lineLength = snprintf(line, sizeof(line), "%lu,%X,%s\n",
                                                msg.timestamp, msg.id, hexData);
                const bool writeOk = lineLength > 0 && lineLength < (int)sizeof(line) &&
                                     logFile.write((const uint8_t *)line,
                                                   (size_t)lineLength) == (size_t)lineLength;

                if (!writeOk || logFile.getWriteError()) {
                    frontSdWriteFailures++;
                    logFile.close();
                    SD.end();
                    frontSdReady = false;
                    batchCount = 0;
                    continue;
                }

                batchCount++;
                if (batchCount >= BATCH_SIZE) {
                    logFile.flush();
                    if (logFile.getWriteError()) {
                        frontSdWriteFailures++;
                        logFile.close();
                        SD.end();
                        frontSdReady = false;
                    }
                    batchCount = 0;
                }
            } else {
                frontSdWriteFailures++;
            }
        } else {
            // Periodic flush if idle
            if (logFile) {
                logFile.flush();
                if (logFile.getWriteError()) {
                    frontSdWriteFailures++;
                    logFile.close();
                    SD.end();
                    frontSdReady = false;
                    batchCount = 0;
                }
            }
        }
    }
}

void updateDisplay(uint8_t currentGear, unsigned long lastLapTime,
                   float gpsSpeed)
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

    display.setTextSize(1);

    if (NO_GPS)
    {
        display.setCursor(50, 25);
        display.printf("G");
    }

    if (NO_REAR)
    {
        display.setCursor(60, 25);
        display.printf("R");
    }

    // GPS speed is retained even while stale; the G marker communicates age.
    display.setCursor(50, 34);
    display.printf("%.1f km/h", gpsSpeed);

    display.display();
}
