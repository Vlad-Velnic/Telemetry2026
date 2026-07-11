#include "includes.h"

// Shared Data
volatile float gps_lat = 0.0;
volatile float gps_lon = 0.0;
volatile float gps_speed = 0.0;
volatile int currentGear = 0;
volatile uint32_t rearMqttQueueDrops = 0;
volatile uint32_t rearMqttPublishFailures = 0;
volatile uint32_t rearCanTxFailures = 0;
volatile bool rearCanReady = false;
volatile bool rearMqttConnected = false;

// Queue for outgoing MQTT messages (CAN data)
QueueHandle_t mqttQueue;
QueueHandle_t lapGpsQueue;

void sendCanMessage(uint32_t id, uint8_t* data, size_t length) {
    if (length > 8) length = 8;
    twai_message_t txMsg = {};
    txMsg.identifier = id;
    txMsg.extd = 0; 
    txMsg.data_length_code = length;
    memcpy(txMsg.data, data, length);
    if (!rearCanReady || twai_transmit(&txMsg, pdMS_TO_TICKS(5)) != ESP_OK) {
        rearCanTxFailures++;
    }
}

void broadcastData(uint32_t id, uint8_t* data, size_t len) {
    if (len > 8) len = 8;

    // 1. Send to CAN (So Front Module can log it to SD)
    twai_message_t txMsg = {};
    txMsg.identifier = id;
    txMsg.extd = 0;
    txMsg.data_length_code = len;
    memcpy(txMsg.data, data, len);
    if (!rearCanReady || twai_transmit(&txMsg, pdMS_TO_TICKS(5)) != ESP_OK) {
        rearCanTxFailures++;
    }

    // 2. Send to MQTT Queue (So we send it to cloud)
    TelemetryMessage tMsg;
    tMsg.id = id;
    tMsg.len = len;
    tMsg.timestamp = millis();
    memcpy(tMsg.data, data, len);
    
    // Never stall acquisition on a full network queue.
    if (!mqttQueue || xQueueSend(mqttQueue, &tMsg, 0) != pdTRUE) {
        rearMqttQueueDrops++;
    }
}

static uint16_t saturateU16(uint32_t value) {
    return value > 0xFFFF ? 0xFFFF : (uint16_t)value;
}

void sendHealthFrame() {
    static uint8_t heartbeat = 0;

    uint16_t queueDrops = saturateU16(rearMqttQueueDrops);
    uint16_t publishFailures = saturateU16(rearMqttPublishFailures);
    uint8_t queueFree = mqttQueue ? min((UBaseType_t)255, uxQueueSpacesAvailable(mqttQueue)) : 0;
    uint8_t flags = 0;

    if (rearMqttQueueDrops > 0) flags |= 0x01;
    if (!rearCanReady) flags |= 0x02;
    if (rearMqttPublishFailures > 0) flags |= 0x04;
    if (rearMqttConnected) flags |= 0x08;
#if ENABLE_OTA
    if (WiFi.status() == WL_CONNECTED) flags |= 0x10;
    if (otaReady) flags |= 0x20;
#endif

    uint8_t healthMsg[8] = {
        HEALTH_NODE_REAR,
        flags,
        (uint8_t)((queueDrops >> 8) & 0xFF),
        (uint8_t)(queueDrops & 0xFF),
        (uint8_t)((publishFailures >> 8) & 0xFF),
        (uint8_t)(publishFailures & 0xFF),
        queueFree,
        heartbeat++
    };

    broadcastData(CAN_ID_SYSTEM_HEALTH, healthMsg, 8);
}

int getGear() {
    // Returns 1-5 if a gear pin is active, otherwise neutral.
    if (!digitalRead(PIN_GEAR_1)) return 1;
    if (!digitalRead(PIN_GEAR_2)) return 2;
    if (!digitalRead(PIN_GEAR_3)) return 3;
    if (!digitalRead(PIN_GEAR_4)) return 4;
    if (!digitalRead(PIN_GEAR_5)) return 5;
    if (!digitalRead(PIN_GEAR_0)) return 0;
    return 0;
}

static bool gnssDegreesMinutesToDecimal(float rawCoordinate,
                                        float maximumDegrees,
                                        float &decimalDegrees) {
    if (!isfinite(rawCoordinate)) return false;

    const float magnitude = fabsf(rawCoordinate);
    const float degrees = floorf(magnitude / 100.0f);
    const float minutes = magnitude - degrees * 100.0f;
    if (degrees > maximumDegrees || minutes < 0.0f || minutes >= 60.0f) {
        return false;
    }

    decimalDegrees = degrees + minutes / 60.0f;
    if (rawCoordinate < 0.0f) decimalDegrees = -decimalDegrees;
    return true;
}

// --- GPS HELPER ---
bool getFastGPS() {
    uint8_t fixStatus = 0;
    float rawLatitude = 0.0f;
    float rawLongitude = 0.0f;
    float speedKnots = 0.0f;
    if (!modem.getGPS(&fixStatus, &rawLatitude, &rawLongitude, &speedKnots) ||
        (fixStatus != 1 && fixStatus != 2 && fixStatus != 3)) {
        return false;
    }

    float decimalLatitude = 0.0f;
    float decimalLongitude = 0.0f;
    if (!gnssDegreesMinutesToDecimal(rawLatitude, 90.0f, decimalLatitude) ||
        !gnssDegreesMinutesToDecimal(rawLongitude, 180.0f, decimalLongitude) ||
        !isfinite(speedKnots) || speedKnots < 0.0f) {
        return false;
    }

    gps_lat = decimalLatitude;
    gps_lon = decimalLongitude;
    gps_speed = speedKnots * 1.852f;

    return true;
}

// --- TASK: SENSOR READING ---
void Sensor_Task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(SENSOR_PERIOD_MS);
    unsigned long lastHealthSend = 0;
    int stableGear = 0;
    int candidateGear = 0;
    uint8_t candidateSamples = 0;

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        unsigned long currentMillis = millis();

        // A. Read Analog (Rear Dampers + Brake)
        int dL = analogRead(PIN_DAMPER_RL);
        int dR = analogRead(PIN_DAMPER_RR);
        int bP = analogRead(PIN_BRAKE_PRESS);

        uint8_t analogMsg[6];
        analogMsg[0] = (dL >> 8) & 0xFF; analogMsg[1] = dL & 0xFF;
        analogMsg[2] = (dR >> 8) & 0xFF; analogMsg[3] = dR & 0xFF;
        analogMsg[4] = (bP >> 8) & 0xFF; analogMsg[5] = bP & 0xFF;
        
        broadcastData(CAN_ID_REAR_ANALOG, analogMsg, 6);

        // B. Read Gear
        const int rawGear = getGear();
        if (rawGear == candidateGear) {
            if (candidateSamples < 2) candidateSamples++;
        } else {
            candidateGear = rawGear;
            candidateSamples = 1;
        }
        if (candidateSamples >= 2) stableGear = candidateGear;
        currentGear = stableGear;

        uint8_t gearMsg[1] = { (uint8_t)stableGear };
        broadcastData(CAN_ID_GEAR, gearMsg, 1);

        if (currentMillis - lastHealthSend >= HEALTH_PERIOD_MS) {
            lastHealthSend = currentMillis;
            sendHealthFrame();
        }
    }
}

// --- TASK: MQTT UPLOAD & GPS (Lower Priority) ---
void MQTT_Task(void *pvParameters) {
    unsigned long lastGpsRead = 0;
    TelemetryMessage msg;
    char payload[224];
    char record[48];
    char hexData[20];

    while (1) {
        // --- GPS READING (1Hz) ---
        if (millis() - lastGpsRead > 1000) {
            lastGpsRead = millis();
            const uint32_t gpsQueryStart = millis();
            if(getFastGPS()) {
                const uint32_t gpsSampleTime =
                    gpsQueryStart + (millis() - gpsQueryStart) / 2;
                GPSPoint point = {
                    (double)gps_lat,
                    (double)gps_lon,
                    gps_speed,
                    gpsSampleTime
                };

                // Keep lap timing independent from CAN and MQTT availability.
                if (lapGpsQueue && xQueueSend(lapGpsQueue, &point, 0) != pdTRUE) {
                    GPSPoint discardedPoint;
                    xQueueReceive(lapGpsQueue, &discardedPoint, 0);
                    xQueueSend(lapGpsQueue, &point, 0);
                }
                
                // Pack Lat/Lon (2 floats = 8 bytes)
                uint8_t posMsg[8];
                memcpy(&posMsg[0], (const void*)&gps_lat, 4);
                memcpy(&posMsg[4], (const void*)&gps_lon, 4);
                broadcastData(CAN_ID_GPS_POS, posMsg, 8);

                // Pack Speed (1 float = 4 bytes)
                uint8_t spdMsg[4];
                memcpy(&spdMsg[0], (const void*)&gps_speed, 4);
                broadcastData(CAN_ID_GPS_SPD, spdMsg, 4);
            }
        }

        // --- MQTT CONNECTION ---
        if (!mqtt.connected()) {
            rearMqttConnected = false;
            // Bound TinyGSM's otherwise long default TCP connection attempt so
            // broker failure cannot stop the next GNSS sample for tens of seconds.
            if (!client.connected() && !client.connect(mqtt_server, 1883, 1)) {
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
            if (mqtt.connect("RearModuleIdentifier")) {
                rearMqttConnected = true;
                // This is a live dashboard: discard an outage backlog rather
                // than presenting old samples as current after reconnect.
                xQueueReset(mqttQueue);
                DEBUG_PRINTLN("MQTT Connected");
            } else {
                vTaskDelay(pdMS_TO_TICKS(100)); // Retry next loop
                continue;
            }
        }
        rearMqttConnected = true;
        mqtt.loop();

        // --- MQTT UPLOAD PROCESSING ---
        // Batch several records into one publish. Sending every CAN frame as a
        // separate modem AT transaction cannot reliably sustain the normal
        // front + rear frame rate.
        int count = 0;
        size_t payloadLength = 0;
        payload[0] = '\0';
        while (count < 6 && xQueueReceive(mqttQueue, &msg, 0) == pdTRUE) {
            // Convert data to Hex
            hexData[0] = '\0';
            for (int i = 0; i < msg.len; i++) {
                sprintf(hexData + (i*2), "%02X", msg.data[i]);
            }

            const int recordLength = snprintf(record, sizeof(record),
                                              "%lu,%X,%s",
                                              msg.timestamp, msg.id, hexData);
            if (recordLength <= 0 || recordLength >= (int)sizeof(record) ||
                payloadLength + (count > 0 ? 1 : 0) + recordLength >= sizeof(payload)) {
                rearMqttQueueDrops++;
                continue;
            }

            if (count > 0) payload[payloadLength++] = ';';
            memcpy(payload + payloadLength, record, (size_t)recordLength);
            payloadLength += (size_t)recordLength;
            payload[payloadLength] = '\0';
            count++;
        }

        if (count > 0 && !mqtt.publish(mqtt_topic, payload)) {
            rearMqttPublishFailures++;
        }

        // Yield to let IDLE tasks run and watchdog kick
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

#if ENABLE_OTA
// --- TASK: OTA UPDATE HANDLER (Lowest Priority) ---
void OTA_Task(void *pvParameters) {
    unsigned long lastReconnectAttempt = 0;

    while (1) {
        if (WiFi.status() == WL_CONNECTED) {
            if (!otaReady) {
                setupOTA();
            } else {
                ArduinoOTA.handle();
            }

            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        otaReady = false;

        if (millis() - lastReconnectAttempt >= OTA_RECONNECT_INTERVAL_MS) {
            lastReconnectAttempt = millis();
            setupOTA();
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
#endif

// --- TASK: CAN LISTENER ---
void CAN_RX_Task(void *pvParameters) {
    twai_message_t rxMsg;
    TelemetryMessage log;

    DEBUG_PRINTLN("--- CAN LISTENER STARTED ---");

    uint8_t messagesSinceYield = 0;
    while (1) {
        // Wait for message (blocking)
        esp_err_t result = twai_receive(&rxMsg, pdMS_TO_TICKS(1000)); // Timeout 1 sec

        if (result == ESP_OK) {
            if (rxMsg.extd || rxMsg.rtr || rxMsg.dlc_non_comp ||
                rxMsg.data_length_code > 8) {
                rearMqttQueueDrops++;
                continue;
            }
            DEBUG_PRINTF("RX ID: 0x%X | Len: %d\n", rxMsg.identifier, rxMsg.data_length_code);
            
            log.id = rxMsg.identifier;
            log.len = rxMsg.data_length_code;
            log.timestamp = millis();
            memcpy(log.data, rxMsg.data, rxMsg.data_length_code);
            
            if (!mqttQueue || xQueueSend(mqttQueue, &log, 0) != pdTRUE) {
                rearMqttQueueDrops++;
                DEBUG_PRINTLN("Error: MQTT queue is full, dropping CAN message!");
            }

            if (++messagesSinceYield >= 32) {
                messagesSinceYield = 0;
                taskYIELD();
            }
        } 
        else if (result == ESP_ERR_TIMEOUT) {
            // Just silence on the bus
            // DEBUG_PRINTLN("Waiting for data...");
        }
        else {
            DEBUG_PRINTF("CAN ERROR: %s (Code: 0x%X)\n", esp_err_to_name(result), result);
            
            twai_status_info_t status_info;
            if (twai_get_status_info(&status_info) == ESP_OK) {
                DEBUG_PRINTF("Status: State=%d, TX_Err=%d, RX_Err=%d, Bus_Err=%d\n",
                    status_info.state, status_info.tx_error_counter, status_info.rx_error_counter, status_info.bus_error_count);
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        uint32_t alerts = 0;
        if (twai_read_alerts(&alerts, 0) == ESP_OK) {
            if (alerts & TWAI_ALERT_RX_QUEUE_FULL) rearMqttQueueDrops++;
            if (alerts & TWAI_ALERT_TX_FAILED) rearCanTxFailures++;
            if (alerts & TWAI_ALERT_BUS_OFF) {
                rearCanReady = false;
                if (twai_initiate_recovery() != ESP_OK) rearCanTxFailures++;
            }
            if (alerts & TWAI_ALERT_BUS_RECOVERED) {
                if (twai_start() == ESP_OK) rearCanReady = true;
            }
        }
    }
}

// --- TASK: LAP TIME CALCULATOR ---
namespace {
constexpr double GEOMETRY_EPSILON = 1e-12;

int orientation(const GPSPoint &p, const GPSPoint &q, const GPSPoint &r) {
    const double value = (q.lon - p.lon) * (r.lat - p.lat) -
                         (q.lat - p.lat) * (r.lon - p.lon);
    const double scale = fabs(q.lon - p.lon) + fabs(q.lat - p.lat) +
                         fabs(r.lon - p.lon) + fabs(r.lat - p.lat) + 1.0;

    if (fabs(value) <= GEOMETRY_EPSILON * scale) {
        return 0;
    }
    return value > 0.0 ? 1 : 2;
}

bool onSegment(const GPSPoint &p, const GPSPoint &q, const GPSPoint &r) {
    return q.lon <= fmax(p.lon, r.lon) + GEOMETRY_EPSILON &&
           q.lon >= fmin(p.lon, r.lon) - GEOMETRY_EPSILON &&
           q.lat <= fmax(p.lat, r.lat) + GEOMETRY_EPSILON &&
           q.lat >= fmin(p.lat, r.lat) - GEOMETRY_EPSILON;
}

bool doIntersect(const GPSPoint &p1, const GPSPoint &q1,
                 const GPSPoint &gateL, const GPSPoint &gateR) {
    const int o1 = orientation(p1, q1, gateL);
    const int o2 = orientation(p1, q1, gateR);
    const int o3 = orientation(gateL, gateR, p1);
    const int o4 = orientation(gateL, gateR, q1);

    if (o1 != o2 && o3 != o4) {
        return true;
    }
    if (o1 == 0 && onSegment(p1, gateL, q1)) return true;
    if (o2 == 0 && onSegment(p1, gateR, q1)) return true;
    if (o3 == 0 && onSegment(gateL, p1, gateR)) return true;
    if (o4 == 0 && onSegment(gateL, q1, gateR)) return true;
    return false;
}

bool getIntersectionTime(const GPSPoint &prev, const GPSPoint &curr,
                         const GPSPoint &gateL, const GPSPoint &gateR,
                         uint32_t &intersectionTime) {
    if (!doIntersect(prev, curr, gateL, gateR)) {
        return false;
    }

    const double x1 = prev.lon, y1 = prev.lat;
    const double x2 = curr.lon, y2 = curr.lat;
    const double x3 = gateL.lon, y3 = gateL.lat;
    const double x4 = gateR.lon, y4 = gateR.lat;
    const double denom = (x1 - x2) * (y3 - y4) -
                         (y1 - y2) * (x3 - x4);

    // Collinear overlap does not provide a unique crossing instant.
    if (fabs(denom) <= GEOMETRY_EPSILON) {
        return false;
    }

    const double interLon = ((x1 * y2 - y1 * x2) * (x3 - x4) -
                             (x1 - x2) * (x3 * y4 - y3 * x4)) / denom;
    const double interLat = ((x1 * y2 - y1 * x2) * (y3 - y4) -
                             (y1 - y2) * (x3 * y4 - y3 * x4)) / denom;
    const double segmentLon = x2 - x1;
    const double segmentLat = y2 - y1;
    const double segmentLengthSquared = segmentLon * segmentLon +
                                        segmentLat * segmentLat;

    if (segmentLengthSquared <= GEOMETRY_EPSILON) {
        return false;
    }

    double fraction = ((interLon - x1) * segmentLon +
                       (interLat - y1) * segmentLat) /
                      segmentLengthSquared;
    if (fraction < -GEOMETRY_EPSILON || fraction > 1.0 + GEOMETRY_EPSILON) {
        return false;
    }
    fraction = fmax(0.0, fmin(1.0, fraction));

    const uint32_t samplePeriod = curr.timestamp - prev.timestamp;
    if (samplePeriod == 0) {
        return false;
    }

    intersectionTime = prev.timestamp +
                       (uint32_t)(fraction * (double)samplePeriod + 0.5);
    return true;
}

bool lapGateIsValid(const GPSPoint &gateL, const GPSPoint &gateR) {
    return isfinite(gateL.lat) && isfinite(gateL.lon) &&
           isfinite(gateR.lat) && isfinite(gateR.lon) &&
           fabs(gateL.lat) <= 90.0 && fabs(gateR.lat) <= 90.0 &&
           fabs(gateL.lon) <= 180.0 && fabs(gateR.lon) <= 180.0 &&
           (fabs(gateL.lat - gateR.lat) > GEOMETRY_EPSILON ||
            fabs(gateL.lon - gateR.lon) > GEOMETRY_EPSILON);
}

int gateCrossingDirection(const GPSPoint &prev, const GPSPoint &curr,
                          const GPSPoint &gateL, const GPSPoint &gateR) {
    const double gateLon = gateR.lon - gateL.lon;
    const double gateLat = gateR.lat - gateL.lat;
    const double prevSide = gateLon * (prev.lat - gateL.lat) -
                            gateLat * (prev.lon - gateL.lon);
    const double currSide = gateLon * (curr.lat - gateL.lat) -
                            gateLat * (curr.lon - gateL.lon);

    if (prevSide < -GEOMETRY_EPSILON && currSide > GEOMETRY_EPSILON) return 1;
    if (prevSide > GEOMETRY_EPSILON && currSide < -GEOMETRY_EPSILON) return -1;
    return 0;
}
} // namespace

void LapTime_Task(void *pvParameters) {
    const GPSPoint gateL = { LAP_GATE_LEFT_LAT, LAP_GATE_LEFT_LON, 0.0f, 0 };
    const GPSPoint gateR = { LAP_GATE_RIGHT_LAT, LAP_GATE_RIGHT_LON, 0.0f, 0 };

    if (!LAP_TIMING_ENABLED || !lapGpsQueue || !lapGateIsValid(gateL, gateR)) {
        DEBUG_PRINTLN("Lap timing disabled: configure a valid start/finish gate");
        vTaskDelete(NULL);
        return;
    }

    GPSPoint prevPoint;
    GPSPoint currentPoint;
    bool havePreviousPoint = false;
    bool timingStarted = false;
    uint32_t lapStartIntersection = 0;
    int expectedCrossingDirection = 0;

    while (1) {
        if (xQueueReceive(lapGpsQueue, &currentPoint, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (!havePreviousPoint) {
            prevPoint = currentPoint;
            havePreviousPoint = true;
            continue;
        }

        const uint32_t sampleGap = currentPoint.timestamp - prevPoint.timestamp;
        if (sampleGap == 0 || sampleGap > LAP_MAX_SAMPLE_GAP_MS) {
            prevPoint = currentPoint;
            continue;
        }

        const float averageSpeed = (prevPoint.speed + currentPoint.speed) / 2.0f;
        const int crossingDirection = gateCrossingDirection(
            prevPoint, currentPoint, gateL, gateR);
        uint32_t intersectionTime = 0;
        if (isfinite(averageSpeed) &&
            averageSpeed >= LAP_MIN_CROSSING_SPEED_KMH &&
            crossingDirection != 0 &&
            (expectedCrossingDirection == 0 ||
             crossingDirection == expectedCrossingDirection) &&
            getIntersectionTime(prevPoint, currentPoint, gateL, gateR,
                                intersectionTime)) {
            if (!timingStarted) {
                lapStartIntersection = intersectionTime;
                timingStarted = true;
                expectedCrossingDirection = crossingDirection;
                DEBUG_PRINTLN("Lap timer armed at start/finish crossing");
            } else {
                const uint32_t lapTimeMs = intersectionTime - lapStartIntersection;

                // Reject repeated intersections caused by GPS jitter near the gate.
                if (lapTimeMs >= LAP_MIN_TIME_MS) {
                    lapStartIntersection = intersectionTime;

                    uint8_t lapTimeMessage[4] = {
                        (uint8_t)((lapTimeMs >> 24) & 0xFF),
                        (uint8_t)((lapTimeMs >> 16) & 0xFF),
                        (uint8_t)((lapTimeMs >> 8) & 0xFF),
                        (uint8_t)(lapTimeMs & 0xFF)
                    };

                    broadcastData(CAN_ID_LAPTIME, lapTimeMessage, 4);
                    DEBUG_PRINTF("Lap completed: %lu ms\n",
                                 (unsigned long)lapTimeMs);
                }
            }
        }

        prevPoint = currentPoint;
    }
}
