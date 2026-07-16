#include "includes.h"

// Shared Data
volatile float gps_lat = 0.0;
volatile float gps_lon = 0.0;
volatile float gps_speed = 0.0;
volatile int currentGear = 0;
volatile uint32_t rearMqttQueueDrops = 0;
volatile uint32_t rearMqttPublishFailures = 0;
volatile uint32_t rearCanTxFailures = 0;
volatile uint32_t rearGpsMissedDeadlines = 0;
volatile uint32_t rearGpsQueueLosses = 0;
volatile uint32_t rearGpsAssistanceFailures = 0;
volatile uint32_t rearGpsValidEpochs = 0;
volatile uint32_t rearGpsDuplicateEpochs = 0;
volatile uint32_t rearGpsParserFailures = 0;
volatile uint32_t rearGpsEpochGapCount = 0;
volatile uint32_t rearGpsMaxQueryMs = 0;
volatile bool rearCanReady = false;
volatile bool rearMqttConnected = false;
volatile bool rearModemReady = false;
volatile bool rearGpsHasFix = false;
volatile uint8_t gpsRateHz = 1;
volatile uint32_t nextGpsDeadlineMs = 0;

QueueHandle_t lapGpsQueue;
SemaphoreHandle_t modemMutex;
SemaphoreHandle_t telemetryMutex;

namespace {
constexpr size_t GPS_TELEMETRY_CAPACITY = 2;
constexpr size_t GEAR_TELEMETRY_CAPACITY = 4;

template <typename T, size_t Capacity>
class FixedQueue {
 public:
    bool push(const T &value) {
        bool dropped = false;
        if (count_ == Capacity) {
            head_ = (head_ + 1) % Capacity;
            count_--;
            dropped = true;
        }
        values_[(head_ + count_) % Capacity] = value;
        count_++;
        return dropped;
    }

    size_t size() const { return count_; }
    size_t free() const { return Capacity - count_; }
    const T &at(size_t index) const { return values_[(head_ + index) % Capacity]; }

    void clear() {
        head_ = 0;
        count_ = 0;
    }

    void discardThrough(uint32_t sequence) {
        while (count_ > 0 && at(0).sequence <= sequence) {
            head_ = (head_ + 1) % Capacity;
            count_--;
        }
    }

    uint32_t lastSequence() const {
        return count_ == 0 ? 0 : at(count_ - 1).sequence;
    }

 private:
    T values_[Capacity] = {};
    size_t head_ = 0;
    size_t count_ = 0;
};

struct GpsTelemetryPair {
    TelemetryMessage position;
    TelemetryMessage speed;
    uint32_t sequence;
};

struct LatestSlot {
    TelemetryMessage message = {};
    uint32_t generation = 0;
    bool valid = false;
    bool dirty = false;
};

struct TelemetryState {
    FixedQueue<GpsTelemetryPair, GPS_TELEMETRY_CAPACITY> gps;
    FixedQueue<TelemetryMessage, GEAR_TELEMETRY_CAPACITY> gear;

    LatestSlot currentGear;
    LatestSlot currentGpsPosition;
    LatestSlot currentGpsSpeed;
    LatestSlot accel;
    LatestSlot gyro;
    LatestSlot lap;

    uint32_t nextSequence = 0;
    uint32_t lastGearQueuedAt = 0;
    bool online = false;
    bool snapshotPending = false;
};

struct BatchCommit {
    uint32_t gpsSequence = 0;
    uint32_t gearSequence = 0;
    uint32_t accelGeneration = 0;
    uint32_t gyroGeneration = 0;
    uint32_t lapGeneration = 0;
    bool accelSent = false;
    bool gyroSent = false;
    bool lapSent = false;
    bool snapshot = false;
};

TelemetryState telemetryState;

static void updateSlot(LatestSlot &slot, const TelemetryMessage &message, bool dirty) {
    slot.message = message;
    slot.generation++;
    slot.valid = true;
    if (dirty) slot.dirty = true;
}

static TelemetryMessage makeMessage(uint32_t id, const uint8_t *data,
                                    size_t len, uint32_t timestamp) {
    TelemetryMessage message = {};
    message.id = id;
    message.len = min(len, (size_t)8);
    message.timestamp = timestamp;
    if (message.len > 0) memcpy(message.data, data, message.len);
    return message;
}

static void clearContinuousLocked() {
    telemetryState.gps.clear();
    telemetryState.gear.clear();
}

static void setTelemetryOnline(bool online) {
    if (!telemetryMutex || xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(20)) != pdTRUE) {
        rearMqttQueueDrops++;
        return;
    }
    if (telemetryState.online != online) {
        clearContinuousLocked();
        telemetryState.online = online;
        telemetryState.snapshotPending = online;
    }
    xSemaphoreGive(telemetryMutex);
}

static bool acceptFrontCanMessage(uint32_t id) {
    return id == CAN_ID_ACCEL || id == CAN_ID_GYRO;
}

static void submitTelemetry(uint32_t id, const uint8_t *data, size_t len,
                            uint32_t timestamp) {
    if (!telemetryMutex || xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(2)) != pdTRUE) {
        rearMqttQueueDrops++;
        return;
    }

    TelemetryMessage message = makeMessage(id, data, len, timestamp);
    message.sequence = ++telemetryState.nextSequence;

    if (id == CAN_ID_ACCEL) {
        updateSlot(telemetryState.accel, message, true);
    } else if (id == CAN_ID_GYRO) {
        updateSlot(telemetryState.gyro, message, true);
    } else if (id == CAN_ID_GEAR && len >= 1) {
        const bool changed = !telemetryState.currentGear.valid ||
                             telemetryState.currentGear.message.data[0] != data[0];
        updateSlot(telemetryState.currentGear, message, false);
        if (telemetryState.online &&
            (changed || timestamp - telemetryState.lastGearQueuedAt >= MQTT_GEAR_REFRESH_MS)) {
            telemetryState.lastGearQueuedAt = timestamp;
            if (telemetryState.gear.push(message)) rearMqttQueueDrops++;
        }
    } else if (id == CAN_ID_LAPTIME) {
        updateSlot(telemetryState.lap, message, true);
    }

    xSemaphoreGive(telemetryMutex);
}

static void submitGpsTelemetry(const GPSPoint &point) {
    const float latitude = (float)point.lat;
    const float longitude = (float)point.lon;
    uint8_t positionData[8];
    uint8_t speedData[4];
    memcpy(positionData, &latitude, sizeof(latitude));
    memcpy(positionData + sizeof(latitude), &longitude, sizeof(longitude));
    memcpy(speedData, &point.speed, sizeof(point.speed));

    if (!telemetryMutex || xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(2)) != pdTRUE) {
        rearMqttQueueDrops++;
        return;
    }

    GpsTelemetryPair pair = {};
    pair.sequence = ++telemetryState.nextSequence;
    pair.position = makeMessage(CAN_ID_GPS_POS, positionData, sizeof(positionData),
                                point.timestamp);
    pair.speed = makeMessage(CAN_ID_GPS_SPD, speedData, sizeof(speedData),
                             point.timestamp);
    pair.position.sequence = pair.sequence;
    pair.speed.sequence = pair.sequence;
    updateSlot(telemetryState.currentGpsPosition, pair.position, false);
    updateSlot(telemetryState.currentGpsSpeed, pair.speed, false);
    if (telemetryState.online && telemetryState.gps.push(pair)) {
        rearMqttQueueDrops++;
    }
    xSemaphoreGive(telemetryMutex);
}

static int formatRecord(const TelemetryMessage &message, char *record, size_t size) {
    char hexData[17] = {};
    for (uint8_t i = 0; i < message.len; ++i) {
        snprintf(hexData + i * 2, sizeof(hexData) - i * 2, "%02X", message.data[i]);
    }
    return snprintf(record, size, "%lu,%X,%s", message.timestamp,
                    (unsigned int)message.id, hexData);
}

static bool appendRecord(char *payload, size_t &payloadLength,
                         const TelemetryMessage &message) {
    char record[48];
    const int recordLength = formatRecord(message, record, sizeof(record));
    if (recordLength <= 0 || recordLength >= (int)sizeof(record)) return false;
    const size_t separator = payloadLength > 0 ? 1 : 0;
    if (payloadLength + separator + (size_t)recordLength >= MQTT_PAYLOAD_BUFFER_SIZE) {
        return false;
    }
    if (separator) payload[payloadLength++] = ';';
    memcpy(payload + payloadLength, record, (size_t)recordLength);
    payloadLength += (size_t)recordLength;
    payload[payloadLength] = '\0';
    return true;
}

static bool appendGpsPair(char *payload, size_t &payloadLength,
                          const GpsTelemetryPair &pair) {
    char positionRecord[48];
    char speedRecord[48];
    const int positionLength = formatRecord(pair.position, positionRecord,
                                            sizeof(positionRecord));
    const int speedLength = formatRecord(pair.speed, speedRecord, sizeof(speedRecord));
    if (positionLength <= 0 || speedLength <= 0 ||
        positionLength >= (int)sizeof(positionRecord) ||
        speedLength >= (int)sizeof(speedRecord)) {
        return false;
    }
    const size_t separators = payloadLength > 0 ? 2 : 1;
    if (payloadLength + separators + (size_t)positionLength + (size_t)speedLength >=
        MQTT_PAYLOAD_BUFFER_SIZE) {
        return false;
    }
    if (payloadLength > 0) payload[payloadLength++] = ';';
    memcpy(payload + payloadLength, positionRecord, (size_t)positionLength);
    payloadLength += (size_t)positionLength;
    payload[payloadLength++] = ';';
    memcpy(payload + payloadLength, speedRecord, (size_t)speedLength);
    payloadLength += (size_t)speedLength;
    payload[payloadLength] = '\0';
    return true;
}

static bool slotIsFresh(const LatestSlot &slot, uint32_t now, uint32_t maxAge) {
    return slot.valid && now - slot.message.timestamp <= maxAge;
}

static size_t prepareBatch(char *payload, BatchCommit &commit) {
    payload[0] = '\0';
    size_t payloadLength = 0;
    if (!telemetryMutex || xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(20)) != pdTRUE) {
        return 0;
    }

    const uint32_t now = millis();
    commit.snapshot = telemetryState.snapshotPending;

    auto appendSlot = [&](LatestSlot &slot, bool allow, bool &sent,
                          uint32_t &generation) {
        if (!allow || !appendRecord(payload, payloadLength, slot.message)) return;
        sent = true;
        generation = slot.generation;
    };

    // Persistent event first.
    appendSlot(telemetryState.lap, telemetryState.lap.valid &&
               (telemetryState.lap.dirty || commit.snapshot), commit.lapSent,
               commit.lapGeneration);

    if (commit.snapshot) {
        bool ignored = false;
        uint32_t ignoredGeneration = 0;
        appendSlot(telemetryState.currentGear,
                   slotIsFresh(telemetryState.currentGear, now, 1500),
                   ignored, ignoredGeneration);
        ignored = false;
        appendSlot(telemetryState.currentGpsPosition,
                   slotIsFresh(telemetryState.currentGpsPosition, now, 3000),
                   ignored, ignoredGeneration);
        ignored = false;
        appendSlot(telemetryState.currentGpsSpeed,
                   slotIsFresh(telemetryState.currentGpsSpeed, now, 3000),
                   ignored, ignoredGeneration);
        ignored = false;
        appendSlot(telemetryState.accel,
                   slotIsFresh(telemetryState.accel, now, 1000),
                   commit.accelSent, commit.accelGeneration);
        appendSlot(telemetryState.gyro,
                   slotIsFresh(telemetryState.gyro, now, 1000),
                   commit.gyroSent, commit.gyroGeneration);

        commit.gpsSequence = telemetryState.gps.lastSequence();
        commit.gearSequence = telemetryState.gear.lastSequence();
        xSemaphoreGive(telemetryMutex);
        return payloadLength;
    }

    for (size_t i = 0; i < telemetryState.gear.size(); ++i) {
        const TelemetryMessage &message = telemetryState.gear.at(i);
        if (!appendRecord(payload, payloadLength, message)) break;
        commit.gearSequence = message.sequence;
    }
    for (size_t i = 0; i < telemetryState.gps.size(); ++i) {
        const GpsTelemetryPair &pair = telemetryState.gps.at(i);
        if (!appendGpsPair(payload, payloadLength, pair)) break;
        commit.gpsSequence = pair.sequence;
    }
    appendSlot(telemetryState.accel,
               telemetryState.accel.valid && telemetryState.accel.dirty,
               commit.accelSent, commit.accelGeneration);
    appendSlot(telemetryState.gyro,
               telemetryState.gyro.valid && telemetryState.gyro.dirty,
               commit.gyroSent, commit.gyroGeneration);

    xSemaphoreGive(telemetryMutex);
    return payloadLength;
}

static void clearSlotIfSent(LatestSlot &slot, bool sent, uint32_t generation) {
    if (sent && slot.generation == generation) slot.dirty = false;
}

static void commitBatch(const BatchCommit &commit) {
    if (!telemetryMutex || xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(20)) != pdTRUE) {
        rearMqttQueueDrops++;
        return;
    }
    telemetryState.gps.discardThrough(commit.gpsSequence);
    telemetryState.gear.discardThrough(commit.gearSequence);
    clearSlotIfSent(telemetryState.accel, commit.accelSent, commit.accelGeneration);
    clearSlotIfSent(telemetryState.gyro, commit.gyroSent, commit.gyroGeneration);
    clearSlotIfSent(telemetryState.lap, commit.lapSent, commit.lapGeneration);
    if (commit.snapshot) telemetryState.snapshotPending = false;
    xSemaphoreGive(telemetryMutex);
}

static bool gpsDeadlineImminent() {
    const int32_t untilDeadline = (int32_t)(nextGpsDeadlineMs - millis());
    return untilDeadline >= 0 && untilDeadline < (int32_t)MQTT_GPS_GUARD_MS;
}

static bool takeModem(uint32_t timeoutMs) {
    return modemMutex && xSemaphoreTake(modemMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
}

static void giveModem() {
    if (modemMutex) xSemaphoreGive(modemMutex);
}
}  // namespace

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
    const uint32_t timestamp = millis();
    sendCanMessage(id, data, len);
    if (id != CAN_ID_FRONT_ANALOG && id != CAN_ID_REAR_ANALOG) {
        submitTelemetry(id, data, len, timestamp);
    }
}

int getGear() {
    // Returns 1-6 if pin active, 0 if none (Neutral)
    if (!digitalRead(PIN_GEAR_0)) return 1;
    if (!digitalRead(PIN_GEAR_1)||!digitalRead(PIN_GEAR_2)) return 2;
    if (!digitalRead(PIN_GEAR_3)||!digitalRead(PIN_GEAR_4)) return 3;
    if (!digitalRead(PIN_GEAR_5)) return 4;
    // if (!digitalRead(PIN_GEAR_0)) return 0;
    return 0;
}

// --- GPS HELPER ---
bool getFastGPS(GPSPoint &point) {
    uint8_t fixStatus = 0;
    float rawLatitude = 0.0f;
    float rawLongitude = 0.0f;
    float speedKnots = 0.0f;
    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    int second = 0;
    int millisecond = 0;
    const bool queryOk = modem.getGPSWithMilliseconds(
        &fixStatus, &rawLatitude, &rawLongitude, &speedKnots, &year, &month,
        &day, &hour, &minute, &second, &millisecond);
    if (!queryOk) {
        // No-fix replies report status zero and are expected during acquisition.
        if (fixStatus == 2 || fixStatus == 3) rearGpsParserFailures++;
        return false;
    }
    if (fixStatus != 2 && fixStatus != 3) {
        return false;
    }

    // A76xx +CGNSSINFO reports signed decimal degrees on this modem variant.
    // TinyGSM already applies the N/S and E/W signs, so no NMEA ddmm conversion
    // belongs here. This point is the canonical value for lap timing, CAN, and MQTT.
    if (!isfinite(rawLatitude) || fabsf(rawLatitude) > 90.0f ||
        !isfinite(rawLongitude) || fabsf(rawLongitude) > 180.0f ||
        !isfinite(speedKnots) || speedKnots < 0.0f ||
        day < 1 || day > 31 || hour < 0 || hour > 23 ||
        minute < 0 || minute > 59 || second < 0 || second > 60 ||
        millisecond < 0 || millisecond > 999) {
        rearGpsParserFailures++;
        return false;
    }

    gps_lat = rawLatitude;
    gps_lon = rawLongitude;
    gps_speed = speedKnots * 1.852f;

    point.lat = rawLatitude;
    point.lon = rawLongitude;
    point.speed = gps_speed;
    point.epochKey =
        (((((uint32_t)day * 24U + (uint32_t)hour) * 60U +
           (uint32_t)minute) * 60U + (uint32_t)second) * 1000U) +
        (uint32_t)millisecond;

    return true;
}

// --- TASK: SENSOR READING ---
void Sensor_Task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(SENSOR_PERIOD_MS);
    int stableGear = 0;
    int candidateGear = 0;
    uint8_t candidateSamples = 0;

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
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

    }
}

// --- TASK: GPS ACQUISITION ---
void GPS_Task(void *pvParameters) {
    const uint8_t configuredRateHz = gpsRateHz;
    const uint32_t periodMs = 1000UL / (configuredRateHz > 0 ? configuredRateHz : 1);
    const TickType_t frequency = pdMS_TO_TICKS(periodMs);
    TickType_t lastWake = xTaskGetTickCount();
    uint32_t lastEpochKey = UINT32_MAX;
    uint32_t lastAcceptedAt = 0;
    uint32_t lastDiagnosticAt = millis();
    nextGpsDeadlineMs = millis() + periodMs;

    while (1) {
        vTaskDelayUntil(&lastWake, frequency);
        const TickType_t wakeNow = xTaskGetTickCount();
        if ((TickType_t)(wakeNow - lastWake) >= frequency) {
            const uint32_t missed = max(
                (uint32_t)1,
                (uint32_t)((wakeNow - lastWake) / max((TickType_t)1, frequency)));
            rearGpsMissedDeadlines += missed;
            lastWake = wakeNow;
            nextGpsDeadlineMs = millis() + periodMs;
            continue;
        }
        const uint32_t queryStart = millis();
        nextGpsDeadlineMs = queryStart + periodMs;

        GPSPoint point = {};
        const uint32_t mutexWaitMs = periodMs > 20 ? periodMs - 20 : periodMs;
        if (!takeModem(mutexWaitMs)) {
            rearGpsMissedDeadlines++;
            continue;
        }
        const bool validFix = getFastGPS(point);
        giveModem();
        const uint32_t queryDuration = millis() - queryStart;
        if (queryDuration > rearGpsMaxQueryMs) rearGpsMaxQueryMs = queryDuration;

        if (validFix) {
            point.timestamp = queryStart + (millis() - queryStart) / 2;
            rearGpsHasFix = true;

            const bool newEpoch = point.epochKey != lastEpochKey;
            if (!newEpoch) {
                rearGpsDuplicateEpochs++;
            } else {
                lastEpochKey = point.epochKey;
                if (lastAcceptedAt != 0) {
                    const uint32_t sampleGap = point.timestamp - lastAcceptedAt;
                    if (sampleGap > periodMs + periodMs / 2U) {
                        const uint32_t missing =
                            max((uint32_t)1, sampleGap / periodMs - 1U);
                        rearGpsEpochGapCount += missing;
                    }
                }
                lastAcceptedAt = point.timestamp;
                rearGpsValidEpochs++;

                const bool lapGateConfigured = LAP_TIMING_ENABLED &&
                    isfinite(LAP_GATE_LEFT_LAT) && isfinite(LAP_GATE_LEFT_LON) &&
                    isfinite(LAP_GATE_RIGHT_LAT) && isfinite(LAP_GATE_RIGHT_LON) &&
                    (fabs(LAP_GATE_LEFT_LAT - LAP_GATE_RIGHT_LAT) > 1e-12 ||
                     fabs(LAP_GATE_LEFT_LON - LAP_GATE_RIGHT_LON) > 1e-12);
                if (lapGateConfigured &&
                    xQueueSend(lapGpsQueue, &point, 0) != pdTRUE) {
                    GPSPoint discardedPoint;
                    xQueueReceive(lapGpsQueue, &discardedPoint, 0);
                    xQueueSend(lapGpsQueue, &point, 0);
                    rearGpsQueueLosses++;
                }

                const float latitude = (float)point.lat;
                const float longitude = (float)point.lon;
                uint8_t positionData[8];
                uint8_t speedData[4];
                memcpy(positionData, &latitude, sizeof(latitude));
                memcpy(positionData + sizeof(latitude), &longitude, sizeof(longitude));
                memcpy(speedData, &point.speed, sizeof(point.speed));
                sendCanMessage(CAN_ID_GPS_POS, positionData, sizeof(positionData));
                sendCanMessage(CAN_ID_GPS_SPD, speedData, sizeof(speedData));
                submitGpsTelemetry(point);
            }
        }

        const TickType_t now = xTaskGetTickCount();
        if ((TickType_t)(now - lastWake) >= frequency) {
            const uint32_t missed = max((uint32_t)1,
                (uint32_t)((now - lastWake) / max((TickType_t)1, frequency)));
            rearGpsMissedDeadlines += missed;
            lastWake = now;
            nextGpsDeadlineMs = millis() + periodMs;
        }

        if (millis() - lastDiagnosticAt >= 5000U) {
            lastDiagnosticAt = millis();
            DEBUG_PRINTF(
                "GPS epochs=%lu duplicates=%lu missed=%lu gaps=%lu parser=%lu maxQuery=%lums\n",
                (unsigned long)rearGpsValidEpochs,
                (unsigned long)rearGpsDuplicateEpochs,
                (unsigned long)rearGpsMissedDeadlines,
                (unsigned long)rearGpsEpochGapCount,
                (unsigned long)rearGpsParserFailures,
                (unsigned long)rearGpsMaxQueryMs);
        }
    }
}

// --- TASK: LTE/MQTT STATE MACHINE ---
void MQTT_Task(void *pvParameters) {
    bool networkConfigured = false;
    bool gprsReady = false;
    bool agpsAttempted = false;
    bool mqttOnline = false;
    uint8_t connectionFailures = 0;
    uint32_t lastNetworkAttempt = 0;
    uint32_t lastConnectAttempt = 0;
    uint32_t lastMqttLoop = 0;
    uint32_t lastPublish = 0;
    char payload[MQTT_PAYLOAD_BUFFER_SIZE];

    setTelemetryOnline(false);

    while (1) {
        const uint32_t now = millis();

        if (!networkConfigured && !gpsDeadlineImminent() && takeModem(200)) {
            modem.sendAT("+CNMP=38");
            networkConfigured = modem.waitResponse(1000L) == 1;
            giveModem();
            if (!networkConfigured) vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if (!gprsReady) {
            rearMqttConnected = false;
            mqttOnline = false;
            setTelemetryOnline(false);
            if (now - lastNetworkAttempt < 1000 || gpsDeadlineImminent()) {
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }
            lastNetworkAttempt = now;

            bool networkReady = false;
            if (takeModem(500)) {
                networkReady = modem.isNetworkConnected();
                giveModem();
            }
            if (!networkReady) {
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }

            if (takeModem(5000)) {
                gprsReady = modem.gprsConnect(apn, user, pass);
                giveModem();
            }
            if (!gprsReady) {
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }
            DEBUG_PRINTLN("GPRS connected");
        }

        if (!agpsAttempted) {
            agpsAttempted = true;
            if (!rearGpsHasFix && takeModem(1000)) {
                modem.sendAT("+CAGPS");
                const bool accepted = modem.waitResponse(2000L) == 1;
                const bool assisted = accepted &&
                    modem.waitResponse(9000L, "+AGPS: success.", "+AGPS:") == 1;
                giveModem();
                if (!assisted) rearGpsAssistanceFailures++;
                DEBUG_PRINTLN(assisted ? "A-GPS assistance loaded" :
                                           "A-GPS unavailable; using standalone GNSS");
            }
        }

        if (!mqttOnline) {
            rearMqttConnected = false;
            setTelemetryOnline(false);
            if (now - lastConnectAttempt < 1000 || gpsDeadlineImminent()) {
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }
            lastConnectAttempt = now;

            bool connected = false;
            if (takeModem(1500)) {
                if (!client.connected()) client.connect(mqtt_server, 1883, 1);
                connected = client.connected() && mqtt.connect("RearModuleIdentifier");
                giveModem();
            }
            if (!connected) {
                if (++connectionFailures >= 3) {
                    connectionFailures = 0;
                    gprsReady = false;
                    agpsAttempted = false;
                }
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }

            connectionFailures = 0;
            mqttOnline = true;
            rearMqttConnected = true;
            setTelemetryOnline(true);
            lastMqttLoop = now;
            lastPublish = now - MQTT_PUBLISH_PERIOD_MS;
            DEBUG_PRINTLN("MQTT connected");
        }

        if (now - lastMqttLoop >= MQTT_LOOP_PERIOD_MS && !gpsDeadlineImminent()) {
            bool loopOk = false;
            if (takeModem(300)) {
                loopOk = mqtt.loop();
                giveModem();
            }
            lastMqttLoop = now;
            if (!loopOk) {
                mqttOnline = false;
                rearMqttConnected = false;
                setTelemetryOnline(false);
                continue;
            }
        }

        if (now - lastPublish >= MQTT_PUBLISH_PERIOD_MS && !gpsDeadlineImminent()) {
            BatchCommit commit = {};
            const size_t payloadLength = prepareBatch(payload, commit);
            lastPublish = now;
            if (payloadLength > 0) {
                bool published = false;
                if (takeModem(500)) {
                    published = mqtt.publish(mqtt_topic, payload);
                    giveModem();
                }
                if (published) {
                    commitBatch(commit);
                } else {
                    rearMqttPublishFailures++;
                    mqttOnline = false;
                    rearMqttConnected = false;
                    setTelemetryOnline(false);
                }
            }
        }

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
            if (acceptFrontCanMessage(rxMsg.identifier)) {
                submitTelemetry(rxMsg.identifier, rxMsg.data,
                                rxMsg.data_length_code, millis());
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
