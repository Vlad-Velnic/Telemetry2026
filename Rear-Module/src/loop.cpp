#include "includes.h"

// Shared Data
volatile float gps_lat = 0.0;
volatile float gps_lon = 0.0;
volatile float gps_speed = 0.0;
volatile int currentGear = 0;

// Queue for outgoing MQTT messages (CAN data)
QueueHandle_t mqttQueue;

void sendCanMessage(uint32_t id, uint8_t* data, size_t length) {
    if (length > 8) length = 8;
    twai_message_t txMsg;
    txMsg.identifier = id;
    txMsg.extd = 0; 
    txMsg.data_length_code = length;
    memcpy(txMsg.data, data, length);
    twai_transmit(&txMsg, pdMS_TO_TICKS(5));
}

void broadcastData(uint32_t id, uint8_t* data, size_t len) {
    if (len > 8) len = 8;

    // 1. Send to CAN (So Front Module can log it to SD)
    twai_message_t txMsg;
    txMsg.identifier = id;
    txMsg.extd = 0;
    txMsg.data_length_code = len;
    memcpy(txMsg.data, data, len);
    twai_transmit(&txMsg, pdMS_TO_TICKS(5));

    // 2. Send to MQTT Queue (So we send it to cloud)
    TelemetryMessage tMsg;
    tMsg.id = id;
    tMsg.len = len;
    tMsg.timestamp = millis();
    memcpy(tMsg.data, data, len);
    
    xQueueSend(mqttQueue, &tMsg, 0);
}


int getGear() {
    // Returns 1-6 if pin active, 0 if none (Neutral)
    if (!digitalRead(PIN_GEAR_1)) return 1;
    if (!digitalRead(PIN_GEAR_2)) return 2;
    if (!digitalRead(PIN_GEAR_3)) return 3;
    if (!digitalRead(PIN_GEAR_4)) return 4;
    if (!digitalRead(PIN_GEAR_5)) return 5;
    if (!digitalRead(PIN_GEAR_0)) return 0;
    return 0;
}

// --- GPS HELPER (From original code) ---
bool getFastGPS() {
    // Direct AT command access for speed
    while (Serial1.available()) Serial1.read();
    Serial1.println("AT+CGNSSINFO");
    
    String response = "";
    unsigned long start = millis();
    while (millis() - start < 200) { 
        if (Serial1.available()) response += (char)Serial1.read();
    }
    response.trim(); 

    if (response.indexOf("+CGNSSINFO:") == -1) return false;
    if (response.indexOf(",,,,,,,,") != -1) return false; 

    // Simple parsing logic
    int commaIndex[16];
    int commaCount = 0;
    int searchStart = response.indexOf(": ") + 2;
    
    for (int i = searchStart; i < response.length(); i++) {
        if (response.charAt(i) == ',') {
            commaIndex[commaCount++] = i;
            if (commaCount >= 13) break; 
        }
    }

    if (commaCount < 12) return false;

    String latStr = response.substring(commaIndex[4] + 1, commaIndex[5]);
    String nsStr  = response.substring(commaIndex[5] + 1, commaIndex[6]);
    String lonStr = response.substring(commaIndex[6] + 1, commaIndex[7]);
    String ewStr  = response.substring(commaIndex[7] + 1, commaIndex[8]);
    String spdStr = response.substring(commaIndex[11] + 1, commaIndex[12]);

    if (latStr.length() < 2 || lonStr.length() < 2) return false;

    gps_lat = latStr.toFloat();
    gps_lon = lonStr.toFloat();
    gps_speed = spdStr.toFloat();

    if (nsStr == "S") gps_lat *= -1;
    if (ewStr == "W") gps_lon *= -1;

    return true;
}

// --- TASK: CAN LISTENER ---
void CAN_RX_Task(void *pvParameters) {
    twai_message_t rxMsg;
    TelemetryMessage log;

    Serial.println("--- CAN LISTENER STARTED ---");

    while (1) {
        // Așteaptă un mesaj (blocant)
        esp_err_t result = twai_receive(&rxMsg, pdMS_TO_TICKS(1000)); // Timeout 1 sec

        if (result == ESP_OK) {
            Serial.printf("RX ID: 0x%X | Len: %d\n", rxMsg.identifier, rxMsg.data_length_code);
            
            // Trimite spre SD
            log.id = rxMsg.identifier;
            log.len = rxMsg.data_length_code;
            log.timestamp = millis();
            memcpy(log.data, rxMsg.data, rxMsg.data_length_code);
            
            if (xQueueSend(mqttQueue, &log, 0) != pdTRUE) {
                Serial.println("Eroare: Coada mqtt este plina!");
            }
        } 
        else if (result == ESP_ERR_TIMEOUT) {
            // Nu e eroare, doar liniste pe fir
            Serial.println("Waiting for data..."); 
        }
        else {
            // Aici vedem erorile fizice!
            Serial.printf("CAN ERROR: %s (Code: 0x%X)\n", esp_err_to_name(result), result);
            
            // Verificam starea magistralei
            twai_status_info_t status_info;
            twai_get_status_info(&status_info);
            Serial.printf("Status: State=%d, TX_Err=%d, RX_Err=%d, Bus_Err=%d\n", 
                status_info.state, status_info.tx_error_counter, status_info.rx_error_counter, status_info.bus_error_count);
        }
    }
}

// --- MAIN LOOP FUNCTION: SENSORS & MQTT PUBLISH ---
// We handle MQTT in the main loop to avoid stack overflow on small tasks with heavy networking
void MQTT_And_Sensor_Loop() {
    static unsigned long lastSensorRead = 0;
    static unsigned long lastGpsRead = 0;
    
    // --- MQTT CONNECTION ---
    if (!mqtt.connected()) {
        if (mqtt.connect("RearModuleIdentifier")) {
            Serial.println("MQTT Connected");
        } else {
            delay(100); 
            return; // Retry next loop
        }
    }
    mqtt.loop();

    // --- SENSOR READING (20Hz) ---
    if (millis() - lastSensorRead > 50) {
        lastSensorRead = millis();

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
        int newGear = getGear();
        // Broadcast gear continuously or on change? 
        // Let's do continuously @ 20Hz for reliability
        uint8_t gearMsg[1] = { (uint8_t)newGear };
        broadcastData(CAN_ID_GEAR, gearMsg, 1);
    }

    // --- GPS READING (1Hz) ---
    if (millis() - lastGpsRead > 1000) {
        lastGpsRead = millis();
        if(getFastGPS()) { // If we got a valid fix
            
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

    // --- MQTT UPLOAD PROCESSING ---
    // Empty the queue and send to cloud in "timestamp,id,data" format
    TelemetryMessage msg;
    char payload[64];
    char hexData[20];

    // Process up to 10 messages per loop to avoid blocking too long
    int count = 0;
    while (xQueueReceive(mqttQueue, &msg, 0) && count < 10) {
        
        // Convert data to Hex
        hexData[0] = '\0';
        for (int i = 0; i < msg.len; i++) {
            sprintf(hexData + (i*2), "%02X", msg.data[i]);
        }

        // UNIFIED FORMAT: timestamp,id,data
        // Example: "12500,500,AABB1122"
        sprintf(payload, "%lu,%X,%s", msg.timestamp, msg.id, hexData);
        
        mqtt.publish(mqtt_topic, payload);
        count++;
    }
}