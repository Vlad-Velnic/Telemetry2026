#define TINY_GSM_MODEM_A7670
#include <Arduino.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>

// --- CONFIGURATION ---
const char apn[] = "net";  // Orange RO. Use "live.vodafone.com" for Vodafone
const char user[] = "";
const char pass[] = "";

// MQTT Details (Public Broker)
const char* mqtt_server = "broker.hivemq.com"; 
const char* mqtt_topic = "lilygo/gps/data";

// --- HARDWARE PINS (LILYGO T-CALL V1.0/1.1) ---
#define MODEM_TX      26
#define MODEM_RX      25
#define MODEM_PWRKEY  4
#define MODEM_DTR     14
#define MODEM_RI      13
#define MODEM_FLIGHT  25
#define MODEM_STATUS  34
#define BAT_ADC       35
#define BOARD_POWERON_PIN 12

#define SerialAT Serial1
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

// Variables to store fast GPS data
float fast_lat = 0.0;
float fast_lon = 0.0;
float fast_speed = 0.0;

// --- HELPERS ---
float readBattery() {
    return ((float)analogRead(BAT_ADC) / 4095.0) * 2.0 * 3.3 * 1.1;
}

void setupModem() {
    pinMode(BOARD_POWERON_PIN, OUTPUT); digitalWrite(BOARD_POWERON_PIN, HIGH);
    pinMode(MODEM_PWRKEY, OUTPUT);
    
    // Power Cycle
    digitalWrite(MODEM_PWRKEY, LOW); delay(100);
    digitalWrite(MODEM_PWRKEY, HIGH); delay(1000);
    digitalWrite(MODEM_PWRKEY, LOW);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    // Not listening in this example, just sending
}

void reconnect() {
    while (!mqtt.connected()) {
        Serial.print("Connecting to MQTT...");
        String clientId = "ESP32Client-";
        clientId += String(random(0xffff), HEX);
        if (mqtt.connect(clientId.c_str())) {
            Serial.println("connected");
        } else {
            Serial.print("failed, rc="); Serial.print(mqtt.state());
            delay(3000);
        }
    }
}

bool getFastGPS() {
    // 1. Clear buffer & Send command
    while (SerialAT.available()) SerialAT.read();
    SerialAT.println("AT+CGNSSINFO");
    
    // 2. Read Response (Timeout 200ms)
    String response = "";
    unsigned long start = millis();
    while (millis() - start < 200) { 
        if (SerialAT.available()) response += (char)SerialAT.read();
    }
    
    response.trim(); 
    // Example: +CGNSSINFO: 3,14,,10,02,47.1543732,N,27.5978260,E,151225...

    // 3. Validation
    if (response.indexOf("+CGNSSINFO:") == -1) return false;
    if (response.indexOf(",,,,,,,,") != -1) return false; // No Fix

    // 4. Fast Parsing by counting commas
    // Based on your data, Lat is the 6th field (Index 5), Lon is 8th (Index 7), Speed is 13th (Index 12)
    
    int commaIndex[15]; // Store positions of first 15 commas
    int commaCount = 0;
    
    // Find where the data starts (after ": ")
    int dataStart = response.indexOf(": ") + 2;
    
    for (int i = dataStart; i < response.length(); i++) {
        if (response.charAt(i) == ',') {
            commaIndex[commaCount] = i;
            commaCount++;
            if (commaCount >= 13) break; // We only need up to speed
        }
    }

    // Safety check: Did we find enough commas?
    if (commaCount < 12) return false;

    // Extract Strings using the comma positions
    // Lat is between comma 4 and 5 (indices are 0-based, so 5th field)
    String latStr = response.substring(commaIndex[4] + 1, commaIndex[5]);
    
    // Lon is between comma 6 and 7
    String lonStr = response.substring(commaIndex[6] + 1, commaIndex[7]);
    
    // Speed is between comma 11 and 12
    String speedStr = response.substring(commaIndex[11] + 1, commaIndex[12]);

    // 5. Update Variables
    fast_lat = latStr.toFloat();
    fast_lon = lonStr.toFloat();
    fast_speed = speedStr.toFloat(); // This is in Knots usually
    
    // Direction Check (N/S, E/W) - Indices 6 and 8
    // If your modem outputs negative numbers for South/West automatically, you can skip this.
    // But usually, it provides "N" or "S".
    String ns = response.substring(commaIndex[5] + 1, commaIndex[6]);
    String ew = response.substring(commaIndex[7] + 1, commaIndex[8]);
    
    if (ns == "S") fast_lat *= -1;
    if (ew == "W") fast_lon *= -1;

    return true;
}

void setup() {
    Serial.begin(115200);
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
    
    Serial.println("\n\n--- BOOTING LILYGO A7670 ---");
    unsigned long bootStart = millis();
    unsigned long stepStart;

    // 1. HARDWARE POWER ON
    Serial.print("[1] Hardware Power On... ");
    stepStart = millis();
    
    pinMode(BOARD_POWERON_PIN, OUTPUT); digitalWrite(BOARD_POWERON_PIN, HIGH);
    pinMode(MODEM_PWRKEY, OUTPUT);
    digitalWrite(MODEM_PWRKEY, LOW); delay(100);
    digitalWrite(MODEM_PWRKEY, HIGH); delay(1000);
    digitalWrite(MODEM_PWRKEY, LOW);
    
    Serial.printf("Done (%lu ms)\n", millis() - stepStart);

    // 2. MODEM HANDSHAKE (Optimized)
    Serial.print("[2] Waiting for AT response... ");
    stepStart = millis();

    // Instead of restarting, we just hammer it with "AT" until it speaks
    // The modem takes ~3-4 seconds to boot after hardware power on.
    bool modemAlive = false;
    for (int i = 0; i < 20; i++) { // Try for 10 seconds max
        SerialAT.println("AT"); 
        delay(500); 
        if (SerialAT.available()) {
            String r = SerialAT.readString();
            if (r.indexOf("OK") >= 0) {
                modemAlive = true;
                break;
            }
        }
    }
    
    if (modemAlive) {
        Serial.printf("Done (%lu ms)\n", millis() - stepStart);
        // Only run init() to set internal library flags, NOT to reset power
        modem.init(); 
    } else {
        Serial.println("FAILED (Modem not responding)");
        // Optional: Only NOW try a hard reset if it failed
    }
    modem.simUnlock("1234");

    // 3. FORCE LTE MODE (Crucial for speed)
    Serial.print("[3] Forcing LTE Mode (No 2G scan)... ");
    stepStart = millis();
    modem.sendAT("+CNMP=38"); // 38 = LTE Only
    modem.waitResponse();
    Serial.printf("Done (%lu ms)\n", millis() - stepStart);

    // 4. NETWORK CONNECTION
    Serial.print("[4] Waiting for Network... ");
    stepStart = millis();
    
    // We use a custom loop to show progress dots
    if (!modem.waitForNetwork(10000L)) { // 10s Timeout
         Serial.printf(" FAIL or Timeout (%lu ms)\n", millis() - stepStart);
         Serial.println("    -> Proceeding anyway (GPS test mode)");
    } else {
         Serial.printf(" CONNECTED! (%lu ms)\n", millis() - stepStart);
         
         // 5. APN CONNECTION
         Serial.print("[5] Connecting to APN (");
         Serial.print(apn);
         Serial.print(")... ");
         stepStart = millis();
         if (modem.gprsConnect(apn, user, pass)) {
             Serial.printf("Success (%lu ms)\n", millis() - stepStart);
         } else {
             Serial.printf("Failed (%lu ms)\n", millis() - stepStart);
         }
    }

    // 6. ENABLE GPS
    Serial.print("[6] Enabling GPS... ");
    stepStart = millis();
    modem.sendAT("+CGNSSPWR=1"); // Direct AT is faster than library
    modem.waitResponse();
    Serial.printf("Done (%lu ms)\n", millis() - stepStart);

    // 7. MQTT SETUP
    mqtt.setServer(mqtt_server, 1883);
    mqtt.setCallback(mqttCallback);

    Serial.printf("--- SETUP COMPLETE in %lu ms ---\n", millis() - bootStart);
}

// Timer for MQTT sending
unsigned long lastSend = 0;

void loop() {
    // 1. Keep MQTT Alive
    if (!mqtt.connected()) reconnect();
    mqtt.loop();

    // 2. Read GPS (Fast!)
    bool hasFix = getFastGPS();

    // 3. Send MQTT (Every 1 second)
    // We send more often now because the read is so fast
    if (millis() - lastSend > 100) {
        lastSend = millis();

        if (hasFix) {
            // Build JSON: {"id":"lilygo","lat":47.15,"lon":27.59,"spd":0.0}
            String payload = "{\"id\":\"lilygo\",\"lat\":";
            payload += String(fast_lat, 6);
            payload += ",\"lon\":";
            payload += String(fast_lon, 6);
            payload += ",\"spd\":";
            payload += String(fast_speed, 2);
            payload += "}";

            Serial.print("TX: "); Serial.println(payload);
            mqtt.publish(mqtt_topic, payload.c_str());
        } else {
            Serial.print("."); // Heartbeat while waiting for fix
        }
    }
}