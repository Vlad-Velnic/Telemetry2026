/*

#define TINY_GSM_MODEM_A7670
#include <Arduino.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>

// --- CONFIGURATION ---
const char apn[] = "net";  // Replace with your APN (e.g., "live.vodafone.com")
const char user[] = "";
const char pass[] = "";

const char* mqtt_server = "broker.hivemq.com"; 
const char* mqtt_topic = "tuiracing";

// --- PINS (LILYGO T-CALL A7670) ---
#define MODEM_TX      26
#define MODEM_RX      25
#define MODEM_PWRKEY  4
#define BOARD_POWERON_PIN 12

#define SerialAT Serial1
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

// Variables
float fast_lat = 0.0;
float fast_lon = 0.0;
float fast_speed = 0.0;

// --- FAST GPS PARSER ---
// Returns true if valid fix found, updates global variables
bool getFastGPS() {
    // 1. Flush & Request
    while (SerialAT.available()) SerialAT.read();
    SerialAT.println("AT+CGNSSINFO");
    
    // 2. Read with tight timeout (200ms)
    String response = "";
    unsigned long start = millis();
    while (millis() - start < 200) { 
        if (SerialAT.available()) response += (char)SerialAT.read();
    }
    response.trim(); 

    // 3. Quick Validation
    // Format: +CGNSSINFO: <mode>,<sat_gps>,<sat_glonass>,<sat_beidou>,<lat>,<ns>,<lon>,<ew>,<date>,<time>,<alt>,<speed>,<course>,<pdop>,<hdop>,<vdop>
    if (response.indexOf("+CGNSSINFO:") == -1) return false;
    
    // Check for "empty" fix fields (Simcom returns many commas if no fix)
    // A valid lat looks like "47.123456", invalid is empty ",,"
    if (response.indexOf(",,,,,,,,") != -1) return false; 

    // 4. Parse by delimiters (Optimized)
    // We expect Lat at index 5, Lon at index 7, Speed at index 11 (varies by firmware, this is standard A7670)
    
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

    // Extract
    String latStr = response.substring(commaIndex[4] + 1, commaIndex[5]);
    String nsStr  = response.substring(commaIndex[5] + 1, commaIndex[6]);
    String lonStr = response.substring(commaIndex[6] + 1, commaIndex[7]);
    String ewStr  = response.substring(commaIndex[7] + 1, commaIndex[8]);
    String spdStr = response.substring(commaIndex[11] + 1, commaIndex[12]);

    // Convert
    if (latStr.length() < 2 || lonStr.length() < 2) return false; // Extra safety
    fast_lat = latStr.toFloat();
    fast_lon = lonStr.toFloat();
    fast_speed = spdStr.toFloat();

    // Adjust signs
    if (nsStr == "S") fast_lat *= -1;
    if (ewStr == "W") fast_lon *= -1;

    return true;
}

void reconnect() {
    if (!mqtt.connected()) {
        String clientId = "A7670-" + String(random(0xffff), HEX);
        if (mqtt.connect(clientId.c_str())) {
            Serial.println("MQTT Connected");
        }
    }
}

void setup() {
    Serial.begin(115200);
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
    
    Serial.println("\n\n--- SETUP START ---");

    // 1. Hardware Power On
    pinMode(BOARD_POWERON_PIN, OUTPUT); digitalWrite(BOARD_POWERON_PIN, HIGH);
    pinMode(MODEM_PWRKEY, OUTPUT);
    digitalWrite(MODEM_PWRKEY, LOW); delay(100);
    digitalWrite(MODEM_PWRKEY, HIGH); delay(1000);
    digitalWrite(MODEM_PWRKEY, LOW);
    
    // 2. Wait for AT
    bool modemReady = false;
    for(int i=0; i<30; i++) {
        SerialAT.println("AT");
        delay(200);
        if(SerialAT.available()) { 
             String r = SerialAT.readString();
             if(r.indexOf("OK") >= 0) { modemReady = true; break; }
        }
    }
    if(!modemReady) { Serial.println("Modem Failed!"); while(1); }
    
    modem.init();

    // 3. CONNECT TO NETWORK FIRST (CRITICAL FOR A-GPS)
    // The modem needs to talk to the tower to get time & coarse location
    Serial.print("Connecting to Network/APN... ");
    modem.sendAT("+CNMP=38"); // Force LTE
    modem.waitResponse();
    
    if (!modem.waitForNetwork(10000L) || !modem.gprsConnect(apn, user, pass)) {
        Serial.println("FAIL (GPS will be slow)");
    } else {
        Serial.println("OK (A-GPS Ready)");
    }

    // 4. CONFIGURE GPS (The Magic Part)
    Serial.print("Configuring GPS... ");
    
    // Turn OFF GPS first to configure
    modem.sendAT("+CGNSSPWR=0");
    modem.waitResponse();
    
    // Set Mode: 3 = GPS + GLONASS + BDS (BeiDou). 
    // Using multiple systems drastically improves reliability vs GPS-only.
    // (Note: Some firmware uses 7 for GPS+GLO+BDS, but 3 is standard 'GNSS+BD' on many A76 series. Try 3 or 7)
    modem.sendAT("+CGNSSMODE=3");
    modem.waitResponse();
    
    // Turn ON GPS
    modem.sendAT("+CGNSSPWR=1");
    modem.waitResponse();
    Serial.println("Done.");

    mqtt.setServer(mqtt_server, 1883);
}

unsigned long lastSend = 0;

void loop() {
    if (!mqtt.connected()) reconnect();
    mqtt.loop();

    bool fix = getFastGPS();

    if (millis() - lastSend > 2000) { // Send every 2s
        lastSend = millis();
        if (fix) {
            String p = "{\"lat\":" + String(fast_lat, 6) + 
                       ",\"lon\":" + String(fast_lon, 6) + 
                       ",\"spd\":" + String(fast_speed, 1) + "}";
            Serial.println(p);
            mqtt.publish(mqtt_topic, p.c_str());
        } else {
            Serial.print(".");
        }
    }
}

*/