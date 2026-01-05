#include "includes.h"

// --- GLOBAL INSTANCES ---
#define SerialAT Serial1
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

// --- SETTINGS ---
const char apn[] = "net";
const char user[] = "";
const char pass[] = "";
const char* mqtt_server = "broker.hivemq.com"; 
const char* mqtt_topic = "tuiracing";

void setupPins() {
    // Modem Power
    pinMode(MODEM_POWER_ON, OUTPUT);
    digitalWrite(MODEM_POWER_ON, HIGH);
    pinMode(MODEM_PWRKEY, OUTPUT);

    // Sensors
    pinMode(PIN_DAMPER_RL, INPUT);
    pinMode(PIN_DAMPER_RR, INPUT);
    pinMode(PIN_BRAKE_PRESS, INPUT);

    // Gear Sensor
    pinMode(PIN_GEAR_1, INPUT);
    pinMode(PIN_GEAR_2, INPUT);
    pinMode(PIN_GEAR_3, INPUT);
    pinMode(PIN_GEAR_4, INPUT);
    pinMode(PIN_GEAR_5, INPUT);
    pinMode(PIN_GEAR_0, INPUT);
}

void setupModem() {
    DEBUG_PRINTLN("Initializing Modem...");
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);

    // Power Cycle Sequence
    digitalWrite(MODEM_PWRKEY, LOW); delay(100);
    digitalWrite(MODEM_PWRKEY, HIGH); delay(1000);
    digitalWrite(MODEM_PWRKEY, LOW);

    // Wait for AT Ready
    bool modemReady = false;
    for(int i=0; i<30; i++) {
        SerialAT.println("AT");
        delay(200);
        if(SerialAT.available()) { 
             String r = SerialAT.readString();
             if(r.indexOf("OK") >= 0) { modemReady = true; break; }
        }
    }
    
    if(!modemReady) { 
        DEBUG_PRINTLN("Modem Hardware Fail!"); 
        // We continue anyway to not block other features
    } else {
        modem.init("0000");
        DEBUG_PRINTLN("Modem Init OK. Connecting to Network...");
        
        // Network Connect
        modem.sendAT("+CNMP=38"); // Force LTE
        modem.waitResponse();
        
        if (!modem.waitForNetwork(10000L) || !modem.gprsConnect(apn, user, pass)) {
            DEBUG_PRINTLN("Network Fail (GPS/MQTT may not work)");
        } else {
            DEBUG_PRINTLN("Network OK");
        }
        
        // GPS Setup
        DEBUG_PRINTLN("Configuring GNSS...");
        modem.sendAT("+CGNSSPWR=0"); modem.waitResponse();
        modem.sendAT("+CGNSSMODE=3"); modem.waitResponse(); // GPS+GLONASS+BDS
        modem.sendAT("+CGNSSPWR=1"); modem.waitResponse();
    }
}

void setupMQTT() {
    mqtt.setServer(mqtt_server, 1883);
    // Callback can be added here if we need to receive commands
}

void setupCAN() {
    DEBUG_PRINTLN("Initializing CAN...");
    
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)PIN_CAN_TX, (gpio_num_t)PIN_CAN_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        DEBUG_PRINTLN("CAN Driver Installed");
    } else {
        DEBUG_PRINTLN("CAN Install FAILED");
        return;
    }

    if (twai_start() == ESP_OK) {
        DEBUG_PRINTLN("CAN Started");
    } else {
        DEBUG_PRINTLN("CAN Start FAILED");
    }
}