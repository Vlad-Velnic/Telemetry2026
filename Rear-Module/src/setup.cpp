#include "includes.h"

// --- GLOBAL INSTANCES ---
#define SerialAT Serial1
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);
#if ENABLE_OTA
volatile bool otaReady = false;
#endif

// --- SETTINGS ---
const char apn[] = "live.vodafone.com";
const char user[] = "";
const char pass[] = "";
const char* mqtt_server = "broker.hivemq.com"; 
const char* mqtt_topic = "tuiracing";

static bool responseListsRate(const String &response, uint8_t rate) {
    int value = -1;
    int rangeStart = -1;
    bool readingRangeEnd = false;
    for (size_t i = 0; i < response.length(); ++i) {
        const char ch = response.charAt(i);
        if (ch >= '0' && ch <= '9') {
            if (value < 0) value = 0;
            value = value * 10 + (ch - '0');
            continue;
        }
        if (ch == '-' && value >= 0) {
            rangeStart = value;
            value = -1;
            readingRangeEnd = true;
            continue;
        }
        if (readingRangeEnd && value >= 0 && rangeStart <= rate && rate <= value) {
            return true;
        }
        if (!readingRangeEnd && value == rate) return true;
        value = -1;
        rangeStart = -1;
        readingRangeEnd = false;
    }
    return (readingRangeEnd && value >= 0 && rangeStart <= rate && rate <= value) ||
           (!readingRangeEnd && value == rate);
}

static uint8_t configureGpsRate() {
    String supportedRates;
    modem.sendAT("+CGPSNMEARATE=?");
    const bool queryOk = modem.waitResponse(2000L, supportedRates) == 1;

    const uint8_t candidates[] = {5, 2, 1};
    for (uint8_t candidate : candidates) {
        if (queryOk && !responseListsRate(supportedRates, candidate)) continue;
        if (modem.setGPSOutputRate(candidate)) return candidate;
    }

    // Some firmware omits the test-command response but still accepts 1 Hz.
    modem.setGPSOutputRate(1);
    return 1;
}

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
    rearModemReady = false;
    DEBUG_PRINTLN("Initializing Modem...");
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);

    // Power Cycle Sequence
    digitalWrite(MODEM_PWRKEY, LOW); delay(100);
    digitalWrite(MODEM_PWRKEY, HIGH); delay(1000);
    digitalWrite(MODEM_PWRKEY, LOW); delay(2000);

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
        if (!modem.init("0000")) {
            DEBUG_PRINTLN("Modem initialization failed");
            return;
        }
        DEBUG_PRINTLN("Modem Init OK");

        // Start GNSS before network registration so satellite acquisition can
        // proceed while the lower-priority MQTT task brings LTE online.
        DEBUG_PRINTLN("Configuring GNSS...");
        modem.sendAT("+CGNSSPWR=0"); modem.waitResponse();
        modem.sendAT("+CGNSSPWR=1");
        if (modem.waitResponse(30000UL, "+CGNSSPWR: READY!", "ERROR") != 1) {
            DEBUG_PRINTLN("GNSS failed to become ready");
            return;
        }

        // Foreign A7670E/SA UNICORE variants use mode 3 for
        // GPS + GLONASS + GALILEO + SBAS + QZSS.
        if (!modem.setGPSMode(3)) {
            DEBUG_PRINTLN("GNSS mode configuration failed");
        }
        gpsRateHz = configureGpsRate();
        DEBUG_PRINTF("GNSS output rate: %u Hz\n", gpsRateHz);
        rearModemReady = true;
    }
}

void setupMQTT() {
    mqtt.setServer(mqtt_server, 1883);
    mqtt.setSocketTimeout(1);
    if (!mqtt.setBufferSize(MQTT_PACKET_BUFFER_SIZE)) {
        DEBUG_PRINTLN("MQTT packet buffer allocation failed");
    }
    // Callback can be added here if we need to receive commands
}

#if ENABLE_OTA
void setupOTA() {
    DEBUG_PRINTLN("Initializing OTA Wi-Fi...");

    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.setHostname(OTA_HOSTNAME);
    WiFi.begin(OTA_WIFI_SSID, OTA_WIFI_PASSWORD);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < OTA_CONNECT_TIMEOUT_MS) {
        delay(100);
    }

    if (WiFi.status() != WL_CONNECTED) {
        otaReady = false;
        DEBUG_PRINTLN("OTA Wi-Fi not available; continuing without OTA for now");
        return;
    }

    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_UPDATE_PASSWORD);

    ArduinoOTA.onStart([]() {
        DEBUG_PRINTLN("OTA update started");
    });

    ArduinoOTA.onEnd([]() {
        DEBUG_PRINTLN("OTA update finished");
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        DEBUG_PRINTF("OTA progress: %u%%\n", (progress * 100) / total);
    });

    ArduinoOTA.onError([](ota_error_t error) {
        DEBUG_PRINTF("OTA error[%u]\n", error);
    });

    ArduinoOTA.begin();
    otaReady = true;

    DEBUG_PRINT("OTA ready. IP: ");
    DEBUG_PRINTLN(WiFi.localIP());
}
#endif

void setupCAN() {
    rearCanReady = false;
    DEBUG_PRINTLN("Initializing CAN...");
    
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)PIN_CAN_TX, (gpio_num_t)PIN_CAN_RX, TWAI_MODE_NORMAL);
    g_config.tx_queue_len = 16;
    g_config.rx_queue_len = 32;
    g_config.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED |
                              TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_TX_FAILED;
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        DEBUG_PRINTLN("CAN Driver Installed");
    } else {
        DEBUG_PRINTLN("CAN Install FAILED");
        return;
    }

    if (twai_start() == ESP_OK) {
        rearCanReady = true;
        DEBUG_PRINTLN("CAN Started");
    } else {
        DEBUG_PRINTLN("CAN Start FAILED");
    }
}
