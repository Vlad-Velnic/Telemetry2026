#include "includes.h"


SPIClass sdSPI(HSPI);
SPIClass oledSPI(VSPI);
Adafruit_SSD1306 display(128, 64, &oledSPI, PIN_OLED_DC, PIN_OLED_RESET, PIN_OLED_CS);
Adafruit_MPU6050 mpu;
bool NO_REAR = true, NO_ECU = true, NO_GPS = true;
bool mpuReady = false;
bool displayReady = false;

void setupPins() {
  pinMode(PIN_DAMPER_1, INPUT);
  pinMode(PIN_DAMPER_2, INPUT);
  pinMode(PIN_STEERING, INPUT);
}

void setupOLED() {
  oledSPI.begin(PIN_OLED_CLK, -1, PIN_OLED_MOSI, PIN_OLED_CS);
  displayReady = display.begin(SSD1306_SWITCHCAPVCC);
  if(!displayReady) {
    DEBUG_PRINTLN(F("SSD1306 allocation failed"));
  } else {
    display.clearDisplay();
    display.setTextSize(4); // Adjusted for boot screen
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10,20);
    display.println(F("BOOT"));
    display.display();
  }
}

void setupSD() {
  DEBUG_PRINTLN(F("Initializing SD Card..."));
  
  sdSPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  DEBUG_PRINTLN("SPI Started");
  sdSPI.setDataMode(SPI_MODE0);
  DEBUG_PRINTLN("SPI mode 0 Started");

  delay(100);

  mountSDStorage();
}

bool mountSDStorage() {
  static bool sessionNameAssigned = false;

  frontSdReady = false;
  if (!SD.begin(PIN_SD_CS, sdSPI)) {
    DEBUG_PRINTLN(F("SD Card Init Failed!"));
    return false;
  } else {
    DEBUG_PRINTLN(F("SD Card Ready."));

    if (!sessionNameAssigned) {
      // Retrieve session index from NVS (Flash)
      Preferences prefs;
      if (!prefs.begin("telemetry", false)) {
        DEBUG_PRINTLN(F("Failed to open telemetry preferences"));
        SD.end();
        return false;
      }
      unsigned int sessionIndex = prefs.getUInt("index", 0);

      // Never truncate an earlier session if NVS was erased or an SD card was
      // moved from another module.
      do {
        sessionIndex++;
        snprintf(logFileName, sizeof(logFileName), "/log_%04u.csv", sessionIndex);
      } while (SD.exists(logFileName) && sessionIndex < UINT_MAX);

      if (prefs.putUInt("index", sessionIndex) != sizeof(sessionIndex)) {
        DEBUG_PRINTLN(F("Failed to persist telemetry session index"));
      }
      prefs.end();

      DEBUG_PRINT(F("Starting session index: "));
      DEBUG_PRINTLN(sessionIndex);
      DEBUG_PRINT(F("Log file: "));
      DEBUG_PRINTLN(logFileName);

      File f = SD.open(logFileName, FILE_WRITE);
      if(f) {
        f.println(F("---- BOOT SESSION START ----"));
        f.close();
        sessionNameAssigned = true;
      } else {
        DEBUG_PRINTLN(F("Failed to open log file"));
        SD.end();
        return false;
      }
    }

    frontSdReady = true;
    return true;
  }
}

void setupMPU() {
  Wire.begin(PIN_MPU_SDA, PIN_MPU_SCL);
  Wire.setClock(100000);
  mpuReady = mpu.begin(MPU6050_I2CADDR_DEFAULT,&Wire,0);
  if (!mpuReady) {
    DEBUG_PRINTLN(F("MPU6050 Not Found"));
  } else {
    DEBUG_PRINTLN(F("MPU6050 OK"));
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
}

void setupCAN() {
  frontCanReady = false;
  DEBUG_PRINTLN(F("Initializing Native CAN (TWAI)..."));

  // 1. General Config: TX, RX, Normal Mode
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)PIN_CAN_TX, (gpio_num_t)PIN_CAN_RX, TWAI_MODE_NORMAL);
  g_config.tx_queue_len = 16;
  g_config.rx_queue_len = 32;
  g_config.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED |
                            TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_TX_FAILED;
  
  // 2. Timing Config: 500kbps (Matches MegaSquirt)
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  
  // 3. Filter Config: Accept All
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install Driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    DEBUG_PRINTLN(F("CAN Driver Installed"));
  } else {
    DEBUG_PRINTLN(F("CAN Driver Install FAILED"));
    return;
  }

  // Start Driver
  if (twai_start() == ESP_OK) {
    frontCanReady = true;
    DEBUG_PRINTLN(F("CAN Bus Started"));
  } else {
    DEBUG_PRINTLN(F("CAN Start FAILED"));
  }
}
