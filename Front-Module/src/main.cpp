#include "includes.h"

QueueHandle_t canQueue;
char logFileName[32] = "/datalog.csv";

volatile int currentGear = 0;
volatile float currentGpsSpeed = 0.0;
volatile uint16_t currentBrakePressure = 0;
volatile unsigned long lastLapTime = 0;
volatile unsigned long lastGearCanRxMs = 0;
volatile unsigned long lastRearAnalogCanRxMs = 0;
volatile unsigned long lastGpsPositionCanRxMs = 0;
volatile uint32_t frontCanQueueDrops = 0;
volatile uint32_t frontCanTxFailures = 0;
volatile bool frontCanReady = false;
volatile bool frontSdReady = false;
volatile uint32_t frontSdWriteFailures = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Booting Front Module...");

  setupPins();  // GPIO
  setupSD();    // SD on HSPI
  setupOLED();  // OLED on VSPI/Software
  setupMPU();   // I2C
  setupCAN();   // TWAI

  canQueue = xQueueCreate(200, sizeof(LogMessage));
  if (!canQueue) {
    DEBUG_PRINTLN(F("CAN log queue allocation failed"));
  }

  // Task Start
  if (canQueue) {
    xTaskCreatePinnedToCore(SD_Task,  "SD_Log", 4096, NULL, 1, NULL, 0); // Core 0
  }
  if (frontCanReady && canQueue) {
    xTaskCreatePinnedToCore(CAN_Task, "CAN_RX", 4096, NULL, 4, NULL, 1); // Core 1
  }
}

void loop() {
  static TickType_t lastWakeTime = xTaskGetTickCount();
  static unsigned long lastDisplayUpdate = 0;
  static unsigned long lastHealthUpdate = 0;
  static unsigned long lastMpuRetry = 0;
  unsigned long currentMillis = millis();

  // Mark each remote data source missing until a recent valid CAN frame arrives.
  NO_REAR = lastGearCanRxMs == 0 || lastRearAnalogCanRxMs == 0 ||
            currentMillis - lastGearCanRxMs > REAR_CAN_TIMEOUT_MS ||
            currentMillis - lastRearAnalogCanRxMs > REAR_CAN_TIMEOUT_MS;
  NO_GPS = lastGpsPositionCanRxMs == 0 ||
           currentMillis - lastGpsPositionCanRxMs > GPS_CAN_TIMEOUT_MS;

  // 1. Read Analog Sensors (Dampers, Steering)
  int d1 = analogRead(PIN_DAMPER_1);
  int d2 = analogRead(PIN_DAMPER_2);
  int str = analogRead(PIN_STEERING);

  uint8_t analogMsg[6];
  analogMsg[0] = (d1 >> 8) & 0xFF;  analogMsg[1] = d1 & 0xFF;
  analogMsg[2] = (d2 >> 8) & 0xFF;  analogMsg[3] = d2 & 0xFF;
  analogMsg[4] = (str >> 8) & 0xFF; analogMsg[5] = str & 0xFF;

  broadcastData(CAN_ID_FRONT_ANALOG, analogMsg, 6);

  // 2. Read MPU (Accel & Gyro). Probe first because the Adafruit read API
  // does not reliably surface an I2C disconnect as a failed getEvent().
  if (mpuReady) {
    Wire.beginTransmission(MPU6050_I2CADDR_DEFAULT);
    if (Wire.endTransmission() != 0) mpuReady = false;
  } else if (currentMillis - lastMpuRetry >= 5000) {
    lastMpuRetry = currentMillis;
    setupMPU();
  }

  sensors_event_t a, g, temp;
  if (mpuReady && mpu.getEvent(&a, &g, &temp)) {
    int16_t ax = (int16_t)(a.acceleration.x * 100);
    int16_t ay = (int16_t)(a.acceleration.y * 100);
    int16_t az = (int16_t)(a.acceleration.z * 100);

    int16_t gx = (int16_t)(g.gyro.x * 100);
    int16_t gy = (int16_t)(g.gyro.y * 100);
    int16_t gz = (int16_t)(g.gyro.z * 100);

    uint8_t accelMsg[6];
    accelMsg[0] = (ax >> 8) & 0xFF;  accelMsg[1] = ax & 0xFF;
    accelMsg[2] = (ay >> 8) & 0xFF;  accelMsg[3] = ay & 0xFF;
    accelMsg[4] = (az >> 8) & 0xFF;  accelMsg[5] = az & 0xFF;
    broadcastData(CAN_ID_ACCEL, accelMsg, 6);

    uint8_t gyroMsg[6];
    gyroMsg[0] = (gx >> 8) & 0xFF;  gyroMsg[1] = gx & 0xFF;
    gyroMsg[2] = (gy >> 8) & 0xFF;  gyroMsg[3] = gy & 0xFF;
    gyroMsg[4] = (gz >> 8) & 0xFF;  gyroMsg[5] = gz & 0xFF;
    broadcastData(CAN_ID_GYRO, gyroMsg, 6);
  }

  // 3. Update Display at a lower frequency (e.g., 10Hz)
  if (currentMillis - lastDisplayUpdate >= DISPLAY_PERIOD_MS) {
    if (displayReady) {
      updateDisplay(currentGear, lastLapTime, currentGpsSpeed,
                    currentBrakePressure);
    }
    lastDisplayUpdate = currentMillis;
  }

  if (currentMillis - lastHealthUpdate >= HEALTH_PERIOD_MS) {
    sendHealthFrame();
    lastHealthUpdate = currentMillis;
  }

  // Precise LOGGING_FREQ_HZ loop timing
  vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(LOGGING_PERIOD_MS));
}
