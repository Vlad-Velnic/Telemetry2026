#include "includes.h"

QueueHandle_t canQueue;

volatile int currentRPM = 0;
volatile float currentTemp = 0.0;
volatile float currentBat = 0.0;
volatile int currentGear = 0;
volatile unsigned long lastLapTime = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Booting Front Module...");

  setupPins();  // GPIO
  setupSD();    // SD pe HSPI
  setupOLED();  // OLED pe VSPI/Software
  setupMPU();   // I2C
  setupCAN();   // TWAI

  canQueue = xQueueCreate(100, sizeof(LogMessage));

  // Pornire Task-uri
  xTaskCreatePinnedToCore(SD_Task,  "SD_Log", 4096, NULL, 1, NULL, 0); // Core 0
  xTaskCreatePinnedToCore(CAN_Task, "CAN_RX", 4096, NULL, 5, NULL, 1); // Core 1
}

void loop() {
  unsigned long currentMillis = millis();

  // --- 1. CITIRE SENZORI ANALOGICI ---
  int damper1 = analogRead(PIN_DAMPER_1);
  int damper2 = analogRead(PIN_DAMPER_2);
  int steering = analogRead(PIN_STEERING);

  uint8_t analogMsg[6];
  analogMsg[0] = (damper1 >> 8) & 0xFF; analogMsg[1] = damper1 & 0xFF;
  analogMsg[2] = (damper2 >> 8) & 0xFF; analogMsg[3] = damper2 & 0xFF;
  analogMsg[4] = (steering >> 8) & 0xFF; analogMsg[5] = steering & 0xFF;

  sendCanMessage(CAN_ID_ANALOG_SENSORS, analogMsg, 6);


  // --- 2. CITIRE MPU6050 ---
  sensors_event_t a, g, temp;
  if (mpu.getEvent(&a, &g, &temp)) {
      int16_t ax = (int16_t)(a.acceleration.x * 100);
      int16_t ay = (int16_t)(a.acceleration.y * 100);
      int16_t az = (int16_t)(a.acceleration.z * 100);

      uint8_t accelMsg[6];
      accelMsg[0] = (ax >> 8) & 0xFF; accelMsg[1] = ax & 0xFF;
      accelMsg[2] = (ay >> 8) & 0xFF; accelMsg[3] = ay & 0xFF;
      accelMsg[4] = (az >> 8) & 0xFF; accelMsg[5] = az & 0xFF;
      
      sendCanMessage(CAN_ID_ACCEL_DATA, accelMsg, 6);
  }

  updateDisplay(currentGear, lastLapTime, currentTemp, currentBat, currentRPM);

  delay(50);
}