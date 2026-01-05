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

    // 1. Read Analog Sensors
    int d1 = analogRead(PIN_DAMPER_1);
    int d2 = analogRead(PIN_DAMPER_2);
    int str = analogRead(PIN_STEERING);

    uint8_t analogMsg[6];
    analogMsg[0] = (d1 >> 8) & 0xFF; analogMsg[1] = d1 & 0xFF;
    analogMsg[2] = (d2 >> 8) & 0xFF; analogMsg[3] = d2 & 0xFF;
    analogMsg[4] = (str >> 8) & 0xFF; analogMsg[5] = str & 0xFF;

    broadcastData(CAN_ID_FRONT_ANALOG, analogMsg, 6);

    // 2. Read MPU (Accel)
    sensors_event_t a, g, temp;
    if (mpu.getEvent(&a, &g, &temp)) {
        int16_t ax = (int16_t)(a.acceleration.x * 100);
        int16_t ay = (int16_t)(a.acceleration.y * 100);
        int16_t az = (int16_t)(a.acceleration.z * 100);

        uint8_t accelMsg[6];
        accelMsg[0] = (ax >> 8) & 0xFF; accelMsg[1] = ax & 0xFF;
        accelMsg[2] = (ay >> 8) & 0xFF; accelMsg[3] = ay & 0xFF;
        accelMsg[4] = (az >> 8) & 0xFF; accelMsg[5] = az & 0xFF;
        
        broadcastData(CAN_ID_ACCEL, accelMsg, 6);
    }

    // 3. Update Display
    updateDisplay(currentGear, lastLapTime, currentTemp, currentBat, currentRPM);

    delay(20); // 50Hz Loop
}