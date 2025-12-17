#include "includes.h"

QueueHandle_t canQueue;

volatile int currentRPM = 0;
volatile float currentTemp = 0.0;
volatile float currentBat = 0.0;

void setup() {
  DEBUG_BEGIN(115200);
  DEBUG_PRINTLN("Booting Front Module...");

  setupPins();
  setupMPU();
  setupSD();
  setupOLED();
  setupCAN();

  canQueue = xQueueCreate(300, sizeof(LogMessage));

  xTaskCreatePinnedToCore(
      SD_Task,    "SD_Log",   4096,  NULL,  1,  NULL,  0 
  );

  xTaskCreatePinnedToCore(
      CAN_Task,   "CAN_RX",   4096,  NULL,  10, NULL,  1 
  );
}

void loop() {
  // Main Loop runs on Core 1 (Low Priority)
  // Use this for updating Display and Reading Local Sensors at 10Hz
  
  // 1. Read Local Sensors (Example)
  // int damp1 = analogRead(PIN_DAMPER_1);
  // sendCanMessage(0x200, (uint8_t*)&damp1, 2); // Send to backend
  //readMPUData();
  //delay(1000);
  
  // 2. Update Display with latest data (Thread Safe-ish reading of volatiles)
  // We pass 0 as lap time for now
  updateDisplay(0, millis(), currentTemp, currentBat, currentRPM);
}