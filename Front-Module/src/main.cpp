#include "includes.h"

void setup() {
  DEBUG_BEGIN(115200);
  DEBUG_PRINTLN("Booting Front Module...");

  setupPins();
  setupOLED();
  setupMPU();
  setupSD();
  setupCAN();
}

void loop() {
  
  
}