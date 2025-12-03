#include "includes.h"

void sendCanMessage(int id, uint8_t* data, size_t length) {
  if (length > 8) return; // CAN limit is 8 bytes per frame

  CAN.beginPacket(id);
  CAN.write(data, length);
  uint8_t status = CAN.endPacket();

  if (status) {
    DEBUG_PRINT(F("CAN TX Success: ID=0x"));
    DEBUG_PRINTF("%X\n", id);
  } else {
    DEBUG_PRINTLN(F("CAN TX FAILED"));
  }
}

void onCanReceive(int packetSize) {
  long id = CAN.packetId();
  bool isRemote = CAN.packetRtr();
  
  DEBUG_PRINT(F("CAN RX: ID=0x"));
  DEBUG_PRINTF("%X", id);
  DEBUG_PRINT(F(" DLC="));
  DEBUG_PRINT(packetSize);
  
  if (isRemote) {
    DEBUG_PRINTLN(F(" (RTR)"));
  } else {
    DEBUG_PRINT(F(" Data=["));
    while (CAN.available()) {
      DEBUG_PRINTF("%02X ", CAN.read());
    }
    DEBUG_PRINTLN(F("]"));
  }
}

void updateDisplay(u_int8_t currentGear, unsigned long lastLapTime, float currentTemp, float currentBatteryVoltage, int currentRPM)
{
    display.clearDisplay();

    // Large gear number on left
    display.setTextSize(8);
    display.setCursor(0, 5);
    if (currentGear == 0 && !NO_REAR)
        display.print("N");
    else
        display.print(currentGear);

    // Right side time
    display.setTextSize(2);

    // Time at top right
    unsigned long totalMs = lastLapTime;
    unsigned int mins = (totalMs / 60000) % 60;
    unsigned int secs = (totalMs / 1000) % 60;
    unsigned int tenths = (totalMs / 100) % 10;

    display.setCursor(50, 8);
    display.printf("%01d:%02d:%d", mins, secs, tenths);

    // Right side temp and voltage
    display.setTextSize(1);

    if (NO_WIFI)
    {
        display.setCursor(55, 30);
        display.printf("W");
    }

    if (NO_REAR)
    {
        display.setCursor(65, 30);
        display.printf("R");
    }

    if (NO_ECU)
    {
        display.setCursor(75, 30);
        display.printf("M");
    }

    // Temperature
    display.setCursor(100, 42);
    display.printf("%.0f%cC", currentTemp, 247);

    // RPM
    display.setCursor(51, 42);
    display.printf("%d", currentRPM);

    // Battery voltage
    display.setCursor(95, 57);
    display.printf("%.1fV", currentBatteryVoltage);

    display.display();
    return;
}