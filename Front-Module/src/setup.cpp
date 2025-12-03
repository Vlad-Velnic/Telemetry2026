#include "includes.h"

void setupPins() {
  pinMode(PIN_DAMPER_1, INPUT);
  pinMode(PIN_DAMPER_2, INPUT);
  pinMode(PIN_STEERING, INPUT);
  pinMode(PIN_THROTTLE, INPUT);
}

void setupOLED() {
  if(!display.begin(SSD1306_SWITCHCAPVCC)) { 
    DEBUG_PRINTLN(F("SSD1306 allocation failed"));
  } else {
    display.clearDisplay();
    display.setTextSize(8);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(5,5);
    display.println(F("262"));
    display.display();
  }
}

void setupSD() {
  if (!SD.begin(PIN_SD_CS)) {
    DEBUG_PRINTLN(F("SD Card initialization failed!"));
  } else {
    DEBUG_PRINTLN(F("SD Card initialized."));
  }
}

void setupMPU() {
  Wire.begin(PIN_MPU_SDA, PIN_MPU_SCL);

  if (!mpu.begin(MPU6050_I2CADDR_DEFAULT,&Wire,0)) {
    DEBUG_PRINTLN(F("Failed to find MPU6050 chip"));
  } else {
    DEBUG_PRINTLN(F("MPU6050 Found!"));
    // Optional: Configure ranges
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
}

void setupCAN() {
  DEBUG_PRINTLN(F("Initializing CAN Bus..."));

  CAN.setPins(PIN_CAN_RX, PIN_CAN_TX);

  if (!CAN.begin(500E3)) {
    DEBUG_PRINTLN(F("Starting CAN failed!"));
    for(int i=0;i<10;i++) {if(CAN.begin(500E3)) break;};
  } else {
    DEBUG_PRINTLN(F("CAN Bus Started."));

    CAN.onReceive(onCanReceive);
  }
}