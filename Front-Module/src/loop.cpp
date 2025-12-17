#include "includes.h"

// Helper to send message AND log it
void sendCanMessage(uint32_t id, uint8_t* data, size_t length) {
    if (length > 8) length = 8;
    
    // 1. Send to Bus
    twai_message_t txMsg;
    txMsg.identifier = id;
    txMsg.extd = 0; // Standard ID
    txMsg.data_length_code = length;
    memcpy(txMsg.data, data, length);
    
    if (twai_transmit(&txMsg, pdMS_TO_TICKS(10)) == ESP_OK) {
        // DEBUG_PRINTLN("TX OK");
    }

    // 2. Push to Logger Queue (So we know what WE said)
    LogMessage log;
    log.id = id;
    log.len = length;
    log.timestamp = millis();
    log.isRx = false; // Outgoing
    memcpy(log.data, data, length);
    
    xQueueSend(canQueue, &log, 0);
}

// --- TASK 1: HIGH PRIORITY CAN RECEIVER ---
void CAN_Task(void *pvParameters) {
    twai_message_t rxMsg;
    LogMessage log;

    while (1) {
        // Block indefinitely until a message arrives
        if (twai_receive(&rxMsg, portMAX_DELAY) == ESP_OK) {
            
            // 1. Prepare Log Object
            log.id = rxMsg.identifier;
            log.len = rxMsg.data_length_code;
            log.timestamp = millis();
            log.isRx = true; // Incoming
            memcpy(log.data, rxMsg.data, rxMsg.data_length_code);

            // 2. Parse Critical Data for Display (MegaSquirt Example)
            // Adjust IDs based on your MegaSquirt config!
            if (rxMsg.identifier == 1512) { // Example ID for RPM
                // MegaSquirt usually sends RPM in bytes 6 and 7 (Big Endian?)
                // This is just an example parser:
                 currentRPM = (rxMsg.data[6] << 8) | rxMsg.data[7];
            }
            if (rxMsg.identifier == 1513) { // Example ID for Temp
                 currentTemp = (float)rxMsg.data[0]; 
            }

            // 3. Push to SD Queue (Don't wait if full)
            xQueueSend(canQueue, &log, 0);
        }
    }
}

// --- TASK 2: LOW PRIORITY SD WRITER ---
void SD_Task(void *pvParameters) {
    LogMessage msg;
    char buffer[128]; // Buffer for one line
    
    // Pre-allocate buffer for SD (speedup)
    const int BATCH_SIZE = 20; 
    int batchCount = 0;

    while (1) {
        // Wait for data (Block up to 100ms)
        if (xQueueReceive(canQueue, &msg, pdMS_TO_TICKS(100))) {
            
            File logFile = SD.open("/datalog.csv", FILE_APPEND);
            if (logFile) {
                // Format: Time, RX/TX, ID, Len, Data...
                int n = sprintf(buffer, "%lu,%s,%X,%d", 
                    msg.timestamp, 
                    msg.isRx ? "RX" : "TX", 
                    msg.id, 
                    msg.len
                );

                for (int i = 0; i < msg.len; i++) {
                    n += sprintf(buffer + n, ",%02X", msg.data[i]);
                }
                sprintf(buffer + n, "\n");

                logFile.print(buffer);

                // Flush occasionally to save SD wear and speed up
                batchCount++;
                if (batchCount >= BATCH_SIZE) {
                    logFile.flush();
                    batchCount = 0;
                }
                logFile.close();
            } else {
                // SD Error - maybe blink an LED?
                DEBUG_PRINTLN("SD Write Fail");
            }
        }
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

void readMPUData() {
    sensors_event_t a, g, temp;
    
    // getEvent returns true on success, false on failure
    if (mpu.getEvent(&a, &g, &temp)) {
        Serial.printf("MPU6050 >> Accel: [%.2f, %.2f, %.2f] m/s^2 | Gyro: [%.2f, %.2f, %.2f] rad/s | Temp: %.2f C\n",
            a.acceleration.x,
            a.acceleration.y,
            a.acceleration.z,
            g.gyro.x,
            g.gyro.y,
            g.gyro.z,
            temp.temperature
        );
    } else {
        Serial.println("MPU6050 >> Read Error (I2C Failed)");
    }
}

void readMPUData2() {
    const int MPU_ADDR = 0x68;
    int16_t acX, acY, acZ, tmp, gyX, gyY, gyZ;

    // 1. Begin Transmission
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // Start at Accel Register
    int error = Wire.endTransmission(false);

    if (error != 0) {
        // If connection fails, try to reset the bus
        Serial.printf("MPU >> I2C Error [%d]. Resetting Bus...\n", error);
        //resetI2CBus();
        return; 
    }

    // 2. Request 14 Bytes
    int count = Wire.requestFrom(MPU_ADDR, 14, 1);
    if (count == 14) {
        acX = Wire.read() << 8 | Wire.read();
        acY = Wire.read() << 8 | Wire.read();
        acZ = Wire.read() << 8 | Wire.read();
        tmp = Wire.read() << 8 | Wire.read();
        gyX = Wire.read() << 8 | Wire.read();
        gyY = Wire.read() << 8 | Wire.read();
        gyZ = Wire.read() << 8 | Wire.read();

        // Convert to physical values
        // Accel range default +/- 2g (16384 LSB/g)
        float ax = acX / 16384.0;
        float ay = acY / 16384.0;
        float az = acZ / 16384.0;
        
        // Temp formula: (Raw / 340.0) + 36.53
        float temperature = (tmp / 340.00) + 36.53;

        // Filter out garbage data (e.g., > 80C is impossible)
        if (temperature > 80.0 || temperature < -20.0) {
            Serial.println("MPU >> Garbage Data Ignored");
            return;
        }

        Serial.printf("MPU >> Acc: [%.2f, %.2f, %.2f] | Temp: %.2f C\n", ax, ay, az, temperature);
        
        // Update global variables for Display
        currentTemp = temperature;
        
    } else {
        Serial.println("MPU >> Read Timeout");
    }
}