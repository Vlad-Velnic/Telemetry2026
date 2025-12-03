#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>

// ================================================================
// DEBUG CONFIGURATION
// ================================================================
// Set this to 0 to disable all Serial prints
#define ENABLE_DEBUG 1 

// Macro definitions
#if ENABLE_DEBUG
  #define DEBUG_BEGIN(speed)    Serial.begin(speed)
  #define DEBUG_PRINT(...)      Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...)    Serial.println(__VA_ARGS__)
  #define DEBUG_PRINTF(...)     Serial.printf(__VA_ARGS__)
#else
  #define DEBUG_BEGIN(speed)
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
  #define DEBUG_PRINTF(...)
#endif

#endif