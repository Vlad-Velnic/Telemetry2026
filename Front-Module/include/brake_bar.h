#ifndef FRONT_MODULE_BRAKE_BAR_H
#define FRONT_MODULE_BRAKE_BAR_H

#include <stddef.h>
#include <stdint.h>

// Calibration is intentionally independent from the OLED geometry.
constexpr uint16_t BRAKE_PRESSURE_MIN_RAW = 0;
constexpr uint16_t BRAKE_PRESSURE_MAX_RAW = 4095;

constexpr int BRAKE_BAR_X = 60;
constexpr int BRAKE_BAR_Y = 49;
constexpr int BRAKE_BAR_WIDTH = 68;
constexpr int BRAKE_BAR_HEIGHT = 11;
constexpr size_t BRAKE_BAR_INTERIOR_WIDTH = 66;
constexpr int BRAKE_BAR_INTERIOR_HEIGHT = 9;

constexpr size_t brakeBarFillWidth(uint16_t raw, uint16_t minimum,
                                   uint16_t maximum, size_t pixelWidth) {
    return maximum <= minimum || raw <= minimum
               ? 0
               : (raw >= maximum
                      ? pixelWidth
                      : (size_t)((((uint32_t)raw - minimum) * pixelWidth +
                                  ((uint32_t)maximum - minimum) / 2U) /
                                 ((uint32_t)maximum - minimum)));
}

constexpr size_t displayedBrakeBarFillWidth(uint16_t raw, bool rearStale,
                                            uint16_t minimum, uint16_t maximum,
                                            size_t pixelWidth) {
    return rearStale ? 0
                     : brakeBarFillWidth(raw, minimum, maximum, pixelWidth);
}

static_assert(brakeBarFillWidth(0, 0, 4095, 66) == 0,
              "Brake minimum must draw an empty bar");
static_assert(brakeBarFillWidth(4095, 0, 4095, 66) == 66,
              "Brake maximum must fill the bar");
static_assert(brakeBarFillWidth(2048, 0, 4095, 66) == 33,
              "Brake midpoint must fill half the bar");
static_assert(brakeBarFillWidth(100, 100, 100, 66) == 0,
              "Invalid calibration must remain safe");
static_assert(brakeBarFillWidth(0, 100, 4000, 66) == 0,
              "Values below calibration must clamp to empty");
static_assert(brakeBarFillWidth(4095, 100, 4000, 66) == 66,
              "Values above calibration must clamp to full");
static_assert(displayedBrakeBarFillWidth(4095, true, 0, 4095, 66) == 0,
              "Stale rear data must draw an empty bar");

#endif
