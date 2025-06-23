#ifndef TIME_UTILS_H
#define TIME_UTILS_H

#include <Arduino.h>
#include <time.h>
#include "vehicle_os/general_config.h"

// Auxiliar
constexpr uint32_t TIME_TIMEOUT_MS = 10000; // Timeout para la sincronización de hora NTP (en ms)
constexpr bool TIME_DEBUG_MODE = true || GENERAL_DEBUG_MODE;

bool init_time();
uint32_t get_unix_timestamp();                 // Actual
String timestamp_to_string(uint32_t ts);      // De Unix a legible

#endif // TIME_UTILS_H