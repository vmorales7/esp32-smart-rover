#ifndef WIFI_UTILS_H
#define WIFI_UTILS_H

#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include "secrets.h" // Credenciales WiFi
#include "vehicle_os/general_config.h"


// Configuración de WiFi

constexpr uint32_t WIFI_TIMEOUT_MS = 10000; // Timeout para la conexión WiFi (en ms)
constexpr bool WIFI_DEBUG_MODE = true || GENERAL_DEBUG_MODE;

bool begin_wifi(); // Inicia la conexión WiFi
bool check_wifi(); // Verifica si WiFi está conectado
bool ensure_wifi_connected(); // Asegura que WiFi esté conectado


// Configuración de hora NTP

constexpr uint32_t TIME_TIMEOUT_MS = 10000; // Timeout para la sincronización de hora NTP (en ms)
constexpr bool TIME_DEBUG_MODE = true || GENERAL_DEBUG_MODE;

bool init_time();
uint32_t get_unix_timestamp();                // Actual
String timestamp_to_string(uint32_t ts);      // De Unix a legible

#endif // WIFI_UTILS_H