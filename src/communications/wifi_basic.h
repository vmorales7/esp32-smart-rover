#ifndef WIFI_BASIC_H
#define WIFI_BASIC_H

#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include "secrets.h"            ///< Credenciales WiFi (WIFI_SSID, WIFI_PASS)

// ============================
// Manejo básico de WiFi
// ============================

/// @brief Enum para representar el estado de la conexión WiFi.
enum class WifiStatus : uint8_t {
    OK = 1,
    DISCONNECTED = 2,
    TIMEOUT = 3
};

/// @brief Timeout máximo para la conexión WiFi (en milisegundos).
constexpr uint32_t WIFI_TIMEOUT_MS = 10000;

/// @brief Activa mensajes de depuración para WiFi.
constexpr bool WIFI_DEBUG_MODE = false;

/// @brief Inicia la conexión WiFi con espera activa hasta lograr conexión o timeout.
/// @return true si la conexión fue exitosa, false si falló.
bool begin_wifi();

/// @brief Verifica si el WiFi está conectado. Si no, intenta reconectar sin esperar el resultado.
/// @return true si ya estaba conectado, false si no lo estaba (aunque ya se haya iniciado reconexión).
WifiStatus check_wifi();

/// @brief Asegura que WiFi esté conectado. Internamente llama a check_wifi() y confirma si logró reconectar.
/// @return true si logró conectar, false si no fue posible.
bool ensure_wifi_connected();


// ============================
//        Sincronización NTP
// ============================

/// @brief Timeout máximo para la sincronización de hora NTP (en milisegundos).
constexpr uint32_t TIME_TIMEOUT_MS = 10000;

/// @brief Activa mensajes de depuración para NTP y timestamp.
constexpr bool TIME_DEBUG_MODE = true;

/// @brief Inicializa la hora del sistema usando NTP.
/// @return true si la sincronización fue exitosa, false si falló.
bool init_time();

/// @brief Obtiene el timestamp actual como número Unix (segundos desde 1970).
/// @return Timestamp actual, o 0 si no pudo obtenerlo.
uint32_t get_unix_timestamp();

/// @brief Convierte un timestamp Unix a string legible con formato "YYYY-MM-DD HH:MM:SS".
/// @param ts Timestamp Unix en segundos.
/// @return Fecha y hora como string legible. Si falla, retorna "0000-00-00 00:00:00".
String timestamp_to_string(uint32_t ts);

#endif // WIFI_BASIC_H
