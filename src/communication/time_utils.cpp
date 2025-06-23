#include "time_utils.h"

// Configuración de hora NTP
bool init_time() {
    
    configTime(0, 0, "pool.ntp.org");
    struct tm timeinfo;
    unsigned long t0 = millis();

    if (TIME_DEBUG_MODE) {
        Serial.println();
        Serial.println("Sincronizando hora con NTP...");
        Serial.println();
    }
    while (!getLocalTime(&timeinfo)) {
        if (millis() - t0 > TIME_TIMEOUT_MS) {
            if (TIME_DEBUG_MODE) {Serial.println("No se pudo sincronizar la hora (timeout)");}
            return false;
        }
        delay(500);
    }
    if (TIME_DEBUG_MODE) {
        Serial.println("\nHora sincronizada correctamente.");
        Serial.println();
    }
    return true; 
}

uint32_t get_unix_timestamp() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        if (TIME_DEBUG_MODE) Serial.println("Fallo al obtener hora");
        return 0;
    }
    return mktime(&timeinfo);
}

String timestamp_to_string(uint32_t timestamp) {
    time_t raw = static_cast<time_t>(timestamp);
    struct tm* timeinfo = localtime(&raw);

    if (!timeinfo) return "0000-00-00 00:00:00";

    char buffer[25];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo);
    return String(buffer);
}