#include "wifi_basic.h"

bool begin_wifi() {
    if (WiFi.status() == WL_CONNECTED) {
        if (WIFI_DEBUG_MODE) Serial.println("WiFi ya está conectado.");
        return true;
    }
    // Inicio de wifi en modo de conexión (STA)
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    if (WIFI_DEBUG_MODE) {
        Serial.print("Conectando a WiFi: ");
        Serial.println(WIFI_SSID);
    }
    unsigned long start_time = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - start_time > WIFI_TIMEOUT_MS) {
            if (TIME_DEBUG_MODE) Serial.println("Fallo al conectar a WiFi (timeout)");
            return false;
        }
        delay(500);
    }
    if (WIFI_DEBUG_MODE) {
        Serial.println("Conectado a WiFi correctamente.");
        Serial.print("Dirección IP: ");
        Serial.println(WiFi.localIP());
    }
    return true;
}

bool check_wifi() {
    if (WiFi.status() == WL_CONNECTED) return true;
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    if (WIFI_DEBUG_MODE) {
        Serial.println();
        Serial.println("Conexión a WiFi perdida, intentando reconectar...");
    }
    return false;
}

bool ensure_wifi_connected() {
    if (check_wifi()) {
        if (WIFI_DEBUG_MODE) {
            Serial.println("Reconexión WiFi exitosa");
        }
        return true;
    }
    return false;
}


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
