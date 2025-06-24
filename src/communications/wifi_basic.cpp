#include "wifi_basic.h"

bool begin_wifi() {
    if (WiFi.status() == WL_CONNECTED) {
        if (WIFI_DEBUG_MODE) Serial.println("WiFi ya está conectado.");
        return true;
    }

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    if (WIFI_DEBUG_MODE)  Serial.print("Conectando a WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        if (WIFI_DEBUG_MODE) Serial.print(".");
    }
    if (WIFI_DEBUG_MODE) {
        Serial.println();
        Serial.println("Conectado a WiFi correctamente.");
        Serial.print("IP asignada: ");
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

    if (TIME_DEBUG_MODE) {
        Serial.print("\nSincronizando hora con NTP");
    }

    while (!getLocalTime(&timeinfo)) {
        delay(500);
        if (TIME_DEBUG_MODE) Serial.print(".");
    }

    if (TIME_DEBUG_MODE) {
        Serial.println();
        Serial.println("Hora sincronizada correctamente con NTP.");
        Serial.print("Hora actual: ");
        Serial.println(timestamp_to_string(mktime(&timeinfo)));
    }
    return true;
}

uint32_t get_unix_timestamp() {
    // Usar directamente el reloj del sistema una vez sincronizado
    return static_cast<uint32_t>(time(nullptr));
}

String timestamp_to_string(uint32_t timestamp) {
    time_t raw = static_cast<time_t>(timestamp);
    struct tm* timeinfo = localtime(&raw);

    if (!timeinfo) return "0000-00-00 00:00:00";

    char buffer[25];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo);
    return String(buffer);
}
