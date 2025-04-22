#include "firebase_comm.h"

using namespace firebase_ns;

Objeto global
static FirebaseClient fb;

namespace FirebaseComm {

    void init_wifi() {
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        Serial.print("Conectando a WiFi...");
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }
        Serial.println("\n✅ WiFi conectado");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
    }

    void init_firebase() {
        FirebaseConfig config;
        config.api_key = FIREBASE_API_KEY;
        config.database_url = FIREBASE_DATABASE_URL;

        Signer signer;
        signer.user_email = USER_EMAIL;
        signer.user_password = USER_PASSWORD;

        fb.begin(&config, &signer);
        Serial.println("🔥 FirebaseClient inicializado");
    }

    void firebase_loop() {
        fb.loop();  // mantiene la conexión viva
    }

    bool firebase_ready() {
        return fb.ready();
    }

    /**
     * @brief Lee un valor booleano desde un nodo.
     */
    bool read_bool(const String& path) {
        FirebaseJson result;
        if (fb.RTDB.get(&result, path)) {
            return result.to<bool>();
        }
        return false;
    }

    /**
     * @brief Lee un valor float desde un nodo.
     */
    float read_float(const String& path) {
        FirebaseJson result;
        if (fb.RTDB.get(&result, path)) {
            return result.to<float>();
        }
        return 0.0f;
    }

    /**
     * @brief Escribe un float en Firebase RTDB.
     */
    void write_float(const String& path, float value) {
        fb.RTDB.set(path.c_str(), value);
    }

    /**
     * @brief Sube la pose (x, y, theta) como JSON.
     */
    void upload_pose(const String& path, float x, float y, float theta) {
        FirebaseJson json;
        json.set("x", x);
        json.set("y", y);
        json.set("theta", theta);
        fb.RTDB.set(path.c_str(), json);
    }

}
