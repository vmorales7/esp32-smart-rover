#ifndef FIREBASE_COMM_H
#define FIREBASE_COMM_H

#include <Arduino.h>
#include "vehicle_os/general_config.h"
#include "wifi_basic.h"

// Herramientas para generar cliente de Firebase
#include <WiFiClientSecure.h>
#define SSL_CLIENT WiFiClientSecure
#include "secrets.h" // Credenciales de Firebase y WiFi

// Siempre comenzar con las definiciones de preprocesador para FirebaseClient
#define ENABLE_USER_AUTH
#define ENABLE_DATABASE
#include <FirebaseClient.h>
// #include "MyFirebase.h"

// Extras para manejar datos
#include <ArduinoJson.h>

// Auxiliar para debug print
constexpr bool FB_DEBUG_MODE = true;
constexpr uint8_t PUSH_STATUS_MAX_ERRORS = 10; // Errores máximos para error crítico


// ---------- Enum para los returns ----------

enum CommandProcessResult : uint8_t {
    CMD_OK = 0,                 // Éxito
    CMD_ASYNC_NO_RESULT = 1,    // Aún no disponible o error general
    CMD_ASYNC_ERROR = 2,        // Error en la operación asíncrona
    CMD_PARSE_ERROR = 3,        // Error al parsear JSON
    CMD_MISSING_FIELDS = 4      // Falta alguna de las claves requeridas
};

enum PushStatusResult : uint8_t {
    PUSH_STATUS_OK = 0,              // Éxito al subir el estado
    PUSH_STATUS_NOT_READY = 1,       // Firebase no está listo
    PUSH_STATUS_ERROR = 2,            // Error al subir el estado
    PUSH_STATUS_MAX_ERRORS = 3      // Se alcanzó el máximo de reintentos
};




// Funciones para la comunicación con Firebase
namespace FirebaseComm {

// ---------- Inicialización ----------
bool init(SSL_CLIENT &client);
bool ready();
String lastError();

// ---------- Funciones específicas del proyecto ----------
bool getCommand(int &action, int &controller_type);

bool pushStatusLog(uint32_t timestamp, float x, float y, int rpm_L, int rpm_R, int state);
bool pushWaypointPending(uint32_t input_time, float x, float y);
bool updateCurrentWaypoint(uint32_t input_time, float x, float y);
bool pushWaypointReached(uint32_t input_time, uint32_t reached_time, float x, float y);
bool pushErrorLog(uint32_t timestamp, int controller_type, float rmse, float iae);

// ---------- Debug ----------
void auth_debug_print(AsyncResult &aResult);

} // namespace FirebaseComm

#endif // FIREBASE_COMM_H
