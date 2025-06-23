#ifndef FIREBASE_COMM_H
#define FIREBASE_COMM_H

#include <Arduino.h>
#include "vehicle_os/general_config.h"

// Herramientas para WiFi
#include <WiFi.h>
#include <WiFiClientSecure.h>
#define SSL_CLIENT WiFiClientSecure
#include "secrets.h" // Credenciales de Firebase y WiFi

// Siempre comenzar con las definiciones de preprocesador para FirebaseClient
#define ENABLE_USER_AUTH
#define ENABLE_DATABASE
#include <FirebaseClient.h>
// #include "MyFirebase.h"

// Importar las funciones de tiempo
#include "time_utils.h"

// Extras para manejar datos
#include <ArduinoJson.h>

// Auxiliar para debug print
constexpr bool FB_DEBUG_MODE = true || GENERAL_DEBUG_MODE;

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
