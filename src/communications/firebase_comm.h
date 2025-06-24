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
constexpr bool FB_DEBUG_MODE = false;


// ---------- Enum para los returns ----------

enum class FB_PushType : uint8_t {
    STATUS,
    REACHED,
    REMOVE_PENDING
};

enum class FB_Get_Result : uint8_t {
    OK = 0,              // Éxito: datos recibidos y parseados correctamente
    NO_RESULT = 1,       // Aún no disponible o no ejecutado
    ERROR = 2,           // Error en la operación asíncrona
    PARSE_ERROR = 3,     // Error al parsear JSON
    MISSING_FIELDS = 4,  // El JSON está incompleto (faltan claves requeridas)
};

enum class FB_Push_Result : uint8_t {
    OK = 0,              // Éxito al subir 
    NOT_READY = 1,       // Firebase no está listo
    ERROR = 2,           // Error al subir 
    FATAL_ERROR = 3      // Se alcanzó el máximo de reintentos
};

constexpr uint8_t PUSH_STATUS_MAX_ERRORS = 100;  
constexpr uint8_t PUSH_REACHED_MAX_ERRORS = 100; 
constexpr uint8_t PUSH_REMOVE_MAX_ERRORS = 100; 


// ---------- Funciones para la comunicación con Firebase ----------

namespace FirebaseComm {

/**
 * @brief Inicializa la conexión Firebase con autenticación de usuario y cliente asíncrono.
 * 
 * @return true si se inicializó correctamente.
 */
bool SetupFirebaseConnect();

/**
 * @brief Verifica si la app Firebase está lista para operar.
 * 
 * @return true si está inicializada y lista.
 */
bool ready();

/**
 * @brief Envía un documento JSON a la base de datos en una ruta específica de forma asíncrona.
 * 
 * @param path Ruta en Firebase RTDB.
 * @param doc Documento JSON a subir.
 * @param result Objeto AsyncResult donde se almacenará la respuesta.
 */
void SetJson(const String &path, const JsonDocument &doc, AsyncResult &result);

/**
 * @brief Envia una solicitud para obtener el nodo de comandos desde Firebase.
 */
void RequestCommands();

/**
 * @brief Procesa el resultado de la solicitud de comandos y extrae los valores esperados.
 * 
 * @param action Valor del comando `action` recibido.
 * @param controller_type Tipo de controlador recibido.
 * @return FB_Get_Result Resultado del procesamiento (éxito, error, parse error, etc.).
 */
FB_Get_Result ProcessRequestCommands(volatile uint8_t &action, volatile uint8_t &controller_type);

/**
 * @brief Solicita el waypoint pendiente más antiguo desde Firebase.
 */
void RequestPendingWaypoint();

/**
 * @brief Procesa el resultado del waypoint pendiente y extrae sus coordenadas y timestamp.
 * 
 * @param target_x Referencia a la coordenada X del waypoint objetivo.
 * @param target_y Referencia a la coordenada Y del waypoint objetivo.
 * @param target_ts Referencia al timestamp del waypoint objetivo.
 * @return FB_Get_Result Resultado del procesamiento (éxito, error, parse error, etc.).
 */
FB_Get_Result ProcessPendingWaypoint(
    volatile float &target_x, volatile float &target_y, volatile uint64_t &target_ts
);

/**
 * @brief Envía un estado de posición, velocidad, error y estado del vehículo al nodo /status_log.
 * 
 * @param state Estado del sistema.
 * @param log_msg Mensaje descriptivo del evento o estado.
 * @param x Posición X actual.
 * @param y Posición Y actual.
 * @param wL Velocidad angular izquierda.
 * @param wR Velocidad angular derecha.
 * @param wp_input_ts Timestamp de entrada del waypoint.
 * @param wp_x Coordenada X del waypoint activo.
 * @param wp_y Coordenada Y del waypoint activo.
 * @param controller_type Tipo de controlador en uso.
 * @param iae Error absoluto integral.
 * @param rmse Error cuadrático medio.
 */
void PushStatus(
    const uint8_t state, const char* log_msg, 
    const float x, const float y, const float wL, const float wR,
    const uint64_t wp_input_ts, const float wp_x, const float wp_y,
    const uint8_t controller_type, const float iae, const float rmse
);

/**
 * @brief Sube a Firebase la información de un waypoint alcanzado al nodo /waypoints_reached.
 * 
 * @param input_timestamp Timestamp original de entrada del waypoint.
 * @param wp_x Coordenada X del waypoint.
 * @param wp_y Coordenada Y del waypoint.
 * @param start_timestamp Timestamp en que se inició el movimiento.
 * @param reached_timestamp Timestamp en que se alcanzó el waypoint.
 * @param pos_x Posición X alcanzada.
 * @param pos_y Posición Y alcanzada.
 */
void PushReachedWaypoint(
    const uint64_t input_timestamp, const float wp_x, const float wp_y,  
    const uint64_t start_timestamp, const uint64_t reached_timestamp,
    const float pos_x, const float pos_y,
    const uint8_t controller_type, const float iae, const float rmse
);

/**
 * @brief Elimina de Firebase un waypoint pendiente por su timestamp de entrada.
 * 
 * @param input_timestamp Timestamp del waypoint que será eliminado.
 */
void RemovePendingWaypoint(const uint64_t input_timestamp);

/**
 * @brief Procesa de forma genérica un resultado de subida asíncrona (status, reached, remove).
 * 
 * @param type Tipo de operación a verificar.
 * @return FB_Push_Result Estado del resultado (OK, ERROR, NOT_READY, FATAL).
 */
FB_Push_Result ProcessPush(const FB_PushType type);

/**
 * @brief Imprime en consola los logs de depuración y errores de Firebase si están disponibles.
 * 
 * @param aResult Resultado asíncrono a inspeccionar.
 */
void auth_debug_print(AsyncResult &aResult);

} // namespace FirebaseComm

#endif // FIREBASE_COMM_H
