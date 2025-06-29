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


// ---------- Enum para los returns ----------

enum class FB_Get_Result : uint8_t {
    OK = 0,              // Éxito: datos recibidos y parseados correctamente
    NO_RESULT = 1,       // Aún no disponible o no ejecutado
    ERROR = 2,           // Error en la operación asíncrona
    PARSE_ERROR = 3,     // Error al parsear JSON
    MISSING_FIELDS = 4,  // El JSON está incompleto (faltan claves requeridas)
};


// ---------- Constantes y configuraciones ----------

constexpr uint32_t FB_COMMANDS_TIMEOUT_MS = 5000;
constexpr uint8_t FB_COMMANDS_MAX_ERRORS = 2; 

constexpr uint32_t FB_PENDING_TIMEOUT_MS = 5000;
constexpr uint8_t FB_PENDING_MAX_ERRORS = 2;

constexpr uint32_t FB_PUSH_REACHED_TIMEOUT_MS = 5000;
constexpr uint8_t FB_PUSH_REACHED_MAX_ERRORS = 2; 

constexpr uint32_t FB_PUSH_REMOVE_TIMEOUT_MS = 5000;
constexpr uint8_t FB_PUSH_REMOVE_ERRORS = 2; 

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
bool ConnectFirebase();

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
 * @brief Actualiza el estado del sistema al obtener comandos desde Firebase.
 *        Internamente gestiona la solicitud, espera y procesamiento de los datos recibidos.
 * 
 * @param action Referencia donde se almacenará el comando recibido (0=STOP, 1=START).
 * @param controller_type Referencia donde se almacenará el tipo de controlador (0=PID, 1=LQR).
 * @param fb_state Referencia al estado global de la comunicación Firebase (se actualiza en caso de error).
 * @return FB_State Estado de la operación: OK, PENDING o ERROR.
 */
FB_State UpdateCommands(
    volatile uint8_t &action, volatile uint8_t &controller_type, volatile FB_State &fb_state
);

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
 * @brief Actualiza el estado al obtener el waypoint pendiente desde Firebase.
 *        Maneja internamente los reintentos y errores.
 * 
 * @param target_x Referencia para almacenar la coordenada X del waypoint.
 * @param target_y Referencia para almacenar la coordenada Y del waypoint.
 * @param target_ts Referencia para almacenar el timestamp del waypoint.
 * @param fb_state Referencia al estado global de Firebase (se actualiza si hay error).
 * @return FB_State Estado actual de la operación.
 */
FB_State UpdatePendingWaypoint(
    volatile float &target_x, volatile float &target_y, volatile uint64_t &target_ts, volatile FB_State &fb_state
);

/**
 * @brief Sube a Firebase la información de un waypoint alcanzado al nodo /waypoints_reached.
 * 
 * @param input_timestamp Timestamp original de entrada del waypoint.
 * @param wp_x Coordenada X del waypoint.
 * @param wp_y Coordenada Y del waypoint.
 * @param start_timestamp Timestamp en que se inició el movimiento.
 * @param end_timestamp Timestamp en que se terminó el movimiento hacia el waypoint.
 * @param reached_flag Flag que indica si fue alcanzado o no.
 * @param pos_x Posición X alcanzada.
 * @param pos_y Posición Y alcanzada.
 */
void PushReachedWaypoint(
    const uint64_t input_timestamp, const float wp_x, const float wp_y,  
    const uint64_t start_timestamp, const uint64_t end_timestamp, const bool reached_flag,
    const float pos_x, const float pos_y,
    const uint8_t controller_type, const float iae, const float rmse
);

/**
 * @brief Sube información de un waypoint alcanzado a Firebase con control de errores y tiempo.
 *        Esta función debe ser llamada periódicamente hasta que retorne OK o ERROR.
 * 
 * @param input_timestamp Timestamp original del waypoint.
 * @param wp_x Coordenada X del waypoint.
 * @param wp_y Coordenada Y del waypoint.
 * @param start_timestamp Timestamp de inicio de ejecución del waypoint.
 * @param end_timestamp Timestamp en que se terminó el movimiento hacia el waypoint.
 * @param reached_flag Flag que indica si fue alcanzado o no.
 * @param pos_x Posición X real alcanzada.
 * @param pos_y Posición Y real alcanzada.
 * @param controller_type Tipo de controlador utilizado.
 * @param iae Error absoluto integral.
 * @param rmse Error cuadrático medio.
 * @param fb_state Referencia al estado global de Firebase (se actualiza si hay error).
 * @return FB_State Estado de la operación (OK, PENDING o ERROR).
 */
FB_State ControlledPushReachedWaypoint(
    const uint64_t input_timestamp, const float wp_x, const float wp_y,  
    const uint64_t start_timestamp, const uint64_t end_timestamp, const bool reached_flag,
    const float pos_x, const float pos_y, 
    const uint8_t controller_type, const float iae, const float rmse,
    volatile FB_State &fb_state
);

/**
 * @brief Elimina de Firebase un waypoint pendiente por su timestamp de entrada.
 * 
 * @param input_timestamp Timestamp del waypoint que será eliminado.
 */
void RemovePendingWaypoint(const uint64_t input_timestamp);

/**
 * @brief Elimina de Firebase un waypoint pendiente con control de tiempo y errores.
 *        Esta función debe ser llamada periódicamente hasta completar o fallar.
 * 
 * @param input_timestamp Timestamp del waypoint a eliminar.
 * @param fb_state Referencia al estado global de Firebase (se actualiza si hay error).
 * @return FB_State Estado de la operación (OK, PENDING o ERROR).
 */
FB_State ControlledRemovePendingWaypoint(const uint64_t input_timestamp, volatile FB_State &fb_state);

/**
 * @brief Completa el procesamiento de un waypoint alcanzado: primero lo registra en Firebase y luego lo elimina de los pendientes.
 * 
 * Esta función debe ser llamada periódicamente dentro del ciclo operativo. Internamente controla el estado del proceso en dos pasos:
 * 
 * 1. **PUSH_REACHED**: sube la información del waypoint alcanzado a `/waypoints_reached/`.
 * 2. **REMOVE_PENDING**: elimina el waypoint correspondiente desde `/waypoints_pending/` usando su timestamp de entrada.
 * 
 * La función mantiene su estado entre llamadas y sólo retorna `FB_State::OK` cuando ambos pasos se completan con éxito. 
 * Si ocurre un error en cualquier etapa, se reinicia el flujo.
 * 
 * @param input_ts Timestamp original con que se ingresó el waypoint (clave en Firebase).
 * @param wp_x Coordenada X del waypoint objetivo.
 * @param wp_y Coordenada Y del waypoint objetivo.
 * @param start_ts Timestamp en que comenzó el seguimiento de este waypoint.
 * @param end_ts Timestamp en que se terminó el movimiento hacia el waypoint.
 * @param reached_flag Flag que indica si fue alcanzado o no.
 * @param pos_x Posición X real alcanzada al llegar al waypoint.
 * @param pos_y Posición Y real alcanzada al llegar al waypoint.
 * @param controller_type Tipo de controlador utilizado (0=PID, 1=BACKS, etc.).
 * @param iae Integral del error absoluto (IAE) acumulado en el control.
 * @param rmse Raíz del error cuadrático medio (RMSE) durante el control.
 * @param fb_state Referencia al estado global de comunicación con Firebase (se actualiza si hay errores permanentes).
 * 
 * @return FB_State Estado del flujo de control: 
 * - `OK` si ambas operaciones fueron exitosas. 
 * - `PENDING` si aún hay operaciones en curso. 
 * - `ERROR` si ocurrió un fallo permanente.
 */
FB_State CompleteWaypoint(
    const uint64_t input_ts, const float wp_x, const float wp_y,
    const uint64_t start_ts, const uint64_t end_ts, const bool reached_flag,
    const float pos_x, const float pos_y,
    const uint8_t controller_type, const float iae, const float rmse,
    volatile FB_State& fb_state
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
 * @brief Imprime en consola los logs de depuración y errores de Firebase si están disponibles.
 * 
 * @param aResult Resultado asíncrono a inspeccionar.
 */
void auth_debug_print(AsyncResult &aResult);

/**
 * @brief Tarea que envía periódicamente el estado del vehículo a Firebase.
 */
void Task_PushStatus(void *pvParameters);

/**
 * @brief Tarea que obtiene comandos desde Firebase periódicamente.
 * 
 * Esta tarea se encarga de solicitar y procesar los comandos de control del vehículo
 * desde Firebase, actualizando el estado del sistema según corresponda.
 */
void Task_GetCommands(void *pvParameters);

/**
 * @brief Tarea que mantiene la conexión con Firebase.
 */
void Task_Loop(void *pvParameters);

} // namespace FirebaseComm

#endif // FIREBASE_COMM_H
