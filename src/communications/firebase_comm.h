#ifndef FIREBASE_COMM_H
#define FIREBASE_COMM_H

// Generales
#include <Arduino.h>
#include "vehicle_os/general_config.h"
#include "vehicle_os/vehicle_os.h"

// Herramientas para generar cliente de Firebase
#include "wifi_basic.h"
#include <WiFiClientSecure.h>
#include "secrets.h" // Credenciales de Firebase y WiFi

// Siempre comenzar con las definiciones de preprocesador para FirebaseClient
#define ENABLE_USER_AUTH
#define ENABLE_DATABASE
#include <FirebaseClient.h>

// Extras para manejar datos
#include <ArduinoJson.h>

// Auxiliar para debug print
constexpr bool FB_DEBUG_MODE = false;


// ---------- Enum para los returns ----------

enum class FB_Get_Result : uint8_t {
    OK = 0,              // Éxito: datos recibidos y parseados correctamente
    NO_RESULT = 1,       // Aún no disponible o no ejecutado
    ERROR = 2,           // Error en la operación asíncrona
    PARSE_ERROR = 3,     // Error al parsear JSON
    MISSING_FIELDS = 4,  // El JSON está incompleto (faltan claves requeridas)
};


// ---------- Constantes y configuraciones ----------

constexpr uint32_t FB_COMMANDS_TIMEOUT_MS = 1100;
constexpr uint8_t FB_COMMANDS_MAX_ERRORS = 10; 

constexpr uint32_t FB_PENDING_TIMEOUT_MS = 3000;
constexpr uint8_t FB_PENDING_MAX_ERRORS = 10;

constexpr uint32_t FB_PUSH_REACHED_TIMEOUT_MS = 10000;
constexpr uint8_t FB_PUSH_REACHED_MAX_ERRORS = 10; 

constexpr uint32_t FB_PUSH_REMOVE_TIMEOUT_MS = 10000;
constexpr uint8_t FB_PUSH_REMOVE_MAX_ERRORS = 10; 

constexpr uint32_t FB_PUSH_CLEAR_TIMEOUT_MS = 5000;
constexpr uint8_t FB_PUSH_CLEAR_MAX_ERRORS = 10;


// ---------- Funciones para la comunicación con Firebase ----------

namespace FirebaseComm {

/**
 * @brief Inicializa la conexión Firebase con autenticación de usuario y cliente asíncrono.
 * 
 * @return true si se inicializó correctamente.
 */
bool ConnectFirebase();

/**
 * @brief Libera todos los objetos dinámicos asociados a la conexión con Firebase.
 * 
 * Esta función elimina y pone en nullptr los punteros globales creados dinámicamente 
 * para manejar la autenticación, cliente HTTPS, y base de datos en tiempo real (RTDB).
 * 
 * Debe ser llamada antes de una nueva inicialización mediante `ConnectFirebase()` en 
 * caso de reconexión WiFi, errores de autenticación o reinicio del sistema de comunicación.
 * 
 * Es segura para ser llamada incluso si los objetos aún no han sido inicializados.
 * 
 * @note Esta función no reinicia el estado de comunicación (`FB_State`) ni reinicia tareas RTOS.
 */
void ResetFirebase();


/**
 * @brief Verifica si la app Firebase está lista para operar. Mantiene el loop, actualizando valores y estados.
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
    volatile float &target_x, volatile float &target_y, 
    volatile uint64_t &target_ts, volatile uint64_t &last_completed_ts,
    volatile FB_State &fb_state
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
 * @brief Ejecuta el flujo completo para registrar un waypoint alcanzado y eliminarlo de la cola de pendientes en Firebase.
 * 
 * Esta función implementa un proceso asíncrono que realiza en paralelo las siguientes dos operaciones:
 * 
 * 1. **Push a `/waypoints_finalized/`**: Sube los datos del waypoint completado para registro y análisis posterior.
 *    - Utiliza `ControlledPushReachedWaypoint()`, que gestiona sus propios reintentos y errores internos.
 *    - Si esta operación falla, no bloquea el sistema: se permite continuar mientras se elimine el waypoint pendiente.
 * 
 * 2. **Eliminación de `/waypoints_pending/`**: Borra el waypoint desde la cola de pendientes usando su `input_timestamp`.
 *    - Utiliza `ControlledRemovePendingWaypoint()`.
 *    - Esta etapa es crítica. Si falla, la función seguirá intentando hasta completarla correctamente.
 * 
 * La función debe ser llamada periódicamente desde el ciclo principal (`VehicleOS`) cuando el campo 
 * `fb_completed_but_not_sent` esté en `true`. Cada llamada evalúa el estado actual de ambas operaciones.
 * 
 * @param input_ts Timestamp de entrada del waypoint (clave en `/waypoints_pending/`).
 * @param wp_x Coordenada X del waypoint objetivo.
 * @param wp_y Coordenada Y del waypoint objetivo.
 * @param start_ts Timestamp cuando comenzó el seguimiento del waypoint.
 * @param end_ts Timestamp cuando finalizó el seguimiento del waypoint.
 * @param reached_flag Indica si el waypoint fue efectivamente alcanzado (`true`) o descartado/lógico (`false`).
 * @param pos_x Posición X real del vehículo al finalizar el trayecto.
 * @param pos_y Posición Y real del vehículo al finalizar el trayecto.
 * @param controller_type Tipo de controlador usado (0 = clásico, 1 = avanzado).
 * @param iae Integral del error absoluto (IAE) durante el trayecto.
 * @param rmse Raíz del error cuadrático medio (RMSE) durante el trayecto.
 * @param fb_state Referencia al estado global de Firebase (puede actualizarse si ocurre un error permanente).
 * 
 * @return FB_State Estado actual del proceso:
 * - `FB_State::OK`: ambas operaciones completadas (push y eliminación).
 * - `FB_State::PENDING`: al menos una operación sigue en curso, debe volver a llamarse en el siguiente ciclo.
 * - `FB_State::ERROR`: ocurrió un error permanente al eliminar el waypoint pendiente.
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
 * @brief Elimina todos los registros de estado y waypoints finalizados en Firebase.
 *
 * Esta función envía solicitudes asíncronas para eliminar los nodos `/status_log` y 
 * `/waypoints_finalized` de la base de datos Firebase. Se debe invocar periódicamente hasta 
 * que retorne un estado distinto de `FB_State::PENDING`.
 * 
 * Internamente, la función realiza:
 * - Solicitud asíncrona de eliminación si no hay una en curso.
 * - Verificación de resultados de las operaciones previas (éxito o error).
 * - Control de errores y reintentos en caso de fallas o timeout.
 * 
 * El resultado final puede ser:
 * - `FB_State::OK`: ambos nodos fueron eliminados correctamente.
 * - `FB_State::ERROR`: ocurrió un error en al menos una de las eliminaciones, o se alcanzó el máximo de errores.
 * - `FB_State::PENDING`: operación en curso; se debe seguir llamando hasta resolver.
 *
 * @param fb_state Referencia al estado de comunicación general con Firebase, que será actualizado.
 * 
 * @return FB_State Estado actual de la operación: OK, PENDING o ERROR.
 */
FB_State ClearAllLogs(volatile FB_State &fb_state);

/**
 * @brief Elimina todos los waypoints pendientes en Firebase.
 * 
 * Realiza una operación asíncrona `remove()` sobre el nodo `/waypoints_pending/`.
 * Esta función es no bloqueante: debes llamarla repetidamente hasta que retorne OK o ERROR.
 * 
 * @param fb_state Variable de estado que será actualizada según el progreso.
 * @return FB_State Estado de la operación: OK, PENDING o ERROR.
 */
FB_State ClearPendingWaypoints(volatile FB_State& fb_state);


/**
 * @brief Elimina todos los logs, waypoints finalizados, waypoints pendientes y fuerza /commands a IDLE, de forma atómica.
 * 
 * Esta función ejecuta la operación como una sola “transacción lógica”, manejando sus propios timeouts y errores.
 * Solo vuelve a permitir una nueva operación cuando termina o falla el proceso completo.
 * 
 * @param fb_state Referencia al estado global de Firebase.
 * @return FB_State OK si todo fue exitoso, ERROR si falló algún paso, PENDING si aún está en progreso.
 */
FB_State FullReset(volatile FB_State& fb_state);

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

/**
 * @brief Suspende todas las tareas relacionadas con la comunicación Firebase.
 *
 * Esta función detiene temporalmente la ejecución de las tareas RTOS encargadas de la
 * lectura y escritura asíncrona con Firebase, evitando condiciones de carrera y
 * corrupción de memoria durante operaciones críticas (como reseteos o cambios de estado).
 *
 * @param tasks Referencia a la estructura TaskHandlers que contiene los manejadores de las tareas.
 */
void suspend_firebase_tasks(TaskHandlers& tasks);

/**
 * @brief Reanuda todas las tareas relacionadas con la comunicación Firebase.
 *
 * Esta función reactiva la ejecución de las tareas RTOS encargadas de la
 * comunicación asíncrona con Firebase, permitiendo reanudar el flujo normal
 * de lectura y escritura tras finalizar operaciones críticas.
 *
 * @param tasks Referencia a la estructura TaskHandlers que contiene los manejadores de las tareas.
 */
void resume_firebase_tasks(TaskHandlers& tasks);


} // namespace FirebaseComm

#endif // FIREBASE_COMM_H
