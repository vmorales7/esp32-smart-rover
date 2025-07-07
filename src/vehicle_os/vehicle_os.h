#ifndef VEHICLE_OS_H
#define VEHICLE_OS_H

/* ------------------------ Mis librerías ------------------------*/

#include "vehicle_os/general_config.h"

// Sensores
#include "sensors_firmware/encoder_reader.h"
#include "sensors_firmware/distance_sensors.h"
#include "sensors_firmware/imu_reader.h"

// Estimación y control
#include "position_system/pose_estimator.h"
#include "position_system/position_controller.h"

// Control de motores
#include "motor_drive/motor_controller.h"

// Control de evasión de obstáculos
#include "vehicle_os/evade_controller.h"

// Comunicación
#include "communications/firebase_comm.h"
#include "communications/wifi_basic.h"

/* ------------------------ Constantes ------------------------*/

constexpr bool OS_DEBUG_MODE = false;
constexpr float MIN_EVADE_BEHIND_DIST = 0.2f; // Distancia mínima obstáculo-waypoint para iniciar evasión [m]
constexpr float MAX_EVADE_SKIP_DIST = 0.4f; // Distancia máxima para saltar un waypoint evasión [m]
constexpr bool NO_OBSTACLES_MODE = false; // Desactivar detección de obstáculos y evasión (para pruebas)


/* ------------------------ Funciones ------------------------*/

/**
 * @brief Namespace que contiene las funciones del sistema operativo del vehículo
 */
namespace OS {

/**
 * @brief Actualiza el sistema operativo según el estado actual, para operación local
 * 
 * Esta función implementa la máquina de estados principal del sistema operativo.
 * Se encarga de gestionar las transiciones entre estados y ejecutar las accionescorrespondientes a cada estado.
 *
 * @param ctx_ptr Puntero al contexto global con datos del sistema
 */
void update_local(GlobalContext* ctx_ptr);

/**
 * @brief Actualiza el sistema operativo según el estado actual, para operación online
 * 
 * Esta función implementa la máquina de estados principal del sistema operativo.
 * Se encarga de gestionar las transiciones entre estados y ejecutar las accionescorrespondientes a cada estado.
 *
 * @param ctx_ptr Puntero al contexto global con datos del sistema
 */
void update_online(GlobalContext* ctx_ptr);

/**
 * @brief Realiza las operaciones de inicialización del sistema
 * 
 * Inicializa todos los subsistemas: encoders, estimador de pose, motores,
 * sensores de distancia, IMU (futuro) y controlador de posición.
 * 
 * @param ctx_ptr Puntero al contexto global con datos del sistema
 */
bool enter_init(GlobalContext* ctx_ptr);

/**
 * @brief Realiza las operaciones para entrar al estado IDLE
 * 
 * Desactiva motores, sensores y controladores para minimizar consumo energético y permitir manipular el vehículo.
 * Reinicia la pose del vehículo a cero.
 * 
 * @param ctx Puntero al contexto global con datos del sistema
 */
bool enter_idle(GlobalContext* ctx);

/**
 * @brief Realiza las operaciones para entrar al estado STAND_BY
 * 
 * Mantiene el vehículo con referencia 0 de velocidad, pero con sensores y estimación de posición activados.
 * Se desactivan los sensores de distancia y se limpian (forzadamente) las banderas de obstáculos.
 * Se deja el modo de control de posición en manual, con velocidad de referencia 0 -> permite fijar referencia de posición.
 * 
 * @param ctx Puntero al contexto global con datos del sistema
 */
bool enter_stand_by(GlobalContext* ctx);

/**
 * @brief Realiza las operaciones para entrar al estado ALIGN
 * 
 * Configura el sistema para alinear el vehículo hacia el objetivo,
 * activando los sensores y el estimador de posición, y configurando el controlador de posición.
 * @param ctx Puntero al contexto global con datos del sistema
 * @return true si se configuró correctamente la alineación, false en caso contrario
 */
bool enter_align(GlobalContext* ctx);

/**
 * @brief Realiza las operaciones para entrar al estado MOVE
 * 
 * Activa todos los sensores y controladores, establece el punto objetivo como el primer punto de la lista de trayectoria,
 * y configura el controlador de posición según el tipo de control seleccionado. 
 * 
 * Genera una primera lectura de los sensores de distancia para detectar obstáculos y actualiza la bandera global.
 * 
 * @param ctx Puntero al contexto global con datos del sistema
 * @return true si se inició el movimiento correctamente, false en caso contrario
 */
bool enter_move(GlobalContext* ctx);

/**
 * @brief Realiza las operaciones para entrar al estado EVADE
 * 
 * Configura el sistema para evitar un obstáculo detectado, deteniendo el movimiento
 * y preparando el sistema para la estrategia de evasión.
 * 
 * @param ctx Puntero al contexto global con datos del sistema
 * @return true si se configuró correctamente la evasión, false en caso contrario
 */
bool enter_evade(GlobalContext* ctx);

/**
 * @brief Realiza las operaciones para permitir que el vehículo rote en torno a su eje
 * 
 * Configura el sistema para rotar el vehículo en torno a su eje, activando los sensores y el estimador de posición, 
 * y configurando el controlador de posición. Los sensores de distancia se mantienen desactivados.
 * 
 * @param ctx Puntero al contexto global con datos del sistema
 * @return true si se configuró correctamente la rotación, false en caso contrario
 */
bool enter_rotate(GlobalContext* ctx_ptr);

/**
 * @brief Realiza las operaciones para entrar al estado detenido esperando liberación de obstáculo
 * 
 * @param ctx_ptr Puntero al contexto global con datos del sistema
 * @return true si se configuró correctamente el estado de espera, false en caso contrario
 */
bool enter_wait_free_path(GlobalContext* ctx_ptr);

/**
 * @brief Establece el siguiente punto objetivo desde la trayectoria
 * 
 * Extrae el primer punto de la trayectoria y lo asigna como punto objetivo.
 * Reinicia la bandera de objetivo alcanzado.
 * 
 * @param ctx_ptr Puntero al contexto global con datos del sistema
 * @return true si se estableció un punto objetivo, false si no hay puntos en la trayectoria
 */
bool set_local_waypoint(GlobalContext* ctx_ptr);

/**
 * @brief Inicializa la trayectoria con valores nulos
 * 
 * Establece todos los puntos de la trayectoria con valores NULL_WAYPOINT_XY
 * y establece el contador de puntos a cero.
 * 
 * @param os Referencia a la estructura de datos del sistema operativo
 */
void clear_local_trajectory(volatile OperationData& os);

/**
 * @brief Completa el waypoint actual y desplaza los siguientes en la trayectoria
 * 
 * Elimina el primer punto de la trayectoria y desplaza todos los demás un lugar hacia adelante.
 * El último punto se establece como nulo y se reduce el contador de puntos.
 * 
 * @param os Referencia a la estructura de datos del sistema operativo
 */
bool complete_local_waypoint(volatile OperationData& os);

/**
 * @brief Agrega un nuevo waypoint al final de la trayectoria
 * 
 * Añade un punto (x,y) al final de la trayectoria si hay espacio disponible.
 * 
 * @param x Coordenada X del nuevo waypoint [m]
 * @param y Coordenada Y del nuevo waypoint [m]
 * @param ts Timestamp del nuevo waypoint [epoch unix timestamp]
 * @param os Referencia a la estructura de datos del sistema operativo
 * @return true si se agregó correctamente, false si la trayectoria está llena
 */
bool add_local_waypoint(const float x, const float y, const float ts, volatile OperationData& os);


/**
 * @brief Verifies whether the system is considered online based on WiFi and Firebase status.
 *
 * This function checks two conditions to determine if the system can operate online:
 * - WiFi connection must not be in TIMEOUT state.
 * - Firebase must not be in CONNECTION_ERROR state.
 *
 * Both conditions are evaluated using flags (`wifi_status` and `fb_state`) updated by independent RTOS tasks.
 * This approach ensures a non-blocking and fast response suitable for real-time systems.
 *
 * @param ctx_ptr Pointer to the global system context (`GlobalContext*`).
 * @return true if both WiFi and Firebase are considered online.
 * @return false if either condition indicates offline status.
 */
bool CheckOnlineStatus(GlobalContext *ctx_ptr);

/**
 * @brief Resets the vehicle's operational state when transitioning to online mode.
 *
 * This function performs a complete reset of both local and remote (Firebase) status data. 
 * It ensures that local reset operations (e.g., control buffers, waypoint tracking flags) 
 * are executed only once per reset attempt, and avoids repeated resets until Firebase deletion completes.
 *
 * The remote reset involves deleting the `/status_log` and `/waypoints_finalized` nodes from Firebase 
 * by calling `FirebaseComm::ClearAllLogs()`. The operation is asynchronous and may require multiple 
 * calls until it completes or fails.
 *
 * This function must be called repeatedly (e.g., during IDLE state) until it returns `FB_State::OK` 
 * or `FB_State::ERROR`. It returns `FB_State::PENDING` if the Firebase operation is still in progress.
 *
 * @param ctx_ptr Pointer to the global system context (`GlobalContext*`).
 * @return FB_State
 * - `FB_State::OK`: Reset completed successfully.
 * - `FB_State::PENDING`: Firebase deletion still in progress.
 * - `FB_State::ERROR`: Firebase deletion failed.
 */
FB_State reset_online_status(GlobalContext *ctx_ptr);

/**
 * @brief Determines whether the current waypoint should be skipped due to obstacle proximity.
 *
 * This function compares the distance to the current waypoint against the minimum distance
 * reported by the ultrasonic sensors. If the waypoint is very close and directly blocked 
 * by an obstacle with little space behind it, the waypoint is considered unreachable and 
 * should be skipped.
 *
 * Conditions for skipping:
 * - Waypoint is closer than `MAX_EVADE_SKIP_DIST`
 * - Obstacle is closer than the waypoint
 * - The space behind the obstacle is less than `MIN_EVADE_BEHIND_DIST`
 *
 * @param ctx_ptr Pointer to the global system context (`GlobalContext*`)
 * @return true if the waypoint should be skipped due to blockage
 * @return false if the system should attempt obstacle evasion
 */
bool check_skip_evade(GlobalContext *ctx_ptr);

/**
 * @brief Ejecuta el flujo completo para marcar como completado un waypoint en Firebase.
 *
 * Esta función extrae los datos actuales del waypoint (`fb_waypoint_data`) almacenado en
 * `OperationData`, y ejecuta el flujo `CompleteWaypoint()` del módulo `FirebaseComm`.
 * Este flujo realiza las siguientes acciones en orden:
 * 1. Realiza un push del waypoint al nodo `/waypoints_finalized/`, incluyendo si fue alcanzado o fallado.
 * 2. Elimina el waypoint correspondiente del nodo `/waypoints_pending/` usando su timestamp de entrada.
 *
 * La función está diseñada para ser llamada repetidamente desde el Vehicle OS durante el estado
 * `STAND_BY`. El estado retornado indicará el progreso o errores del proceso.
 *
 * Si el flujo se completa con éxito (`FB_State::OK`), se limpia automáticamente la bandera
 * `fb_completed_but_not_sent` para evitar intentos de reenvío.
 *
 * @param ctx_ptr Puntero al contexto global del sistema (`GlobalContext`).
 * @return FB_State Estado actual del proceso:
 *         - `FB_State::PENDING` si alguna operación sigue en curso.
 *         - `FB_State::OK` si ambas acciones fueron completadas exitosamente.
 *         - `FB_State::ERROR` si ocurrió un error permanente.
 */
FB_State SendReachedWaypoint(GlobalContext* ctx_ptr);

/**
 * @brief Estalece el waypoint desde la data recibida de Firebase y guarda los datos iniciales
 * 
 * Configura el waypoint actual en el controlador de posición a partir de los datos recibidos de Firebase.
 * Guarda los datos iniciales del waypoint en el buffer de datos del sistema operativo.
 * @param ctx_ptr Puntero al contexto global con datos del sistema
 * 
 */
void set_online_waypoint(GlobalContext* ctx_ptr);

/**
 * @brief Registra los datos del waypoint finalizado en la estructura de datos del sistema operativo.
 * 
 * Esta función guarda la información del waypoint al momento de ser completado o fallado, incluyendo:
 * - Timestamp de finalización (end_ts)
 * - Posición actual del vehículo (pos_x, pos_y)
 * - Tipo de controlador utilizado
 * - Métricas de desempeño (IAE, RMSE)
 * - Indicador de éxito (reached_flag)
 * 
 * Estos datos son utilizados posteriormente para ser enviados a Firebase mediante el nodo
 * `/waypoints_finalized/`.
 * 
 * @param reached_flag Indica si el waypoint fue exitosamente alcanzado (`true`) o fallado (`false`)
 * @param ctx_ptr Puntero al contexto global del sistema
 */
void register_finished_waypoint_data(const bool reached_flag, GlobalContext* ctx_ptr);

/**
 * @brief Tarea RTOS para el sistema operativo del vehículo
 * 
 * Ejecuta periódicamente la función update() para mantener actualizado el estado del sistema.
 * 
 * @param pvParameters Puntero a los parámetros de la tarea (GlobalContext)
 */
void Task_VehicleOS(void* pvParameters);

/**
 * @brief Tarea periódica para detener el movimiento del vehículo ante condiciones de riesgo.
 * 
 * Esta tarea se ejecuta en segundo plano cada `OS_CHECK_STOP_PERIOD_MS` milisegundos y supervisa condiciones críticas
 * que requieren una detención inmediata del vehículo. No realiza transiciones de estado, solo fuerza `stop_movement()`
 * para bloquear las referencias del controlador de posición y mantener el vehículo quieto hasta que el sistema principal
 * (`VehicleOS::update`) tome una decisión.
 * 
 * Se evalúan las siguientes condiciones:
 * - Si el vehículo está en estado `MOVE` y se detecta un obstáculo (`us_obstacle == true`), se detiene.
 * - Si el vehículo está en `MOVE` o `ALIGN` y el comando desde Firebase no es `START`, se detiene.
 * - Si el vehículo está en `MOVE` o `ALIGN`, y se encuentra en modo `ONLINE`, y se detecta pérdida de conexión (`wifi_status == TIMEOUT`), se detiene.
 * 
 * @param pvParameters Puntero al `GlobalContext` del sistema.
 */
void Task_StopOnRiskFlags(void *pvParameters);

/**
 * @brief Establece un mensaje de log en la estructura de datos del sistema operativo
 * 
 * Guarda un mensaje de log en la estructura de datos del sistema operativo.
 * Se asegura de que el mensaje esté correctamente terminado con null.
 * 
 * @param new_state Nuevo estado del sistema operativo
 * @param old_state Estado anterior del sistema operativo
 * @param os Referencia a la estructura de datos del sistema operativo
 */
void set_operation_log(const OS_State new_state, const OS_State old_state, GlobalContext* ctx_ptr);

void reset_firebase_data(GlobalContext *ctx_ptr);

} // namespace OS

/**
 * @brief Convierte un valor entero recibido desde Firebase al comando de usuario correspondiente.
 * 
 * @param val Valor numérico recibido (0 = STOP, 1 = START, 2 = IDLE).
 * @return UserCommand Enum representando el comando de usuario.
 */
UserCommand Int2Cmd(uint8_t val);

/**
 * @brief Convierte un comando de usuario (`UserCommand`) a su valor numérico equivalente para escribir en Firebase.
 * 
 * @param cmd Comando de usuario (STOP, START, IDLE).
 * @return uint8_t Valor entero (0 = STOP, 1 = START, 2 = IDLE).
 */
uint8_t Cmd2Int(UserCommand cmd);

/**
 * @brief Convierte un valor entero recibido desde Firebase al tipo de controlador de posición.
 * 
 * @param val Valor numérico recibido (0 = PID, 1 = BACKS).
 * @return ControlType Enum representando el tipo de controlador.
 */
ControlType Int2CtrlType(uint8_t val);

/**
 * @brief Convierte un tipo de controlador (`ControlType`) a su valor numérico equivalente para escribir en Firebase.
 * 
 * @param ct Tipo de controlador (PID, BACKS).
 * @return uint8_t Valor entero (0 = PID, 1 = BACKS).
 */
uint8_t CtrlType2Int(ControlType ct);

#endif // VEHICLE_OS_H