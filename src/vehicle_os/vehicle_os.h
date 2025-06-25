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
#include "communications/wifi_basic.h"
#include "communications/firebase_comm.h"

/* ------------------------ Constantes ------------------------*/



/* ------------------------ Funciones ------------------------*/

/**
 * @brief Namespace que contiene las funciones del sistema operativo del vehículo
 */
namespace OS {

/**
 * @brief Actualiza el sistema operativo según el estado actual
 * 
 * Esta función implementa la máquina de estados principal del sistema operativo.
 * Se encarga de gestionar las transiciones entre estados y ejecutar las accionescorrespondientes a cada estado.
 *
 * @param ctx_ptr Puntero al contexto global con datos del sistema
 */
void update_local(GlobalContext* ctx_ptr);

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


bool CheckOnlineStatus(GlobalContext* ctx_ptr);

/**
 * @brief Ejecuta el flujo completo para marcar como completado un waypoint en Firebase.
 *
 * Esta función extrae una copia de los datos actuales del waypoint (`fb_waypoint_data`)
 * almacenado en `OperationData`, y luego ejecuta secuencialmente las siguientes acciones:
 * 1. Realiza un push al nodo `/waypoints_reached/` con la información del waypoint completado.
 * 2. Elimina el mismo waypoint desde `/waypoints_pending/` utilizando su timestamp de entrada.
 *
 * Este proceso se realiza de forma controlada, manejando los estados internos y errores posibles.
 * La función está diseñada para ser llamada repetidamente desde el Vehicle OS durante el estado
 * `STAND_BY`, y solo retornará `FB_State::OK` una vez que ambas operaciones (push y delete)
 * hayan sido completadas exitosamente.
 *
 * @param ctx_ptr Puntero al contexto global del sistema (`GlobalContext`).
 * @return FB_State Estado actual del proceso:
 *         - `FB_State::PENDING` si alguna acción sigue en curso.
 *         - `FB_State::OK` si se completaron exitosamente ambas acciones.
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
 * @brief Registra los datos del waypoint alcanzado en la estructura de datos del sistema operativo
 * 
 * Guarda los datos del waypoint alcanzado, incluyendo timestamp, posición y tipo de controlador.
 * Estos datos se utilizan para enviar información a Firebase sobre el waypoint completado.
 * 
 * @param ctx_ptr Puntero al contexto global con datos del sistema
 */
void register_finished_waypoint_data(GlobalContext* ctx_ptr);

/**
 * @brief Tarea RTOS para el sistema operativo del vehículo
 * 
 * Ejecuta periódicamente la función update() para mantener actualizado el estado del sistema.
 * 
 * @param pvParameters Puntero a los parámetros de la tarea (GlobalContext)
 */
void Task_VehicleOS(void* pvParameters);

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

} // namespace OS



#endif // VEHICLE_OS_H