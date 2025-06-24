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

// Comunicación (a futuro)
// #include "communication/firebase_comm.h" // Solo si empiezas a usar Firebase

// Control de evasión de obstáculos
#include "vehicle_os/evade_controller.h"

/* ------------------------ Constantes ------------------------*/

constexpr float DIST_DEBUGGEO_PUNTO_EVASION= 0.5f;

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
void update(GlobalContext* ctx_ptr);

/**
 * @brief Establece el siguiente punto objetivo desde la trayectoria
 * 
 * Extrae el primer punto de la trayectoria y lo asigna como punto objetivo.
 * Reinicia la bandera de objetivo alcanzado.
 * 
 * @param ctx_ptr Puntero al contexto global con datos del sistema
 * @return true si se estableció un punto objetivo, false si no hay puntos en la trayectoria
 */
bool set_waypoint(GlobalContext* ctx_ptr);

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
 * @brief Inicializa la trayectoria con valores nulos
 * 
 * Establece todos los puntos de la trayectoria con valores NULL_WAYPOINT_XY
 * y establece el contador de puntos a cero.
 * 
 * @param os Referencia a la estructura de datos del sistema operativo
 */
void clear_trajectory_with_null(volatile OperationData& os);

/**
 * @brief Completa el waypoint actual y desplaza los siguientes en la trayectoria
 * 
 * Elimina el primer punto de la trayectoria y desplaza todos los demás un lugar hacia adelante.
 * El último punto se establece como nulo y se reduce el contador de puntos.
 * 
 * @param os Referencia a la estructura de datos del sistema operativo
 */
bool complete_current_waypoint(volatile OperationData& os);

/**
 * @brief Agrega un nuevo waypoint al final de la trayectoria
 * 
 * Añade un punto (x,y) al final de la trayectoria si hay espacio disponible.
 * 
 * @param x Coordenada X del nuevo waypoint [m]
 * @param y Coordenada Y del nuevo waypoint [m]
 * @param os Referencia a la estructura de datos del sistema operativo
 * @return true si se agregó correctamente, false si la trayectoria está llena
 */
bool add_waypoint(const float x, const float y, volatile OperationData& os);

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