/**
 * @file imu_reader.h
 * @brief Módulo de lectura y preprocesamiento del sensor BNO055 para ESP32.
 *
 * Proporciona funciones para inicializar, resetear, leer y gestionar el estado de la IMU.
 * Acumula la orientación absoluta (theta) desde el arranque, entrega velocidad angular (rad/s) 
 * y aceleración en el eje configurado, todo listo para integrar en el estimador de pose.
 */

#ifndef IMU_READER_H
#define IMU_READER_H

#include "vehicle_os/general_config.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>


/* ----------- Constantes del sistema ------------*/

enum class IMUdir : uint8_t {
    Y = 0, 
    X = 1,
    Z = 2
};
constexpr IMUdir ACCEL_DIR = IMUdir::X;

constexpr bool INVERT_SIGN = true;
constexpr float ACCEL_TOLERANCE = 0.03f;  // Tolerancia de aceleración para considerar ruido
constexpr float THETA_TOLERANCE = 0.001f; // Tolerancia de orientación para considerar ruido
constexpr float W_TOLERANCE = 0.01f;
constexpr float MAX_DELTA_THETA = 15 * DEG_TO_RAD;

/* -------------------- Módulo IMUSensor -------------------- */

namespace IMUSensor {

/**
 * @brief Inicializa la IMU y define el valor inicial de orientación.
 *
 * Inicializa el bus I2C, activa el cristal externo y obtiene la orientación inicial
 * para usarla como referencia acumulativa de theta.
 * 
 * @param global_acc Referencia a la variable global de aceleración lineal (m/s^2).
 * @param global_w Referencia a la variable global de velocidad angular (rad/s).
 * @param global_theta Referencia a la variable global del ángulo acumulado (rad).
 * @param imu_state Variable global de estado de la IMU.
 * @return true si la inicialización fue exitosa, false si hubo error de conexión.
 */
bool init(
    volatile float& global_acc,
    volatile float& global_w,
    volatile float& global_theta,
    volatile uint8_t& imu_state
);

/**
 * @brief Cambia el estado de operación de la IMU.
 *
 * Permite activar/desactivar la lectura de la IMU según el estado global del sistema.
 * Si se cambia a INACTIVE, se hace una última lectura de datos y se resetean las variables de aceleración y velocidad angular.
 * Si se cambia a ACTIVE, se actualiza el offset de aceleración basado en la lectura actual.
 *
 * @param new_state Nuevo estado a asignar.
 * @param imu_state Variable global de estado de la IMU.
 * @param imu_acc Referencia a la variable global de aceleración lineal (m/s^2).
 * @param imu_w Referencia a la variable global de velocidad angular (rad/s).
 * @param imu_theta Referencia a la variable global del ángulo acumulado (rad).
 * 
 */
void set_state(
    const uint8_t new_state, volatile uint8_t& imu_state,
    volatile float& imu_acc, volatile float& imu_w, volatile float& imu_theta
);

/**
 * @brief Lee los datos actuales de la IMU y actualiza las variables globales.
 *
 * Lee aceleración lineal, velocidad angular y orientación del BNO055. 
 * Aplica filtros de ruido y suma incrementalmente el ángulo acumulado desde el arranque.
 * 
 * @param global_acc Referencia a la variable global de aceleración lineal (m/s^2).
 * @param global_w Referencia a la variable global de velocidad angular (rad/s).
 * @param global_theta Referencia a la variable global del ángulo acumulado (rad).
 * @param imu_state Variable global de estado de la IMU.
 */
void read_data(
    volatile float& global_acc,
    volatile float& global_w,
    volatile float& global_theta,
    const uint8_t imu_state
);

/**
 * @brief Resincroniza el offset de aceleración basado en la lectura actual.
 *
 * Actualiza el offset de aceleración lineal para compensar el ruido y asegurar lecturas precisas.
 * Dependiendo de la dirección configurada (X o Y), se ajusta el offset correspondiente.
 */
void resync_acceleration_offset();

/**
 * @brief Tarea periódica RTOS para actualizar la lectura de la IMU.
 *
 * Llama a read_data() con el contexto global apropiado en cada periodo definido.
 * 
 * @param pvParameters Puntero al contexto global del sistema.
 */
void Task_IMUData(void* pvParameters);

} // namespace IMUSensor

#endif //IMU_READER_H