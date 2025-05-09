#ifndef DISTANCE_SENSORS_H
#define DISTANCE_SENSORS_H

#include "project_config.h"

/* -------------------- Parámetros del sensor ultrasónico (HC-SR04) -------------------- */

/**
 * @brief Velocidad del sonido dividida por 2 en cm/µs.
 * 
 * Se usa para convertir el tiempo de ida y vuelta del pulso de ultrasonido
 * en una distancia unidireccional (desde el sensor al objeto).
 * 
 * 343 m/s → 0.0343 cm/µs → dividido por 2 = 0.01715 cm/µs
 */
constexpr float US_CM_PER_US = 0.01715f;

/**
 * @brief Distancia máxima considerada válida para el sensor [cm].
 * 
 * Si la lectura supera este valor, se considera inválida o sin respuesta
 * y se devuelve US_MAX_DISTANCE_CM.
 */
constexpr uint8_t US_MAX_DISTANCE_CM = 100;

/**
 * @brief Tiempo máximo de espera para el pulso de eco [µs].
 * 
 * Limita el tiempo de espera para evitar bloqueos prolongados si no hay reflexión.
 */
constexpr uint32_t US_PULSE_TIMEOUT_US = US_MAX_DISTANCE_CM / US_CM_PER_US;

/**
 * @brief Umbral de detección de obstáculo [cm].
 * 
 * Si la distancia medida es menor a este valor, se considera que hay un obstáculo.
 */
constexpr uint8_t OBSTACLE_THRESHOLD_CM = 30;


/* -------------------- Módulo DistanceSensors -------------------- */

namespace DistanceSensors {

    /**
     * @brief Inicializa un sensor ultrasónico individual (TRIG + ECHO).
     * 
     * Configura los pines entregados como salida (TRIG) y entrada (ECHO),
     * y se asegura de dejar el pin TRIG en estado bajo por defecto.
     * 
     * @param trig_pin Número de pin conectado al TRIG del sensor.
     * @param echo_pin Número de pin conectado al ECHO del sensor.
     */
    void init_sensor(uint8_t trig_pin, uint8_t echo_pin);

    /**
     * @brief Realiza una lectura puntual de distancia desde un sensor ultrasónico.
     * 
     * Emite un pulso TRIG y mide el tiempo hasta la recepción del pulso ECHO.
     * 
     * @param trig_pin Pin TRIG del sensor.
     * @param echo_pin Pin ECHO del sensor.
     * @return Distancia estimada en cm. Retorna US_MAX_DISTANCE_CM en caso de timeout.
     */
    uint8_t read_distance(uint8_t trig_pin, uint8_t echo_pin);

    /**
     * @brief Reinicia completamente la estructura DistanceSensorData.
     * 
     * Establece todas las distancias en US_MAX_DISTANCE_CM y todas las banderas
     * de detección de obstáculos en false. Se utiliza durante la inicialización
     * o al reiniciar el módulo de sensores.
     * 
     * @param data_ptr Puntero a la estructura DistanceSensorData que se desea reiniciar.
     */
    void reset_system(volatile DistanceSensorData* data_ptr);

    /**
     * @brief Inicializa los sensores de distancia y reinicia la estructura de datos asociada.
     * 
     * Configura los pines TRIG y ECHO de los sensores ultrasónicos (izquierdo, medio y derecho).
     * Además, deja en estado INACTIVE el módulo y reinicia la estructura `DistanceSensorData`:
     * - Todas las distancias se establecen en `US_MAX_DISTANCE_CM`.
     * - Todas las banderas de obstáculos se ponen en `false`.
     * 
     * @param distance_state_ptr Puntero al estado global del módulo de sensores (ACTIVE o INACTIVE).
     * @param data_ptr Puntero a la estructura `DistanceSensorData` que será reiniciada.
     */
    void init_system(
        volatile uint8_t* distance_state_ptr,
        volatile DistanceSensorData* data_ptr
    );

    /**
     * @brief Cambia el estado activo/inactivo del módulo de sensores de distancia.
     * 
     * @param mode Valor deseado del estado (ACTIVE o INACTIVE).
     * @param distance_state_ptr Puntero al estado global del módulo.
     */
    void set_state(uint8_t mode, volatile uint8_t* distance_state_ptr);

    /**
     * @brief Evalúa un único sensor ultrasónico y actualiza su distancia y estado de obstáculo.
     * 
     * Realiza doble lectura para mitigar errores y rebotes. Se actualizan tanto la distancia
     * como el flag de obstáculo si el sistema está en estado ACTIVE.
     * 
     * @param trig_pin Pin TRIG del sensor.
     * @param echo_pin Pin ECHO del sensor.
     * @param distance_ptr Puntero a la variable donde se almacena la distancia medida.
     * @param sensor_obstacle_flag_ptr Puntero al flag booleano de obstáculo detectado en el sensor.
     * @param global_obstacle_flag_ptr Puntero al flag booleano de obstáculo detectado.
     * @param distance_state_ptr Puntero al estado global del módulo de sensores.
     */
    void check_sensor_obstacle(
        uint8_t trig_pin, uint8_t echo_pin,
        volatile uint8_t* distance_ptr,
        volatile bool* sensor_obstacle_flag_ptr,
        volatile bool* global_obstacle_flag_ptr,
        volatile uint8_t* distance_state_ptr
    );

    /**
     * @brief Tarea RTOS que verifica periódicamente el sensor ultrasónico izquierdo.
     * 
     * Ejecuta `check_single_us_sensor()` con desfase inicial para distribuir la carga del sistema.
     * 
     * @param pvParameters Puntero al contexto global (cast a `GlobalContext*`).
     */
    void Task_CheckLeftObstacle(void* pvParameters);

    /**
     * @brief Tarea RTOS que verifica periódicamente el sensor ultrasónico central.
     * 
     * Ejecuta `check_single_us_sensor()` con desfase inicial para distribuir la carga del sistema.
     * 
     * @param pvParameters Puntero al contexto global (cast a `GlobalContext*`).
     */
    void Task_CheckMidObstacle(void* pvParameters);

    /**
     * @brief Tarea RTOS que verifica periódicamente el sensor ultrasónico derecho.
     * 
     * Ejecuta `check_single_us_sensor()` con desfase inicial para distribuir la carga del sistema.
     * 
     * @param pvParameters Puntero al contexto global (cast a `GlobalContext*`).
     */
    void Task_CheckRightObstacle(void* pvParameters);

}

#endif // DISTANCE_SENSORS_H
