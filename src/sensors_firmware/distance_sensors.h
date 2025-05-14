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
    void init_sensor(const uint8_t trig_pin, const uint8_t echo_pin);

    /**
     * @brief Realiza una lectura puntual de distancia desde un sensor ultrasónico.
     * 
     * Emite un pulso TRIG y mide el tiempo hasta la recepción del pulso ECHO.
     * 
     * @param trig_pin Pin TRIG del sensor.
     * @param echo_pin Pin ECHO del sensor.
     * @return Distancia estimada en cm. Retorna US_MAX_DISTANCE_CM en caso de timeout.
     */
    uint8_t read_distance(const uint8_t trig_pin, const uint8_t echo_pin);

    /**
     * @brief Reinicia completamente la estructura DistanceSensorData.
     * 
     * Establece todas las distancias en US_MAX_DISTANCE_CM y todas las banderas
     * de detección de obstáculos en false. Se utiliza durante la inicialización
     * o al reiniciar el módulo de sensores.
     * 
     * @param distance_data Variable global con la estructura DistanceSensorData que se desea reiniciar.
     * @param distance_state Variable global con el estado de operación del sistema
     */
    void reset_system(volatile DistanceSensorData& distance_data, volatile uint8_t& distance_state);

    /**
     * @brief Inicializa los sensores de distancia y reinicia la estructura de datos asociada.
     * 
     * Configura los pines TRIG y ECHO de los sensores ultrasónicos (izquierdo, medio y derecho).
     * Además, deja en estado INACTIVE el módulo y reinicia la estructura `DistanceSensorData`:
     * - Todas las distancias se establecen en `US_MAX_DISTANCE_CM`.
     * - Todas las banderas de obstáculos se ponen en `false`.
     * 
     * @param distance_state Variable global con estado global del módulo de sensores (ACTIVE o INACTIVE).
     * @param distance_data Variable global con la estructura `DistanceSensorData` que será reiniciada.
     */
    void init_system(
        volatile uint8_t& distance_state,
        volatile DistanceSensorData& distance_data
    );

    /**
     * @brief Cambia el estado activo/inactivo del módulo de sensores de distancia.
     * 
     * @param new_mode Valor deseado del estado (ACTIVE o INACTIVE).
     * @param distance_state Variable global con estado global del módulo.
     */
    void set_state(const uint8_t new_mode, volatile uint8_t& distance_state);

    /**
     * @brief Evalúa un único sensor ultrasónico y actualiza su distancia y estado de obstáculo.
     * 
     * Realiza doble lectura para mitigar errores y rebotes. Se actualizan tanto la distancia
     * como el flag de obstáculo si el sistema está en estado ACTIVE.
     * 
     * @param trig_pin Pin TRIG del sensor.
     * @param echo_pin Pin ECHO del sensor.
     * @param distance Variable global con la variable donde se almacena la distancia medida.
     * @param sensor_obstacle_flag Variable global con flag booleano de obstáculo detectado en el sensor.
     * @param global_obstacle_flag Variable global con flag booleano de obstáculo detectado.
     * @param distance_state Variable global con estado global del módulo de sensores.
     * @return Booleano que es true si hay un obstáculo
     */
    bool check_sensor_obstacle(
        const uint8_t trig_pin, const uint8_t echo_pin,
        volatile uint8_t& distance,
        volatile bool& sensor_obstacle_flag,
        volatile bool& global_obstacle_flag,
        volatile uint8_t& distance_state
    );

    /**
     * @brief Actualiza el flag global de detección de obstáculos basado en los sensores individuales.
     *
     * Esta función verifica si alguno de los sensores (izquierdo, central o derecho)
     * ha detectado un obstáculo, y actualiza el flag `obstacle_detected` en la estructura
     * `DistanceSensorData`. Retorna el nuevo valor del flag global.
     *
     * @param data Referencia a la estructura `DistanceSensorData` que contiene los flags individuales.
     * @return true si al menos un sensor detecta obstáculo, false en caso contrario.
     */
    bool update_global_obstacle_flag(volatile DistanceSensorData& data);

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

    void Task_UpdateObstacleFlag(void* pvParameters);

}

#endif // DISTANCE_SENSORS_H
