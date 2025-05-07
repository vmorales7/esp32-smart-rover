#ifndef DISTANCE_SENSORS_H
#define DISTANCE_SENSORS_H

#include "project_config.h"

/* -------------------- Parámetros del sensor ultrasónico (HC-SR04) -------------------- */

/**
 * @brief Velocidad del sonido dividida por 2 en cm/µs.
 * 
 * Se usa para convertir el tiempo de ida y vuelta medido por el sensor
 * en una distancia unidireccional (desde el sensor al obstáculo).
 * 
 * 343 m/s → 0.0343 cm/µs → dividido por 2 = 0.01715 cm/µs
 */
constexpr float US_CM_PER_US = 0.01715f;

/**
 * @brief Distancia máxima considerada válida para el sensor [cm].
 * 
 * Si la distancia estimada supera este valor, se considera inválida
 * y se retorna US_MAX_DISTANCE_CM.
 */
constexpr uint8_t US_MAX_DISTANCE_CM = 100;

/**
 * @brief Tiempo máximo de espera del pulso de eco [µs].
 * 
 * Limita el alcance máximo del sensor para evitar bloqueos largos.
 */
constexpr uint32_t US_PULSE_TIMEOUT_US = US_MAX_DISTANCE_CM / US_CM_PER_US;

/**
 * @brief Valor especial que representa error o lectura no válida del sensor US.
 * 
 * Este valor puede usarse si decides diferenciar una lectura fallida.
 */
constexpr uint8_t US_ERROR_VALUE = 0;

/**
 * @brief Umbral de distancia para definir presencia de obstáculo [cm].
 * 
 * Si la lectura es menor a este valor, se asume que hay un obstáculo presente.
 */
constexpr uint8_t OBSTACLE_THRESHOLD_CM = 30;


/* -------------------- Parámetros del sensor infrarrojo (IR digital) -------------------- */

/**
 * @brief Tiempo mínimo de estabilidad de la señal IR para validar un flanco [µs].
 *
 * Este parámetro define cuánto debe permanecer estable la señal del sensor infrarrojo (IR)
 * para considerar un cambio como válido. Protege contra rebotes o ruido digital.
 */
constexpr uint16_t IR_DEBOUNCE_TIME_US = 1000U;


/* -------------------- Funciones del módulo DistanceSensors -------------------- */

/**
 * @brief Contiene funciones para inicializar y leer sensores ultrasónicos e infrarrojos.
 */
namespace DistanceSensors {

    /**
     * @brief Inicializa los pines asociados a los sensores de distancia y deja el estado como INACTIVE.
     * 
     * @param distance_state_ptr Puntero al estado global del módulo de sensores de distancia.
     */
    void init(volatile uint8_t* distance_state_ptr);

    /**
     * @brief Cambia el estado activo/inactivo del módulo de sensores de distancia.
     * 
     * @param mode Valor deseado del estado (ACTIVE o INACTIVE).
     * @param distance_state_ptr Puntero al estado global del módulo.
     */
    void set_state(uint8_t mode, volatile uint8_t* distance_state_ptr);

    /**
     * @brief Realiza una medición puntual de distancia utilizando un sensor ultrasónico.
     * 
     * @param trig_pin Pin de disparo (TRIG) del sensor ultrasónico.
     * @param echo_pin Pin de escucha (ECHO) del sensor ultrasónico.
     * @return Distancia medida en centímetros (truncada a uint8_t). Si el pulso falla, retorna US_MAX_DISTANCE_CM.
     */
    uint8_t us_read_distance(uint8_t trig_pin, uint8_t echo_pin);

    /**
     * @brief Verifica si hay obstáculo al frente utilizando ambos sensores ultrasónicos (izq. y der.).
     * Incluye doble lectura tipo debounce para reducir falsos positivos.
     * 
     * @param obstacle_detected_ptr Puntero al flag general de detección de obstáculos.
     * @param us_left_distance_ptr Puntero a la distancia medida por el sensor izquierdo [cm].
     * @param us_right_distance_ptr Puntero a la distancia medida por el sensor derecho [cm].
     * @param distance_state_ptr Puntero al estado global del módulo (verifica si está activo).
     */
    void us_check_obstacle(
        volatile bool* obstacle_detected_ptr,
        volatile uint8_t* us_left_distance_ptr,
        volatile uint8_t* us_right_distance_ptr,
        volatile uint8_t* distance_state_ptr
    );

    /**
     * @brief Realiza lectura segura con debounce del estado de un sensor IR digital.
     * 
     * @param ir_pin Pin conectado al sensor IR.
     * @return `true` si hay obstáculo (LOW estable), `false` en caso contrario o si hubo rebote.
     */
    bool ir_check_obstacle(uint8_t ir_pin);

    /**
     * @brief Lee ambos sensores IR y actualiza los valores de detección de obstáculos en cada lado.
     * 
     * @param ir_left_obstacle_ptr Puntero al flag de obstáculo a la izquierda.
     * @param ir_right_obstacle_ptr Puntero al flag de obstáculo a la derecha.
     * @param distance_state_ptr Puntero al estado global del módulo (verifica si está activo).
     */
    void ir_read_sensors(
        volatile bool* ir_left_obstacle_ptr,
        volatile bool* ir_right_obstacle_ptr,
        volatile uint8_t* distance_state_ptr
    );

    /**
     * @brief Tarea FreeRTOS que actualiza periódicamente los sensores ultrasónicos frontales.
     * Llama a `us_check_obstacle()` cada cierto intervalo de tiempo.
     * 
     * @param pvParameters Puntero al `GlobalContext` del sistema (cast dentro de la función).
     */
    void Task_FrontObstacleDetect(void* pvParameters);

    /**
     * @brief Tarea FreeRTOS que actualiza periódicamente el estado de los sensores IR laterales.
     * Llama a `ir_read_sensors()` para mantener actualizadas las banderas de obstáculo lateral.
     * 
     * @param pvParameters Puntero al `GlobalContext` del sistema (cast dentro de la función).
     */
    void Task_LateralObstacleDetect(void* pvParameters);

}

#endif // DISTANCE_SENSORS_H
