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
 * @brief Tiempo máximo de espera del pulso de eco [µs].
 * 
 * Limita el alcance máximo del sensor para evitar bloqueos largos.
 * Se recomienda calcular como: `MAX_DISTANCE_CM / US_CM_PER_US`
 */
constexpr uint32_t US_PULSE_TIMEOUT_US = 6000;

/**
 * @brief Distancia máxima considerada válida para el sensor [cm].
 * 
 * Si la distancia estimada supera este valor, se considera inválida
 * y se retorna US_ERROR_VALUE.
 */
constexpr uint8_t US_MAX_DISTANCE_CM = 100;

/**
 * @brief Valor especial que representa error o lectura no válida del sensor US.
 * 
 * Este valor será retornado por `us_read_distance()` si hay timeout
 * o si la distancia medida supera el máximo permitido.
 */
constexpr uint8_t US_ERROR_VALUE = 0;

/* -------------------- Parámetros del sensor infrarrojo (IR digital) -------------------- */

/**
 * @brief Tiempo mínimo de estabilidad de la señal IR para validar un flanco [µs].
 *
 * Este parámetro define el tiempo que la señal del sensor infrarrojo (IR) debe
 * permanecer constante para que se considere una transición válida (debounce).
 * Si la señal cambia antes de completar este tiempo, se considera ruido o rebote.
 *
 * Se utiliza dentro de la función `update_ir_state()` para confirmar cambios
 * estables de nivel lógico en la entrada digital del sensor IR.
 */
constexpr uint16_t IR_DEBOUNCE_TIME_US = 1000U;


/* -------------------- Funciones de sensores de distancia -------------------- */

namespace DistanceSensors {

    /**
     * @brief Inicializa los pines y estado de los sensores IR y US.
     */
    void init();

    /**
     * @brief Realiza una lectura puntual de un sensor ultrasónico.
     *
     * @param trig_pin Pin TRIG
     * @param echo_pin Pin ECHO
     * @return Distancia [cm] o US_ERROR_VALUE si falla
     */
    uint8_t us_read_distance(uint8_t trig_pin, uint8_t echo_pin);

    
    /**
     * @brief Actualiza las distancias medidas por sensores ultrasónicos.
     *
     * @param data Puntero a estructura donde se guardan los resultados.
     */
    void update_ultrasonic_distances(volatile DistanceSensorData* data);
    
    /**
     * @brief Actualiza el estado del sensor infrarrojo con lógica de debounce.
     *
     * @param data Puntero a estructura de sensores a actualizar.
     */
    void update_ir_state(volatile DistanceSensorData* data);

    /**
     * @brief Limpia la detección actual de obstáculo por IR (fuerza el estado a "libre").
     *
     * @param data Puntero a la estructura de sensores.
     */
    void clear_obstacle_detection(volatile DistanceSensorData* data);

    /**
     * @brief Tarea FreeRTOS que actualiza periódicamente los sensores.
     *
     * @param pvParameters Debe ser un puntero a DistanceSensorData*
     */
    void Task_DistanceSensors(void* pvParameters);

}

#endif // DISTANCE_SENSORS_H