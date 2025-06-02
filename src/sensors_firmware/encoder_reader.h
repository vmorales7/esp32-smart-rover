#ifndef ENCODER_READER_H
#define ENCODER_READER_H

#include "vehicle_os/general_config.h"
#include <ESP32Encoder.h>

/* ---------------- Constantes del sistema ------------------*/

// Para ajustar si los cables del encoder se conectaron al revés
constexpr bool INVERT_ENCODER_LEFT = true;
constexpr bool INVERT_ENCODER_RIGHT = false;

// Para aplicar un filtro EMA (exponential moving average) a la velocidad
constexpr bool USE_VELOCITY_FILTER = true;
constexpr float EMA_ALPHA = 0.35f; // 0.5 ≈ promedio de 3 lecturas; menor α promedia más.

/* ---------------- Funciones del sistema ------------------*/

/**
 * @brief Módulo encargado de la lectura de los encoders incrementales.
 * Utiliza el periférico PCNT de la ESP32 para contar pulsos cuadratura.
 * Calcula pasos acumulados y velocidad angular para cada rueda.
 */
namespace EncoderReader {

    /**
     * @brief Inicializa el módulo de encoders.
     *
     * Configura los pines de los encoders, detiene los contadores,
     * y deja las variables de pasos y velocidades en cero.
     * El estado del módulo se establece como INACTIVE.
     *
     * @param steps_L Referencia a la variable que almacena los pasos acumulados de la rueda izquierda.
     * @param steps_R Referencia a la variable que almacena los pasos acumulados de la rueda derecha.
     * @param w_L Referencia a la velocidad angular medida de la rueda izquierda [rad/s].
     * @param w_R Referencia a la velocidad angular medida de la rueda derecha [rad/s].
     * @param encoder_state Estado actual del módulo de encoders (se establece como INACTIVE).
     */
    void init(
        volatile int64_t& steps_L, volatile int64_t& steps_R, 
        volatile float& w_L, volatile float& w_R, 
        volatile uint8_t& encoder_state
    );

    /**
     * @brief Actualiza los pasos acumulados y las velocidades angulares de las ruedas.
     *
     * Esta función calcula el número de pasos desde la última lectura y actualiza
     * las velocidades angulares de cada rueda. No realiza ninguna operación si el módulo está inactivo.
     *
     * @param steps_L Referencia a la variable con los pasos acumulados de la rueda izquierda.
     * @param steps_R Referencia a la variable con los pasos acumulados de la rueda derecha.
     * @param w_L Referencia a la velocidad angular de la rueda izquierda [rad/s].
     * @param w_R Referencia a la velocidad angular de la rueda derecha [rad/s].
     * @param encoder_state Estado actual del módulo de encoders (debe estar en ACTIVE para ejecutar la actualización).
     */
    void update_encoder_data(
        volatile int64_t& steps_L, volatile int64_t& steps_R, 
        volatile float& w_L, volatile float& w_R, 
        const uint8_t encoder_state
    );

    /**
     * @brief Pausa el conteo de los encoders y detiene la actualización de velocidades.
     *
     * Esta función detiene los contadores físicos de los encoders, actualiza una última vez
     * la velocidad angular medida, y luego pone ambas velocidades en cero. Los pasos acumulados
     * se conservan (no se reinician aquí).
     *
     * @param steps_L Referencia a la variable con los pasos acumulados de la rueda izquierda.
     * @param steps_R Referencia a la variable con los pasos acumulados de la rueda derecha.
     * @param w_L Referencia a la velocidad angular de la rueda izquierda [rad/s].
     * @param w_R Referencia a la velocidad angular de la rueda derecha [rad/s].
     * @param encoder_state Estado actual del módulo de encoders (se establece como INACTIVE al pausar).
     */
    void pause(
        volatile int64_t& steps_L, volatile int64_t& steps_R, 
        volatile float& w_L, volatile float& w_R,
        volatile uint8_t& encoder_state
    );

    /**
     * @brief Reanuda el conteo de los encoders y cambia el estado a ACTIVE si no lo estaba ya.
     * 
     * @param encoder_state Variable global con estado del módulo de encoder.
     */
    void resume(volatile uint8_t& encoder_state);

    /**
     * @brief Tarea FreeRTOS que ejecuta periódicamente la actualización del encoder.
     * Accede a `WheelsData` y estado a través del `GlobalContext`.
     * 
     * @param pvParameters Puntero a una estructura `GlobalContext` con punteros relevantes.
     */
    void Task_EncoderUpdate(void* pvParameters);
}

#endif // ENCODER_READER_H
