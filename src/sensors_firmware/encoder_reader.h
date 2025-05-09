#ifndef ENCODER_READER_H
#define ENCODER_READER_H

#include "project_config.h"
#include <ESP32Encoder.h>

/* ---------------- Constantes del sistema ------------------*/

// Para ajustar si los cables del encoder se conectaron al revés
constexpr bool INVERT_ENCODER_LEFT = true;
constexpr bool INVERT_ENCODER_RIGHT = false;

// Para aplicar un filtro EMA (exponential moving average) a la velocidad
constexpr bool USE_VELOCITY_FILTER = true;
constexpr float EMA_ALPHA = 0.5f; // 0.5 ≈ promedio de 3 lecturas; menor α promedia más.

/* ---------------- Funciones del sistema ------------------*/

/**
 * @brief Módulo encargado de la lectura de los encoders incrementales.
 * Utiliza el periférico PCNT de la ESP32 para contar pulsos cuadratura.
 * Calcula pasos acumulados y velocidad angular para cada rueda.
 */
namespace EncoderReader {

    /**
     * @brief Inicializa el módulo de encoders. Configura los pines, pausa los contadores
     * y deja los valores iniciales de pasos y velocidades en cero.
     * 
     * @param wheels_data_ptr Puntero a la estructura con pasos y velocidades de rueda.
     * @param encoder_state_ptr Puntero al estado del módulo de encoder (ACTIVE / INACTIVE).
     */
    void init(
        volatile WheelsData* wheels_data_ptr,
        volatile uint8_t* encoder_state_ptr
    );

    /**
     * @brief Pausa el conteo de los encoders, actualiza velocidades, y las deja en cero.
     * Los acumuladores de pasos se conservan (solo `pose_estimator` los reinicia).
     * 
     * @param wheels_data_ptr Puntero a la estructura con pasos y velocidades de rueda.
     * @param encoder_state_ptr Puntero al estado del módulo de encoder.
     */
    void pause(
        volatile WheelsData* wheels_data_ptr,
        volatile uint8_t* encoder_state_ptr
    );

    /**
     * @brief Reanuda el conteo de los encoders y cambia el estado a ACTIVE si no lo estaba ya.
     * 
     * @param encoder_state_ptr Puntero al estado del módulo de encoder.
     */
    void resume(volatile uint8_t* encoder_state_ptr);

    /**
     * @brief Actualiza los pasos acumulados y la velocidad angular de cada rueda.
     * Si el módulo está inactivo, no realiza ninguna operación.
     * 
     * @param wheels_data_ptr Puntero a la estructura con pasos y velocidades de rueda.
     * @param encoder_state_ptr Puntero al estado del módulo de encoder.
     */
    void update_encoder_data(
        volatile WheelsData* wheels_data_ptr,
        volatile uint8_t* encoder_state_ptr
    );

    /**
     * @brief Tarea FreeRTOS que ejecuta periódicamente la actualización del encoder.
     * Accede a `WheelsData` y estado a través del `GlobalContext`.
     * 
     * @param pvParameters Puntero a una estructura `GlobalContext` con punteros relevantes.
     */
    void Task_EncoderUpdate(void* pvParameters);
}

#endif // ENCODER_READER_H
