#ifndef ENCODER_READER_H
#define ENCODER_READER_H

#include "project_config.h"
#include <ESP32Encoder.h>


/* ---------------- Constantes del sistema ------------------*/

// Para ajustar si los cables se conectaron al revés
constexpr bool INVERT_ENCODER_LEFT = true;
constexpr bool INVERT_ENCODER_RIGHT = false;

// Para aplicar el filtro EMA a la velocidad leída
constexpr bool USE_VELOCITY_FILTER = true;
constexpr float EMA_ALPHA = 0.5; // 0.5 = promediar ~3 valores; y al bajarlo se promedian mas muestras


/* ---------------- Funciones del sistema ------------------*/

/**
 * @brief Módulo encargado de la lectura de los encoders incrementales.
 * Utiliza el periférico PCNT de la ESP32 para contar los pulsos generados por encoders cuadratura.
 * Calcula los pasos acumulados y la velocidad angular para cada rueda.
 */
namespace EncoderReader {

    /**
     * @brief Inicializa el módulo de encoders. Configura los pines, pausa los contadores
     * y deja los valores iniciales en cero.
     * 
     * @param encoder_state_ptr Puntero al estado del encoder (activo/inactivo).
     * @param steps_left_ptr Puntero al acumulador de pasos de la rueda izquierda.
     * @param steps_right_ptr Puntero al acumulador de pasos de la rueda derecha.
     * @param wL_measured_ptr Puntero a la velocidad angular izquierda medida [rad/s].
     * @param wR_measured_ptr Puntero a la velocidad angular derecha medida [rad/s].
     */
    void init(
        volatile uint8_t* encoder_state_ptr,
        volatile int64_t* steps_left_ptr, volatile int64_t* steps_right_ptr,
        volatile float* wL_measured_ptr, volatile float* wR_measured_ptr
    );

    /**
     * @brief Pausa el conteo de los encoders, actualiza velocidades, y deja las velocidades en cero.
     * El acumulador de pasos se conserva (solo pose_estimator lo puede limpiar). Actualiza el estado como INACTIVE.
     * 
     * @param encoder_state_ptr Puntero al estado del encoder (activo/inactivo).
     * @param steps_left_ptr Puntero al acumulador de pasos de la rueda izquierda.
     * @param steps_right_ptr Puntero al acumulador de pasos de la rueda derecha.
     * @param wL_measured_ptr Puntero a la velocidad angular izquierda medida [rad/s].
     * @param wR_measured_ptr Puntero a la velocidad angular derecha medida [rad/s].
     */
    void pause(
        volatile uint8_t* encoder_state_ptr,
        volatile int64_t* steps_left_ptr, volatile int64_t* steps_right_ptr,
        volatile float* wL_measured_ptr, volatile float* wR_measured_ptr
    );

    /**
     * @brief Reanuda el conteo de los encoders y actualiza el estado como ACTIVE.
     * 
     * @param encoder_state_ptr Puntero al estado del encoder.
     */
    void resume(volatile uint8_t* encoder_state_ptr);

    /**
     * @brief Actualiza el número de pasos acumulados y la velocidad angular desde la última lectura.
     * Si el encoder está inactivo, no realiza ninguna operación.
     * 
     * @param encoder_state_ptr Puntero al estado del encoder (activo/inactivo).
     * @param steps_left_ptr Puntero al acumulador de pasos de la rueda izquierda.
     * @param steps_right_ptr Puntero al acumulador de pasos de la rueda derecha.
     * @param wL_measured_ptr Puntero a la velocidad angular izquierda medida [rad/s].
     * @param wR_measured_ptr Puntero a la velocidad angular derecha medida [rad/s].
     */
    void update_encoder_data(
        volatile uint8_t* encoder_state_ptr,
        volatile int64_t* steps_left_ptr, volatile int64_t* steps_right_ptr,
        volatile float* wL_measured_ptr, volatile float* wR_measured_ptr
    );

    /**
     * @brief Tarea FreeRTOS que se ejecuta periódicamente para actualizar la información
     * de los encoders. Accede a los datos globales mediante GlobalContext.
     * 
     * @param pvParameters Puntero a un GlobalContext con los punteros a los datos relevantes.
     */
    void Task_EncoderUpdate(void* pvParameters);

}

#endif // ENCODER_READER_H

