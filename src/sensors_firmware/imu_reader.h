#ifndef IMU_READER_H
#define IMU_READER_H

#include "project_config.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define MPU9250_ADDR 0x68

/* -------------------- definiciones utilizacion IMU (MPU9250) -------------------- */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

/**
 * @brief creacion del objeto para llamarlo y obtener datos.
 */
Adafruit_BNO055 IMU = Adafruit_BNO055(55, 0x28, &Wire);


/* -------------------- Módulo IMUSensor -------------------- */

namespace IMUSensor {

    /**
     * @brief Inicializa los pines de I2C del IMU.
     * 
     * Configura SDA como entrada de datos y SCL clk I2C.
     * 
     * @param imu_state_ptr Puntero al estado global del módulo de sensores.
     */
    void init(volatile uint8_t* imu_state_ptr);

     /**
     * @brief Cambia el estado activo/inactivo del módulo de sensores de distancia.
     * 
     * @param mode_imu Valor deseado del estado (ACTIVE o INACTIVE).
     * @param imu_ptr Puntero al estado global del módulo.
     */
    void set_imu_state(uint8_t imu_mode, volatile uint8_t* imu_state_ptr);

    /**
     * @brief Evalúa las variables de aceleracion y velocidad de la IMU.
     * 
     * Si es el sistema esta en ACTIVE se actualizan los valores.
     * 
     * @param imu_xaccel_ptr Puntero a la variable donde se almacena la aceleracion medida en el eje x.
     * @param imu_yaccel_ptr Puntero a la variable donde se almacena la aceleracion medida en el eje y.
     * @param imu_xspeed_ptr Puntero a la variable donde se almacena la aceleracion medida en el eje x.
     * @param imu_yspeed_ptr Puntero a la variable donde se almacena la aceleracion medida en el eje y.
     * @param imu_state_ptr Puntero al estado global del módulo de sensores.
     */
    void imu_read_accel_speed(
        volatile double* imu_xaccel_ptr,
        volatile double* imu_yaccel_ptr,
        volatile double* imu_xspeed_ptr,
        volatile double* imu_yspeed_ptr,
        volatile uint8_t* imu_state_ptr
    );

    /**
     * @brief Evalúa las variables de velocidad angular y angulo absoluto de la IMU.
     * 
     * Si es el sistema esta en ACTIVE se actualizan los valores.
     * 
     * @param imu_wgyro_ptr Puntero a la variable de velociad angular edida con giroscopio.
     * @param imu_magangle_ptr Puntero a la variable donde se almacena el angulo absoluto con respecto al magentometro.
     * @param imu_state_ptr Puntero al estado global del módulo de sensores.
     */
    void imu_read_angles(
        volatile float* imu_wgyro_ptr,
        volatile float* imu_magangle_ptr,
        volatile uint8_t* imu_state_ptr
    );
    
    /**
     * @brief Tarea de FreeRTOS que ejecuta periódicamente el poleo de datos del sensor.
     * Usa punteros extraídos desde una estructura `GlobalContext`.
     * 
     * @param pvParameters Puntero a un `GlobalContext*` que contiene referencias al estado del sistema.
     */
    void Task_IMUData(void* pvParameters);

}

#endif //IMU_READER_H