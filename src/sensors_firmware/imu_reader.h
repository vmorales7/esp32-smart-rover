#ifndef IMU_READER_H
#define IMU_READER_H

#include "project_config.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define MPU9250_ADDR 0x68

#define BNO055_SAMPLERATE_DELAY_MS (20)

/**
 * @brief creacion del objeto para llamarlo y obtener datos.
 */


/* -------------------- Módulo IMUSensor -------------------- */

namespace IMUSensor {

     /**
     * @brief resetea las variables
     * 
     * Configura SDA como entrada de datos y SCL clk I2C.
     * 
     * @para variable estado global del módulo de sensores.
     * 
     * @param imu_yaccel Puntero a la variable donde se almacena la aceleracion medida en el eje y.
     * @param imu_wgyro Puntero a la variable donde se almacena la aceleracion medida en el eje x.
     * @param imu_orientation Puntero a la variable donde se almacena la aceleracion medida en el eje y.
     */
    void reset(
        volatile float& imu_yaccel_ptr,
        volatile float& imu_wgyro,
        volatile float& imu_orientation);

 
    /**
     * @brief Inicializa los pines de I2C del IMU.
     * 
     * Configura SDA como entrada de datos y SCL clk I2C.
     * 
     * @param imu_state variable estado global del módulo de sensores.
     */
    void init(
        volatile float& imu_yaccel_ptr,
        volatile float& imu_wgyro,
        volatile float& imu_orientation,
        volatile uint8_t imu_state);

     /**
     * @brief Cambia el estado activo/inactivo del módulo de sensores de distancia.
     * 
     * @param mode_imu Valor deseado del estado (ACTIVE o INACTIVE).
     * @param imu_ptr Puntero al estado global del módulo.
     */
    void set_state(uint8_t imu_mode, volatile uint8_t imu_state);

    /**
     * @brief Evalúa las variables de aceleracion y velocidad de la IMU.
     * 
     * Si es el sistema esta en ACTIVE se actualizan los valores.
     * 
     * @param imu_yaccel Puntero a la variable donde se almacena la aceleracion medida en el eje y.
     * @param imu_wgyro Puntero a la variable donde se almacena la aceleracion medida en el eje x.
     * @param imu_orientation Puntero a la variable donde se almacena la aceleracion medida en el eje y.
     * @param imu_state Puntero al estado global del módulo de sensores.
     */
    void imu_read_data(
        volatile float& imu_yaccel_ptr,
        volatile float& imu_wgyro,
        volatile float& imu_orientation,
        volatile uint8_t imu_state
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