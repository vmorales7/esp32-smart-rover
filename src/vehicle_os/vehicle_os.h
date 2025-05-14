#ifndef VEHICLE_OS_H
#define VEHICLE_OS_H

#include "project_config.h"

    /**
     * @brief Tarea RTOS que verifica cuando se encuentra un obstáculo.
     * 
     * Ejecuta `check_single_us_sensor()` con desfase inicial para distribuir la carga del sistema.
     * 
     * @param pvParameters Puntero al contexto global (cast a `GlobalContext*`).
     */
    void Task_HandleObstacleEvent(void* pvParameters);

#endif // VEHICLE_OS_H