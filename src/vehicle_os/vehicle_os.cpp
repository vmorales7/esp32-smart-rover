#include "project_config.h"

void Task_HandleObstacleEvent(void* pvParameters) {
    const TickType_t period = pdMS_TO_TICKS(CHECK_OBSTACLE_FLAG_PERIOD);  // Frecuencia de revisión
    TickType_t xLastWakeTime = xTaskGetTickCount();

    GlobalContext* ctx = static_cast<GlobalContext*>(pvParameters);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);

        if (ctx->distance_ptr->obstacle_detected) {
            // Acción de debug o futura transición de estado
            //Serial.println("Obstáculo detectado. Procesando evento...");

            // (Más adelante: enviar evento a FSM)

            // Consumir el evento
            ctx->distance_ptr->obstacle_detected = false;
        }
    }

}