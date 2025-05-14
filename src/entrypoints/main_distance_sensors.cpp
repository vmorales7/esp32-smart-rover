#include "project_config.h"
#include "sensors_firmware/distance_sensors.h"
#warning "Compilando main_distance_sensors.cpp"

// ====================== VARIABLES GLOBALES ======================
volatile SystemStates states = {0};
volatile DistanceSensorData distances = {0};
GlobalContext ctx = {
    .systems_ptr = &states,
    .kinematic_ptr = nullptr,
    .wheels_ptr = nullptr,
    .distance_ptr = &distances
};

// ====================== TAREA: Manejo de eventos por obstáculo ======================
void Task_ObstacleResponse(void* pvParameters) {
    const TickType_t period = pdMS_TO_TICKS(200);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    GlobalContext* ctx = static_cast<GlobalContext*>(pvParameters);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);

        if (ctx->distance_ptr->obstacle_detected) {
            Serial.println("Obstáculo detectado. Procesando evento...");

            Serial.print("  Distancia izquierda: ");
            Serial.print(ctx->distance_ptr->us_left_distance);
            Serial.println(" cm");

            Serial.print("  Distancia frontal: ");
            Serial.print(ctx->distance_ptr->us_mid_distance);
            Serial.println(" cm");

            Serial.print("  Distancia derecha: ");
            Serial.print(ctx->distance_ptr->us_right_distance);
            Serial.println(" cm");
            Serial.println();

            // Consumir evento
            ctx->distance_ptr->obstacle_detected = false;
        }
    }
}

// ====================== SETUP ======================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Iniciando main_distance_sensors...");
    delay(1000);

    // Inicializar el sistema
    DistanceSensors::init_system(states.distance, distances);
    DistanceSensors::set_state(ACTIVE, states.distance);

    // Crear tareas para cada sensor ultrasónico
    xTaskCreatePinnedToCore(DistanceSensors::Task_CheckLeftObstacle, "US_Left", 2048, &ctx, 2, nullptr, 0);
    xTaskCreatePinnedToCore(DistanceSensors::Task_CheckMidObstacle, "US_Mid", 2048, &ctx, 2, nullptr, 0);
    xTaskCreatePinnedToCore(DistanceSensors::Task_CheckRightObstacle, "US_Right", 2048, &ctx, 2, nullptr, 0);

    // Crear tarea de respuesta a eventos de obstáculo -> Prioridad mayor
    xTaskCreatePinnedToCore(DistanceSensors::Task_UpdateObstacleFlag,"ObstacleCheck",2048, &ctx, 1, nullptr, 0);
    xTaskCreatePinnedToCore(Task_ObstacleResponse,"ObstacleResponse",2048, &ctx, 2, nullptr, 0);
}

// ====================== LOOP VACÍO ======================
void loop() {
    // No se usa. Todo está gestionado por tareas RTOS.
}
