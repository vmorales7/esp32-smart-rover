#include "project_config.h"
#include "sensors_firmware/distance_sensors.h"
#warning "Compilando main_distance_sensors.cpp"

// ====================== VARIABLES GLOBALES ======================
volatile SystemStates states;
volatile DistanceSensorData distances = {0};
GlobalContext ctx = {
    .systems_ptr     = &states,
    .os_ptr          = nullptr,       
    .kinematic_ptr   = nullptr,
    .wheels_ptr      = nullptr,
    .imu_ptr         = nullptr,      
    .distance_ptr    = &distances
};

// ====================== TAREA: Manejo de eventos por obstáculo ======================
void Task_ObstacleResponse(void* pvParameters) {
    const TickType_t period = pdMS_TO_TICKS(200);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    GlobalContext* ctx = static_cast<GlobalContext*>(pvParameters);
    auto& d = *ctx->distance_ptr;  // Referencia directa

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);

        if (d.obstacle_detected) {
            Serial.println("Obstáculo detectado. Procesando evento...");

            Serial.print("  Distancia izquierda: ");
            Serial.print(d.left_dist);
            Serial.println(" cm");

            Serial.print("  Distancia frontal: ");
            Serial.print(d.mid_dist);
            Serial.println(" cm");

            Serial.print("  Distancia derecha: ");
            Serial.print(d.right_dist);
            Serial.println(" cm");
            Serial.println();

            // Consumir evento
            DistanceSensors::update_global_obstacle_flag(
                d.left_obst, d.mid_obst, d.right_obst, d.obstacle_detected);
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
    DistanceSensors::init(
        distances.left_dist, distances.left_obst,
        distances.mid_dist, distances.mid_obst,
        distances.right_dist, distances.right_obst,
        distances.obstacle_detected, states.distance
    );    
    DistanceSensors::set_state(
        ACTIVE, states.distance,
        distances.left_dist, distances.left_obst,
        distances.mid_dist, distances.mid_obst,
        distances.right_dist, distances.right_obst,
        distances.obstacle_detected
    );

    // Crear tareas para cada sensor ultrasónico
    xTaskCreatePinnedToCore(DistanceSensors::Task_CheckLeftObstacle, "US_Left", 2048, &ctx, 2, nullptr, 0);
    xTaskCreatePinnedToCore(DistanceSensors::Task_CheckMidObstacle, "US_Mid", 2048, &ctx, 2, nullptr, 0);
    xTaskCreatePinnedToCore(DistanceSensors::Task_CheckRightObstacle, "US_Right", 2048, &ctx, 2, nullptr, 0);

    // Crear tarea de respuesta a eventos de obstáculo -> Prioridad mayor
    xTaskCreatePinnedToCore(Task_ObstacleResponse,"ObstacleResponse",2048, &ctx, 2, nullptr, 0);
}

// ====================== LOOP VACÍO ======================
void loop() {
    // No se usa. Todo está gestionado por tareas RTOS.
}
