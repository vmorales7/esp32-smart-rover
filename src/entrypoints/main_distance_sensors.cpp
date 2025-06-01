#include "project_config.h"
#include "sensors_firmware/distance_sensors.h"
#warning "Compilando main_distance_sensors.cpp"

// ====================== VARIABLES GLOBALES ======================
volatile SystemStates states;
volatile SensorsData sens;

GlobalContext ctx = {
    .systems_ptr   = &states,
    .sensors_ptr   = &sens,
    .control_ptr   = nullptr,
    .os_ptr        = nullptr,
    .rtos_task_ptr = nullptr
};

// ====================== TAREA: Manejo de eventos por obstáculo ======================
void Task_ObstacleResponse(void* pvParameters) {
    const TickType_t period = pdMS_TO_TICKS(200);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    GlobalContext* ctx = static_cast<GlobalContext*>(pvParameters);
    volatile SensorsData& sens = *ctx->sensors_ptr;

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        if (sens.us_obstacle) {
            Serial.println("Obstáculo detectado. Procesando evento...");

            Serial.print("  Distancia izquierda: ");
            Serial.print(sens.us_left_dist);
            Serial.println(" cm");

            Serial.print("  Distancia frontal: ");
            Serial.print(sens.us_mid_dist);
            Serial.println(" cm");

            Serial.print("  Distancia derecha: ");
            Serial.print(sens.us_right_dist);
            Serial.println(" cm");
            Serial.println();

            // Consumir evento
            DistanceSensors::update_global_obstacle_flag(
                sens.us_left_obst,sens.us_mid_obst, sens.us_right_obst, sens.us_obstacle);
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
    DistanceSensors::init(sens.us_left_dist, sens.us_left_obst, sens.us_mid_dist, sens.us_mid_obst,
        sens.us_right_dist, sens.us_right_obst, sens.us_obstacle, states.distance);    
    DistanceSensors::set_state(ACTIVE, states.distance, sens.us_left_dist, sens.us_left_obst,
        sens.us_mid_dist, sens.us_mid_obst, sens.us_right_dist, sens.us_right_obst, sens.us_obstacle);

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
