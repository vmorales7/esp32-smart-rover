#include "vehicle_os/general_config.h"
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
    .rtos_task_ptr = nullptr,
    .evade_ptr     = nullptr
};

// ====================== TAREA: Manejo de eventos por obstáculo ======================
void Task_PrintDistances(void* pvParameters) {
    const TickType_t period = pdMS_TO_TICKS(500);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    GlobalContext* ctx = static_cast<GlobalContext*>(pvParameters);
    volatile SensorsData& sens = *ctx->sensors_ptr;

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        if (sens.us_left_obst || sens.us_mid_obst || sens.us_right_obst) {
            Serial.print("Obstáculo en sensor:");
            if (sens.us_left_obst) Serial.println(" IZQUIERDO");
            else if (sens.us_mid_obst) Serial.println(" MEDIO");
            else if (sens.us_right_obst) Serial.println(" DERECHO");
        }
        Serial.printf("Distancias | Izq: %3d cm | Mid: %3d cm | Der: %3d cm\n",
            sens.us_left_dist, sens.us_mid_dist, sens.us_right_dist);
        Serial.println();

        // Consumir evento
        DistanceSensors::update_global_obstacle_flag(
            sens.us_left_obst,sens.us_mid_obst, sens.us_right_obst, sens.us_obstacle);
    }
}

void Task_ObstacleResponse(void* pvParameters) {
    const TickType_t period = pdMS_TO_TICKS(10);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    GlobalContext* ctx = static_cast<GlobalContext*>(pvParameters);
    volatile SensorsData& sens = *ctx->sensors_ptr;

    static bool last_obstacle_state = false; // Estado anterior de la bandera

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        
        // Flanco de subida: se detectó obstáculo (pasa de false a true)
        if (!last_obstacle_state && sens.us_obstacle) {
            Serial.println("Obstáculo DETECTADO. Procesando evento...");
            Serial.printf("Distancias | Izq: %3d cm | Mid: %3d cm | Der: %3d cm\n",
                sens.us_left_dist, sens.us_mid_dist, sens.us_right_dist);
            Serial.println();
        }

        // Flanco de bajada: se liberó el obstáculo (pasa de true a false)
        if (last_obstacle_state && !sens.us_obstacle) {
            Serial.println("Obstáculo LIBERADO.");
            Serial.println();
        }

        // Actualiza el estado anterior
        last_obstacle_state = sens.us_obstacle;

        // Consumir evento siempre (esto puede ser fuera del if)
        DistanceSensors::update_global_obstacle_flag(
            sens.us_left_obst, sens.us_mid_obst, sens.us_right_obst, sens.us_obstacle);
    }
}

// ====================== SETUP ======================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println();
    Serial.println("Iniciando main_distance_sensors...");
    Serial.println();
    delay(5000);

    // Inicializar el sistema
    DistanceSensors::init(sens.us_left_dist, sens.us_left_obst, sens.us_mid_dist, sens.us_mid_obst,
        sens.us_right_dist, sens.us_right_obst, sens.us_obstacle, states.distance);    

    // Tarea de lectura de sensor y de respuesta a eventos de obstáculo
    xTaskCreatePinnedToCore(DistanceSensors::Task_CheckObstacle, "US_Check", 2048, &ctx, 1, nullptr, 0);
    // xTaskCreatePinnedToCore(Task_ObstacleResponse,"ObstacleResponse",2048, &ctx, 2, nullptr, 0);
    xTaskCreatePinnedToCore(Task_PrintDistances, "PrintDistances", 2048, &ctx, 1, nullptr, 0);

    // Empezar
    DistanceSensors::set_state(ACTIVE, states.distance, 
        sens.us_left_obst, sens.us_mid_obst, sens.us_right_obst, sens.us_obstacle);
}

// ====================== LOOP VACÍO ======================
void loop() {
    // No se usa. Todo está gestionado por tareas RTOS.
}
