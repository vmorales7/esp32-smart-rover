#include "project_config.h"
#include "sensors_firmware/distance_sensors.h"
#warning "Compilando main_distance_sensors_basic.cpp"

// ====================== VARIABLES GLOBALES ======================
volatile SystemStates system_states = {0};
volatile DistanceSensorData distance_data = {0};
GlobalContext ctx = {
    .systems_ptr = &system_states,
    .kinematic_ptr = nullptr,
    .wheels_ptr = nullptr,
    .distance_ptr = &distance_data
};

// ====================== TAREA: Manejo de eventos por obstáculo ======================
void Task_ObstacleResponse(void* pvParameters) {
    const TickType_t period = pdMS_TO_TICKS(200);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    GlobalContext* ctx = static_cast<GlobalContext*>(pvParameters);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);

        if (ctx->distance_ptr->obstacle_detected) {
            Serial.println("⚠️ Obstáculo detectado. Procesando evento...");

            // Lecturas adicionales en tiempo real
            uint8_t dL = DistanceSensors::read_distance(US_LEFT_TRIG_PIN, US_LEFT_ECHO_PIN);
            uint8_t dM = DistanceSensors::read_distance(US_MID_TRIG_PIN, US_MID_ECHO_PIN);
            uint8_t dR = DistanceSensors::read_distance(US_RIGHT_TRIG_PIN, US_RIGHT_ECHO_PIN);

            Serial.print("  Distancia izquierda: ");
            Serial.print(dL);
            Serial.println(" cm");

            Serial.print("  Distancia frontal: ");
            Serial.print(dM);
            Serial.println(" cm");

            Serial.print("  Distancia derecha: ");
            Serial.print(dR);
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
    Serial.println("Iniciando main_distance_sensors_basic...");
    delay(5000);

    DistanceSensors::init(&system_states.distance);
    DistanceSensors::set_state(ACTIVE, &system_states.distance);

    // Crear tareas para cada sensor ultrasónico
    xTaskCreatePinnedToCore(
        DistanceSensors::Task_CheckLeftObstacle,
        "US_Left",
        2048, &ctx, 1, nullptr, APP_CPU_NUM
    );
    xTaskCreatePinnedToCore(
        DistanceSensors::Task_CheckMidObstacle,
        "US_Mid",
        2048, &ctx, 1, nullptr, APP_CPU_NUM
    );
    xTaskCreatePinnedToCore(
        DistanceSensors::Task_CheckRightObstacle,
        "US_Right",
        2048, &ctx, 1, nullptr, APP_CPU_NUM
    );

    // Crear tarea de respuesta a eventos de obstáculo
    xTaskCreatePinnedToCore(
        Task_ObstacleResponse,
        "ObstacleResponse",
        2048, &ctx, 2, nullptr, APP_CPU_NUM // Prioridad mayor
    );
}

// ====================== LOOP VACÍO ======================
void loop() {
    // No se usa. Todo está gestionado por tareas RTOS.
}
