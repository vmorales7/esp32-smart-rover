#include "project_config.h"
#include "sensors_firmware/distance_sensors.h"
#warning "Compilando main_distance_sensors_basic.cpp"

// ====================== VARIABLES GLOBALES ======================
volatile SystemStates system_states = {0};
volatile DistanceSensorData distance_data = {
    .obstacle_detected = false,
    .us_left_distance = US_MAX_DISTANCE_CM,
    .us_right_distance = US_MAX_DISTANCE_CM,
    .ir_left_obstacle = false,
    .ir_right_obstacle = false
};

// ====================== TAREA: Monitoreo por Serial ======================
void Task_SerialMonitor(void* pvParameters) {
    const TickType_t period = pdMS_TO_TICKS(500); // cada 500ms
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);

        Serial.println("==== Estado de sensores ====");
        Serial.print("US Left: ");
        Serial.print(distance_data.us_left_distance);
        Serial.print(" cm | US Right: ");
        Serial.print(distance_data.us_right_distance);
        Serial.println(" cm");

        Serial.print("IR Left Obstacle: ");
        Serial.print(distance_data.ir_left_obstacle ? "YES" : "NO");
        Serial.print(" | IR Right Obstacle: ");
        Serial.println(distance_data.ir_right_obstacle ? "YES" : "NO");

        Serial.print("Obstacle Detected (US): ");
        Serial.println(distance_data.obstacle_detected ? "YES" : "NO");
        Serial.println();
    }
}

// ====================== SETUP ======================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Iniciando main_distance_sensors_basic...");

    // Inicializar sensores
    DistanceSensors::init(&system_states.distance);
    DistanceSensors::set_state(ACTIVE, &system_states.distance);

    // Crear tareas
    xTaskCreatePinnedToCore(
        DistanceSensors::Task_FrontObstacleDetect,
        "FrontUS",
        2048, nullptr, 1, nullptr, APP_CPU_NUM
    );

    // xTaskCreatePinnedToCore(
    //     DistanceSensors::Task_LateralObstacleDetect,
    //     "LateralIR",
    //     2048, nullptr, 1, nullptr, APP_CPU_NUM
    // );

    xTaskCreatePinnedToCore(
        Task_SerialMonitor,
        "SerialPrint",
        2048, nullptr, 1, nullptr, APP_CPU_NUM
    );
}

// ====================== LOOP VACÍO ======================
void loop() {
    // No se usa, todo ocurre en tareas RTOS
}
