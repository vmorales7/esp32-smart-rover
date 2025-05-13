#include "project_config.h"
#include "motor_drive/motor_controller.h"
#include "sensors_firmware/encoder_reader.h"
#include "sensors_firmware/distance_sensors.h"
#include "position_system/position_controller.h"

#warning "Compilando main_avance2.cpp"

// ====================== VARIABLES GLOBALES ======================
volatile SystemStates system_states = {0};
volatile WheelsData wheels_data = {0};
volatile DistanceSensorData distance_data = {0};
volatile KinematicState kinematic_data = {0};

GlobalContext ctx = {
    .systems_ptr = &system_states,
    .kinematic_ptr = &kinematic_data,
    .wheels_ptr = &wheels_data,
    .distance_ptr = &distance_data
};

// constexpr float W_STOP_THRESHOLD = 0.05 * WM_NOM;  // Threshold de velocidad para considerar stop [rad/s]
const uint16_t TASK_CONTROL_PERIOD = 100;


// ====================== MAQUINA DE ESTADOS ======================
enum class FaseEstado : uint8_t {
    EJECUTANDO,
    OBSTACULO,
    DETENIDO_POR_OBSTACULO,
    FINALIZADO
};
volatile FaseEstado fase_estado = FaseEstado::EJECUTANDO;

// Variables de fase
volatile float fase_wL_ref = 0.0f;
volatile float fase_wR_ref = 0.0f;
volatile uint32_t fase_tiempo_acumulado = 0;
volatile uint8_t fase_indice_actual = 1;

constexpr uint32_t PRINT_INTERVAL_MS = 250;

struct Fase {
    const char* nombre;
    float wL_ref;
    float wR_ref;
    uint32_t duracion_ms;
};

constexpr uint8_t NUM_FASES = 3;
Fase fases[NUM_FASES] = {
    {"Avanzando recto (0.5 wn)", 0.5f * WM_NOM, 0.5f * WM_NOM, 10000},
    {"Giro derecha (0.4 wn)", -0.4f * WM_NOM, 0.4f * WM_NOM, 5000},
    {"Avanzando recto (0.5 wn)", 0.5f * WM_NOM, 0.5f * WM_NOM, 10000},
};

// ====================== TAREAS ======================
void Task_FaseManager(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(TASK_CONTROL_PERIOD);
    static uint32_t t_anterior = millis();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);

        switch (fase_estado) {
            case FaseEstado::EJECUTANDO: {
                if (fase_indice_actual >= NUM_FASES) {
                    fase_estado = FaseEstado::FINALIZADO;
                    break;
                }

                if (distance_data.obstacle_detected) {
                    fase_estado = FaseEstado::OBSTACULO;
                    fase_wL_ref = 0.0f;
                    fase_wR_ref = 0.0f;
                    break;
                }

                fase_wL_ref = fases[fase_indice_actual].wL_ref;
                fase_wR_ref = fases[fase_indice_actual].wR_ref;

                uint32_t t_actual = millis();
                fase_tiempo_acumulado += (t_actual - t_anterior);
                t_anterior = t_actual;

                if (fase_tiempo_acumulado >= fases[fase_indice_actual].duracion_ms) {
                    fase_indice_actual++;
                    fase_tiempo_acumulado = 0;
                    Serial.printf("Fase completada. Avanzando a fase %d\n", fase_indice_actual);
                }
                break;
            }
            case FaseEstado::OBSTACULO: {
                fase_wL_ref = 0.0f;
                fase_wR_ref = 0.0f;
                if (fabsf(wheels_data.wL_measured) < W_STOP_THRESHOLD &&
                    fabsf(wheels_data.wR_measured) < W_STOP_THRESHOLD) {
                    // system_states.motor_operation = INACTIVE;
                    // system_states.encoder = INACTIVE;
                    // system_states.position_controller = INACTIVE;
                    fase_estado = FaseEstado::DETENIDO_POR_OBSTACULO;
                    Serial.println("Vehiculo detenido completamente por obstaculo.");
                }
                break;
            }
            case FaseEstado::DETENIDO_POR_OBSTACULO: {
                if (!distance_data.obstacle_detected) {
                    fase_estado = FaseEstado::EJECUTANDO;
                    Serial.println("Obstaculo retirado. Reanudando fase actual.");
                }
                break;
            }
            case FaseEstado::FINALIZADO: {
                fase_wL_ref = 0.0f;
                fase_wR_ref = 0.0f;
                Serial.println("Finalizado.");
                break;
            }
        }

        PositionController::set_wheel_speed_ref(
            fase_wL_ref, fase_wR_ref,
            wheels_data.wL_ref, wheels_data.wR_ref,
            system_states.position_controller
        );
    }
}

void Task_Printer(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(PRINT_INTERVAL_MS);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        Serial.printf("wL_ref %.1f | wR_ref %.1f\n",
                      wheels_data.wL_ref, wheels_data.wR_ref);        
        Serial.printf("wL %.1f dutyL %.2f | wR %.1f dutyR %.2f\n",
                      wheels_data.wL_measured, wheels_data.duty_left,
                      wheels_data.wR_measured, wheels_data.duty_right);
        Serial.println();
    }
}

void Task_UpdateObstacleFlag(void* pvParameters) {
    const TickType_t period = pdMS_TO_TICKS(25);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    GlobalContext* ctx = static_cast<GlobalContext*>(pvParameters);
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        DistanceSensors::update_global_obstacle_flag(*ctx->distance_ptr);
    }
}

// ====================== SETUP ======================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Inicio: Avance 2 con RTOS y detección de obstáculos");

    MotorController::init(system_states.motor_operation, wheels_data.duty_left, wheels_data.duty_right);
    MotorController::set_motors_mode(MOTOR_AUTO, system_states.motor_operation, wheels_data.duty_left, wheels_data.duty_right);

    EncoderReader::init(wheels_data, system_states.encoder);
    EncoderReader::resume(system_states.encoder);

    PositionController::init(system_states.position_controller, wheels_data.wL_ref, wheels_data.wR_ref);
    PositionController::set_control_mode(SPEED_REF_MANUAL, system_states.position_controller);

    DistanceSensors::init_system(system_states.distance, distance_data);
    DistanceSensors::set_state(ACTIVE, system_states.distance);

    // Tareas principales (núcleo 1)
    xTaskCreatePinnedToCore(EncoderReader::Task_EncoderUpdate, "EncoderUpdate", 2048, &ctx, 1, nullptr, 1);
    xTaskCreatePinnedToCore(MotorController::Task_WheelControl, "WheelControl", 2048, &ctx, 1, nullptr, 1);
    xTaskCreatePinnedToCore(Task_FaseManager, "FaseManager", 2048, nullptr, 2, nullptr, 1);

    // Tareas secundaria (núcleo 0)
    xTaskCreatePinnedToCore(DistanceSensors::Task_CheckLeftObstacle, "US_Left", 2048, &ctx, 1, nullptr, 0);
    xTaskCreatePinnedToCore(DistanceSensors::Task_CheckMidObstacle, "US_Mid", 2048, &ctx, 1, nullptr, 0);
    xTaskCreatePinnedToCore(DistanceSensors::Task_CheckRightObstacle, "US_Right", 2048, &ctx, 1, nullptr, 0);
    xTaskCreatePinnedToCore(Task_UpdateObstacleFlag, "UpdateObstacleFlag", 2048, &ctx, 1, nullptr, 0);
    xTaskCreatePinnedToCore(Task_Printer, "Printer", 2048, nullptr, 1, nullptr, 0);
}

void loop() {
    // Vacío: todo funciona por tareas RTOS
}
