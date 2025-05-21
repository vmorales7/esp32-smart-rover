#include "project_config.h"
#include "motor_drive/motor_controller.h"
#include "sensors_firmware/encoder_reader.h"
#include "sensors_firmware/distance_sensors.h"
#include "position_system/position_controller.h"

#warning "Compilando main_avance2.cpp"

// ====================== VARIABLES GLOBALES ======================
volatile SystemStates states;
volatile WheelsData wheels = {0};
volatile DistanceSensorData distances = {0};
volatile KinematicState kinematic = {0};

GlobalContext ctx = {
    .systems_ptr     = &states,
    .os_ptr          = nullptr,         // Aún no se usa OperationData
    .kinematic_ptr   = &kinematic,
    .wheels_ptr      = &wheels,
    .imu_ptr         = nullptr,         // IMU no implementada aún
    .distance_ptr    = &distances
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
volatile uint8_t fase_indice_actual = 0;

constexpr uint32_t PRINT_INTERVAL_MS = 250;

struct Fase {
    const char* nombre;
    float wL_ref;
    float wR_ref;
    uint32_t duracion_ms;
};

constexpr uint8_t NUM_FASES = 5;
Fase fases[NUM_FASES] = {
    {"Avanzando recto (0.5 wn)", 0.5f * WM_NOM, 0.5f * WM_NOM, 10000},
    {"Deteniendo", 0.0f, 0.0f, 3000},
    {"Giro derecha (0.4 wn)", -0.4f * WM_NOM, 0.4f * WM_NOM, 5000},
    {"Deteniendo", 0.0f, 0.0f, 3000},
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

                if (distances.obstacle_detected) {
                    fase_estado = FaseEstado::OBSTACULO;
                    fase_wL_ref = 0.0f;
                    fase_wR_ref = 0.0f;
                    break;
                }

                fase_wL_ref = fases[fase_indice_actual].wL_ref;
                fase_wR_ref = fases[fase_indice_actual].wR_ref;

                const uint32_t t_actual = millis();
                fase_tiempo_acumulado += (t_actual - t_anterior);
                t_anterior = t_actual;

                if (fase_tiempo_acumulado >= fases[fase_indice_actual].duracion_ms) {
                    fase_indice_actual++;
                    fase_tiempo_acumulado = 0;
                    if(fase_indice_actual <= NUM_FASES-1) {
                        Serial.printf("Fase completada. Avanzando a fase %d\n", fase_indice_actual);
                    }
                }
                break;
            }
            case FaseEstado::OBSTACULO: {
                fase_wL_ref = 0.0f;
                fase_wR_ref = 0.0f;
                if (fabsf(wheels.w_L) < W_STOP_THRESHOLD &&
                    fabsf(wheels.w_R) < W_STOP_THRESHOLD) {
                    MotorController::set_motors_mode(MotorMode::IDLE, states.motors, wheels.duty_L, wheels.duty_R);
                    fase_estado = FaseEstado::DETENIDO_POR_OBSTACULO;
                    Serial.println("Vehiculo detenido completamente por obstaculo.");
                }
                break;
            }
            case FaseEstado::DETENIDO_POR_OBSTACULO: {
                const bool obstacle_flag = DistanceSensors::compute_global_obstacle_flag(
                    distances.left_obst, distances.mid_obst, distances.right_obst);
                if (!obstacle_flag) {
                    fase_estado = FaseEstado::EJECUTANDO;
                    MotorController::set_motors_mode(MotorMode::AUTO, states.motors, wheels.duty_L, wheels.duty_R);
                    fase_wL_ref = fases[fase_indice_actual].wL_ref;
                    fase_wR_ref = fases[fase_indice_actual].wR_ref;
                    Serial.println("Obstaculo retirado. Reanudando fase actual.");
                    DistanceSensors::update_global_obstacle_flag(
                        distances.left_obst, distances.mid_obst, distances.right_obst, distances.obstacle_detected);
                }
                break;
            }
            case FaseEstado::FINALIZADO: {
                fase_wL_ref = 0.0f;
                fase_wR_ref = 0.0f;
                PositionController::set_wheel_speed_ref(
                    fase_wL_ref, fase_wR_ref, wheels.w_L_ref, wheels.w_R_ref, states.position);
                MotorController::set_motors_mode(MotorMode::IDLE, states.motors, wheels.duty_L, wheels.duty_R);
                Serial.println("Finalizado.");
                vTaskSuspend(nullptr);
                break;
            }
        }

        PositionController::set_wheel_speed_ref(fase_wL_ref, fase_wR_ref, wheels.w_L_ref, wheels.w_R_ref, states.position);
    }
}

void Task_Printer(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(PRINT_INTERVAL_MS);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        if (fase_estado != FaseEstado::FINALIZADO) {
            Serial.printf("wL_ref %.1f | wR_ref %.1f\n",
                        wheels.w_L_ref, wheels.w_R_ref);        
            Serial.printf("wL %.1f dutyL %.2f | wR %.1f dutyR %.2f\n",
                        wheels.w_L, wheels.duty_L, wheels.w_R, wheels.duty_R);
            Serial.println();
        }
    }
}


// ====================== SETUP ======================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Inicio: Avance 2");
    delay(1000);

    MotorController::init(states.motors, wheels.duty_L, wheels.duty_R);
    MotorController::set_motors_mode(MotorMode::AUTO, states.motors, wheels.duty_L, wheels.duty_R);

    PositionController::init(states.position, wheels.w_L_ref, wheels.w_R_ref);
    PositionController::set_control_mode(PositionControlMode::MANUAL, states.position, wheels.w_L_ref, wheels.w_R_ref);

    DistanceSensors::init(
        distances.left_dist, distances.left_obst, distances.mid_dist, distances.mid_obst,
        distances.right_dist, distances.right_obst, distances.obstacle_detected, states.distance
    );
    DistanceSensors::set_state(
        ACTIVE, states.distance,
        distances.left_dist, distances.left_obst, distances.mid_dist, distances.mid_obst,
        distances.right_dist, distances.right_obst, distances.obstacle_detected
    );

    EncoderReader::init(wheels.steps_L, wheels.steps_R, wheels.w_L, wheels.w_R, states.encoders);
    EncoderReader::resume(states.encoders);

    // Tareas principales (núcleo 1)
    xTaskCreatePinnedToCore(EncoderReader::Task_EncoderUpdate, "EncoderUpdate", 2048, &ctx, 1, nullptr, 1);
    xTaskCreatePinnedToCore(MotorController::Task_WheelControl, "WheelControl", 2048, &ctx, 1, nullptr, 1);
    xTaskCreatePinnedToCore(Task_FaseManager, "FaseManager", 2048, nullptr, 2, nullptr, 1);

    // Tareas secundaria (núcleo 0)
    xTaskCreatePinnedToCore(DistanceSensors::Task_CheckLeftObstacle, "US_Left", 2048, &ctx, 1, nullptr, 0);
    xTaskCreatePinnedToCore(DistanceSensors::Task_CheckMidObstacle, "US_Mid", 2048, &ctx, 1, nullptr, 0);
    xTaskCreatePinnedToCore(DistanceSensors::Task_CheckRightObstacle, "US_Right", 2048, &ctx, 1, nullptr, 0);
    xTaskCreatePinnedToCore(Task_Printer, "Printer", 2048, nullptr, 1, nullptr, 0);
}

void loop() {
    // Vacío: todo funciona por tareas RTOS
}
