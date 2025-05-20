#include "project_config.h"
#include "motor_drive/motor_controller.h"
#include "sensors_firmware/encoder_reader.h"
#include "sensors_firmware/distance_sensors.h"
#include "position_system/pose_estimator.h"
#include "position_system/position_controller.h"

#warning "Compilando main_position_points.cpp"


// ====================== VARIABLES GLOBALES ======================
volatile SystemStates states = {0};
volatile WheelsData wheels = {0};
volatile KinematicState kinem = {0};
volatile DistanceSensorData distance_data = {0};
volatile PoseData pose = {0};

GlobalContext ctx = {
    .systems_ptr     = &states,
    .os_ptr          = nullptr,
    .kinematic_ptr   = &kinem,
    .wheels_ptr      = &wheels,
    .imu_ptr         = nullptr,
    .distance_ptr    = &distance_data
};


// ====================== CONFIGURACIÓN PUNTOS ======================
struct Punto {
    float x;
    float y;
};
constexpr uint8_t NUM_PUNTOS = 3;
Punto trayectoria[NUM_PUNTOS] = {
    {1.0f, 1.0f},
    {2.0f, 1.0f},
    {2.0f, 2.0f}
};
volatile uint8_t punto_actual = 0;


// ====================== TAREA PRINCIPAL ======================
void Task_ControlPorPuntos(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(100);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);

        // Si ya completó todos los puntos
        if (punto_actual >= NUM_PUNTOS) {
            MotorController::set_motors_mode(MOTOR_IDLE, states.motors, wheels.duty_L, wheels.duty_R);
            PositionController::set_wheel_speed_ref(0.0f, 0.0f, wheels.w_L_ref, wheels.w_R_ref, states.position);
            vTaskSuspend(nullptr);
        }

        // Si hay obstáculo, detenemos motores (solo referencia)
        DistanceSensors::update_global_obstacle_flag(
            distance_data.left_obst, distance_data.mid_obst, distance_data.right_obst, distance_data.obstacle_detected);
        if (distance_data.obstacle_detected) {
            PositionController::set_wheel_speed_ref(0.0f, 0.0f, wheels.w_L_ref, wheels.w_R_ref, states.position);
            continue;
        }

        // Actualizar punto objetivo
        kinem.x_d = trayectoria[punto_actual].x;
        kinem.y_d = trayectoria[punto_actual].y;

        // Verificar si ya llegó
        float dx = kinem.x_d - kinem.x;
        float dy = kinem.y_d - kinem.y;
        float dist = sqrtf(dx * dx + dy * dy);
      
        if (dist > DISTANCE_TOLERANCE) {
            Serial.printf("Punto %d no alcanzado (x=%.2f, y=%.2f)\n", punto_actual, kinem.x, kinem.y);
        } else {
            Serial.printf("Punto %d alcanzado (x=%.2f, y=%.2f)\n", punto_actual, kinem.x, kinem.y);
            punto_actual++;
        }
    }
}


// ====================== SETUP ======================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Inicio: Seguimiento de puntos");

    // Inicialización de sistemas
    EncoderReader::init(wheels.steps_L, wheels.steps_R, wheels.w_L, wheels.w_R, states.encoders);
    PoseEstimator::init(kinem.x, kinem.y, kinem.theta, kinem.v, kinem.w, wheels.steps_L, wheels.steps_R, states.pose);
    MotorController::init(states.motors, wheels.duty_L, wheels.duty_R);
    DistanceSensors::init(
        distance_data.left_dist, distance_data.left_obst, 
        distance_data.mid_dist, distance_data.mid_obst,
        distance_data.right_dist, distance_data.right_obst, 
        distance_data.obstacle_detected, states.distance
    );
    PositionController::init(states.position, wheels.w_L_ref, wheels.w_R_ref);

    // Puesta en marcha de todo
    EncoderReader::resume(states.encoders);
    PoseEstimator::set_state(ACTIVE, states.pose);
    DistanceSensors::set_state(
        ACTIVE, states.distance,
        distance_data.left_dist, distance_data.left_obst, 
        distance_data.mid_dist, distance_data.mid_obst,
        distance_data.right_dist, distance_data.right_obst, 
        distance_data.obstacle_detected
    );
    MotorController::set_motors_mode(MOTOR_AUTO, states.motors, wheels.duty_L, wheels.duty_R);
    PositionController::set_control_mode(SPEED_REF_AUTO_BASIC, states.position, wheels.w_L_ref, wheels.w_R_ref);

    // Tareas de sistema
    xTaskCreatePinnedToCore(EncoderReader::Task_EncoderUpdate, "EncoderUpdate", 2048, &ctx, 1, nullptr, 1);
    xTaskCreatePinnedToCore(MotorController::Task_WheelControl, "WheelControl", 2048, &ctx, 1, nullptr, 1);
    xTaskCreatePinnedToCore(PoseEstimator::Task_PoseEstimatorEncoder, "PoseEstimator", 2048, &ctx, 1, nullptr, 1);
    xTaskCreatePinnedToCore(PositionController::Task_PositionControl, "PositionControl", 2048, &ctx, 1, nullptr, 1);

    // Sensores de distancia (core 0)
    xTaskCreatePinnedToCore(DistanceSensors::Task_CheckLeftObstacle, "US_Left", 2048, &ctx, 1, nullptr, 0);
    xTaskCreatePinnedToCore(DistanceSensors::Task_CheckMidObstacle, "US_Mid", 2048, &ctx, 1, nullptr, 0);
    xTaskCreatePinnedToCore(DistanceSensors::Task_CheckRightObstacle, "US_Right", 2048, &ctx, 1, nullptr, 0);

    // Tarea principal de control por puntos
    xTaskCreatePinnedToCore(Task_ControlPorPuntos, "ControlPorPuntos", 2048, nullptr, 2, nullptr, 0);
}

void loop() {
    // No se usa
}