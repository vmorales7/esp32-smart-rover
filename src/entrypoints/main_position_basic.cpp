#include "vehicle_os/general_config.h"
#include "motor_drive/motor_controller.h"
#include "sensors_firmware/encoder_reader.h"
#include "sensors_firmware/imu_reader.h"
#include "sensors_firmware/distance_sensors.h"
#include "position_system/pose_estimator.h"
#include "position_system/position_controller.h"

#warning "Compilando main_position_basic.cpp"

// ====================== VARIABLES GLOBALES ======================
volatile SystemStates sts;
volatile SensorsData sens;
volatile ControllerData ctrl;
volatile PoseData pose;

GlobalContext ctx = {
    .systems_ptr     = &sts,
    .sensors_ptr     = &sens,
    .pose_ptr        = &pose,
    .control_ptr     = &ctrl,
    .os_ptr          = nullptr, 
    .rtos_task_ptr   = nullptr,
    .evade_ptr       = nullptr
};


// ====================== CONFIGURACIÓN OBJETIVO ======================

// Configuración de control y estimación de posición
constexpr ControlType CONTROLLER_TYPE = ControlType::PID;
constexpr PoseEstimatorType POSE_ESTIMATOR_TYPE = PoseEstimatorType::ENCODER;

// Escoger objetivo y modo de control: MOVE o ROTATE
constexpr PositionControlMode CONTROL_MODE = PositionControlMode::MOVE; 
constexpr float X_OBJETIVO = 1.0f; 
constexpr float Y_OBJETIVO = 1.0f;
constexpr float Q_OBJETIVO = 180.0f * DEG_TO_RAD; // radianes

bool en_pausa_por_obstaculo = false; // Indica si se pausó por un obstáculo

// ====================== PROTOTIPOS ======================

void Task_PrintPose(void* pvParameters);
void Task_PrintXY(void* pvParameters);
void Task_Obstacle(void* pvParameters);


// ====================== SETUP ======================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println();
    Serial.println("Main: Position Control — Basic");
    Serial.println();
    delay(5000);

    // Inicialización de módulos
    EncoderReader::init(sens.enc_phiL, sens.enc_phiR, sens.enc_wL, sens.enc_wR, sts.encoders);
    if (POSE_ESTIMATOR_TYPE == PoseEstimatorType::COMPLEMENTARY) {
        IMUSensor::init(sens.imu_acc, sens.imu_w, sens.imu_theta, sts.imu);
    }
    PoseEstimator::init(pose.x, pose.y, pose.theta, pose.v, pose.w, pose.w_L, pose.w_R, 
        sens.enc_phiL, sens.enc_phiR, sens.imu_theta, sts.pose);
    pose.estimator_type = POSE_ESTIMATOR_TYPE; // Establecer tipo de estimador
    MotorController::init(sts.motors, ctrl.duty_L, ctrl.duty_R);
    PositionController::init(sts.position, ctrl.x_d, ctrl.y_d, ctrl.theta_d, 
        ctrl.waypoint_reached, ctrl.w_L_ref, ctrl.w_R_ref); 
    DistanceSensors::init(sens.us_left_dist, sens.us_left_obst, sens.us_mid_dist, sens.us_mid_obst, 
        sens.us_right_dist, sens.us_right_obst, sens.us_obstacle, sts.distance);   

    // Crear tareas principales
    if (POSE_ESTIMATOR_TYPE == PoseEstimatorType::COMPLEMENTARY) {
        xTaskCreatePinnedToCore(IMUSensor::Task_IMUData, "ReadIMU", 2048, &ctx, 2, nullptr, 1);
    }
    xTaskCreatePinnedToCore(EncoderReader::Task_EncoderUpdate, "EncoderUpdate", 2048, &ctx, 2, nullptr, 1);
    xTaskCreatePinnedToCore(MotorController::Task_WheelControl, "WheelControl", 2048, &ctx, 2, nullptr, 1);
    xTaskCreatePinnedToCore(PoseEstimator::Task_PoseEstimatorEncoder, "PoseEstimator", 2048, &ctx, 2, nullptr, 1);
    xTaskCreatePinnedToCore(PositionController::Task_PositionControl, "PositionControl", 2048, &ctx, 2, nullptr, 1);
    xTaskCreatePinnedToCore(DistanceSensors::Task_CheckObstacle, "CheckObstacles", 2048, &ctx, 2, nullptr, 0);
    xTaskCreatePinnedToCore(Task_Obstacle, "ObstacleHandler", 2048, &ctx, 1, nullptr, 0);

    // Debug
    xTaskCreatePinnedToCore(Task_PrintPose, "PrintPose", 2048, &ctx, 1, nullptr, 0);
    //xTaskCreatePinnedToCore(Task_PrintXY, "PrintXY", 2048, &ctx, 1, nullptr, 0);

    // Activar subsistemas
    EncoderReader::resume(sts.encoders);
    if (POSE_ESTIMATOR_TYPE == PoseEstimatorType::ENCODER) {
        IMUSensor::set_state(ACTIVE, sts.imu, sens.imu_acc, sens.imu_w, sens.imu_theta);
    }
    PoseEstimator::set_state(ACTIVE, sts.pose);
    PositionController::set_controller_type(CONTROLLER_TYPE, ctrl.controller_type);
    DistanceSensors::set_state(
        ACTIVE, sts.distance, sens.us_left_obst, sens.us_mid_obst, sens.us_right_obst, sens.us_obstacle);

    // Comenzar el control de posición
   MotorController::set_motors_mode(MotorMode::AUTO, sts.motors, ctrl.duty_L, ctrl.duty_R);
    PositionController::set_control_mode(CONTROL_MODE, sts.position, ctrl.w_L_ref, ctrl.w_R_ref);

    // Asignar punto objetivo
    if (CONTROL_MODE == PositionControlMode::ROTATE) {
        PositionController::set_diferential_waypoint(
            0.0f, Q_OBJETIVO, ctrl.x_d, ctrl.y_d, ctrl.theta_d, ctrl.waypoint_reached, sts.position);
            Serial.printf("Ángulo objetivo: %.2f °\n", Q_OBJETIVO * RAD_TO_DEG);
    } else {
        PositionController::set_waypoint(
            X_OBJETIVO, Y_OBJETIVO, 0.0f, ctrl.x_d, ctrl.y_d, ctrl.theta_d, ctrl.waypoint_reached, sts.position);
    }

}

void loop() {
    // No se usa
}

// ====================== TAREA DE IMPRESIÓN ======================
void Task_PrintPose(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(500);
    volatile GlobalContext& ctx = *static_cast<GlobalContext*>(pvParameters);
    volatile PoseData& pose = *ctx.pose_ptr;
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        Serial.printf("x: %.2f  y: %.2f  θ: %.1f  |  v: %.2f  w: %.2f\n", 
            pose.x, pose.y, (pose.theta * 180.0f / M_PI), pose.v, pose.w);
        // Serial.printf("wL_ref: %.2f  d_L: %.2f |  wR_ref: %.2f  d_R: %.2f \n", 
        //     ctrl.w_L_ref, ctrl.duty_L, ctrl.w_R_ref, ctrl.duty_R);
    }
}

void Task_PrintXY(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(250);
    volatile GlobalContext& ctx = *static_cast<GlobalContext*>(pvParameters);
    volatile PoseData& pose = *ctx.pose_ptr;
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        Serial.printf("%.2f %.2f\n", pose.x, pose.y);
    }
}

void Task_Obstacle(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(100);
    volatile GlobalContext& ctx = *static_cast<GlobalContext*>(pvParameters);
    volatile SensorsData& sens = *ctx.sensors_ptr;
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        if (sens.us_obstacle) {
            Serial.println("Obstáculo detectado, deteniendo motores.");
            PositionController::set_control_mode(PositionControlMode::MANUAL, sts.position, 
                ctrl.w_L_ref, ctrl.w_R_ref);
            PositionController::set_wheel_speed_ref(0.0f, 0.0f, ctrl.w_L_ref, ctrl.w_R_ref, sts.position);
            en_pausa_por_obstaculo = true;
        } 
        else if (!sens.us_obstacle && en_pausa_por_obstaculo) {
            Serial.printf("Obstáculo despejado, reanudando movimiento hacia el punto (x=%.2f, y=%.2f)\n", 
                ctrl.x_d, ctrl.y_d);
            PositionController::set_control_mode(CONTROL_MODE, sts.position, ctrl.w_L_ref, ctrl.w_R_ref);
            en_pausa_por_obstaculo = false;
        }
        DistanceSensors::update_global_obstacle_flag(
            sens.us_left_obst, sens.us_mid_obst, sens.us_right_obst, sens.us_obstacle);
    }
}