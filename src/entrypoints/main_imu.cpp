#include "vehicle_os/general_config.h"
#include "sensors_firmware/imu_reader.h"

#warning "Compilando main_imu.cpp"

// Instancias globales
volatile SystemStates sts;
volatile SensorsData sens;

GlobalContext ctx = {
    .systems_ptr   = &sts,
    .sensors_ptr   = &sens,
    .control_ptr   = nullptr,
    .os_ptr        = nullptr,
    .rtos_task_ptr = nullptr,
    .evade_ptr     = nullptr
};

// Tarea para mostrar datos por serial (NO llames Serial en la tarea IMU)
void Task_PrintIMU(void* pvParameters) {
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t print_period = pdMS_TO_TICKS(200); // 200 ms

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, print_period);
        char buffer[64];
        sprintf(buffer, "ax (m/s2):%6.2f      wz (rad/s):%7.2f      theta (°):%7.2f", 
            sens.imu_acc, sens.imu_w, sens.imu_theta * 180.0f / M_PI);
        Serial.println(buffer);
    }
}

void setup(){
    Serial.begin(115200);
    delay(1000);

    Serial.println("Test IMU con RTOS");

    // Inicialización de IMU
    IMUSensor::init(sens.imu_acc, sens.imu_w, sens.imu_theta, sts.imu);
    IMUSensor::set_state(ACTIVE, sts.imu, sens.imu_acc, sens.imu_w, sens.imu_theta);

    // Crear tareas para el IMU y para impresión por serial
    xTaskCreatePinnedToCore(IMUSensor::Task_IMUData, "IMU_Check", 2048, &ctx, 1, nullptr, 0);
    xTaskCreatePinnedToCore(Task_PrintIMU, "Print_IMU", 2048, nullptr, 1, nullptr, 1);
}

void loop(){
    // RTOS se encarga de todo
}