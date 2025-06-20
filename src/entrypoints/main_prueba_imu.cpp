#include "project_config.h"
#include "sensors_firmware/imu_reader.h"

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

void setup(){
  Serial.begin(115200);

  delay(1000);
  Serial.println("test iMU con RTOS");
  IMUSensor::init(sens.imu_ay, sens.imu_wz, sens.imu_theta, sts.imu);
  Serial.println("Inicializamos ya....");
  IMUSensor::set_state(ACTIVE, sts.imu);
  sts.imu=ACTIVE;
  //Serial.println(sts.imu);

  // Crear tareas para el IMU
    xTaskCreatePinnedToCore(IMUSensor::Task_IMUData, "IMU_Check", 2048, &ctx, 1, nullptr, 0);
}

void loop(){

    //puro RTOS....

}