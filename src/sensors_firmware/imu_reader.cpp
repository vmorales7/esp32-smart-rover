#include "imu_reader.h"
//nuevos cambios

namespace IMUSensor{

    // Variables internas del sistema para aceleracion y velocidad.
    static unsigned long imu_accellastMillis = 0;
    static float current_xaccel= 0;
    static float current_yaccel=0;
    static float current_xspeed= 0;
    static float current_yspeed= 0;
    static float last_xaccel= 0;
    static float last_yaccel= 0;
    static float last_xspeed= 0;
    static float last_yspeed= 0;
    static float ref_tolerance= 0.09f;
    
    // Variables internas del sistema para obtener angulos.
    static unsigned long imu_gyrolastMillis= 0;
    static float angle_speed= 0;
    static float delta_magangle= 0;
    static float current_magangle= 0;
    static float last_magangle= 0;

    void init(volatile uint8_t* imu_state_ptr){
        // Comenzamos omunicacion I2C
        Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
        
        if(!IMU.begin())
            {
        /* There was a problem detecting the BNO055 ... check your connections */
              Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
            while(100);
            }
        // Dejar sensores inactivos por defecto
        *imu_state_ptr = INACTIVE;
    };

    void set_state(uint8_t imu_mode, volatile uint8_t* imu_state_ptr){
        *imu_state_ptr = (imu_mode == ACTIVE) ? ACTIVE : INACTIVE;
    }; 

    void imu_reset(
        volatile double* imu_xaccel_ptr,
        volatile double* imu_yaccel_ptr,
        volatile double* imu_xspeed_ptr,
        volatile double* imu_yspeed_ptr,
        volatile float* imu_wgyro_ptr,
        volatile float* imu_magangle_ptr,
        volatile uint8_t* imu_state_ptr
    ){
        if (*imu_state_ptr!=ACTIVE) return; //solo se lee si el sistema esta activo
        *imu_xaccel_ptr= 0.0f;
        *imu_yaccel_ptr= 0.0f;
        *imu_xspeed_ptr= 0.0f;
        *imu_yspeed_ptr= 0.0f;
        *imu_wgyro_ptr= 0.0f;
        *imu_magangle_ptr= 0.0f;
    };

    void imu_read_accel_speed(
        volatile float* imu_xaccel_ptr,
        volatile float* imu_yaccel_ptr,
        volatile float* imu_xspeed_ptr,
        volatile float* imu_yspeed_ptr,
        volatile uint8_t* imu_state_ptr
    ){
        if (*imu_state_ptr!=ACTIVE) return; //solo se lee si el sistema esta activo
        unsigned long imu_accelMillis = millis();
        float imudt = (imu_accelMillis - imu_accellastMillis) * MS_TO_S;  // Tiempo (s) transcurrido desde la ultima actualización
        sensors_event_t acceldata;
        IMU.getEvent(&acceldata, Adafruit_BNO055::VECTOR_LINEARACCEL);

        // Creacion de variables temporales para obtener velocidad
        current_xaccel= acceldata.acceleration.x;
        current_yaccel= acceldata.acceleration.y;

        current_xspeed= last_xspeed+0.5*(current_xaccel+last_xaccel)*imudt;
        current_yspeed= last_yspeed+0.5*(current_yaccel+last_yaccel)*imudt;

        *imu_xaccel_ptr= current_xaccel;
        *imu_yaccel_ptr= current_yaccel;
        *imu_xspeed_ptr= current_xspeed;
        *imu_yspeed_ptr= current_yspeed;

        last_xaccel= current_xaccel;
        last_yaccel= current_yaccel;
        last_xspeed= current_xspeed;
        last_yspeed= current_yspeed;
        imu_accellastMillis = imu_accelMillis;
    };

    void imu_read_angles(
        volatile float& imu_wgyro_ptr,
        volatile float& imu_magangle_ptr,
        volatile uint8_t& imu_state_ptr
    ){
      if (imu_state_ptr!=ACTIVE) return;
        unsigned long imu_gyroMillis = millis();
        float imudt = (imu_gyroMillis - imu_gyrolastMillis) * MS_TO_S;  // Tiempo (s) transcurrido desde la ultima actualización
        sensors_event_t gyrodata;
        sensors_event_t magdata;
        IMU.getEvent(&gyrodata, Adafruit_BNO055::VECTOR_GYROSCOPE);
        IMU.getEvent(&magdata, Adafruit_BNO055::VECTOR_MAGNETOMETER);

        float gyr = gyrodata.gyro.z;
        float mag = magdata.orientation.z;

        angle_speed= gyr;
        current_magangle- mag;

        delta_magangle= (current_magangle-last_magangle);

        imu_wgyro_ptr= angle_speed;
        imu_magangle_ptr= delta_magangle;

        last_magangle= current_magangle;
        imu_gyrolastMillis= imu_gyroMillis;
    };

     void Task_IMUData(void* pvParameters) {
        // Datos de RTOS
        TickType_t xLastWakeTime = xTaskGetTickCount();
        const TickType_t period = pdMS_TO_TICKS(IMU_READ_PERIOD_MS);
    
        // Cast del parámetro entregado a GlobalContext
        GlobalContext* ctx_ptr = static_cast<GlobalContext*>(pvParameters);
    
        // Punteros a las variables necesarias
        volatile uint8_t* imu_state_ptr = &ctx_ptr->systems_ptr->imu;
        volatile float* imu_xaccel_ptr  = &ctx_ptr->imu_ptr->ax;
        volatile float* imu_yaccel_ptr  = &ctx_ptr->imu_ptr->ay;
        volatile float* imu_xspeed_ptr  = &ctx_ptr->imu_ptr->vx;
        volatile float* imu_yspeed_ptr  = &ctx_ptr->imu_ptr->vy;
        volatile float* imu_wgyro_ptr    = &ctx_ptr->imu_ptr->w_gyro;
        volatile float* imu_magangle_ptr = &ctx_ptr->imu_ptr->mag_angle;
    
        for (;;) {
            vTaskDelayUntil(&xLastWakeTime, period);
            imu_read_accel_speed(imu_xaccel_ptr, imu_yaccel_ptr,
                imu_xspeed_ptr, imu_yspeed_ptr, imu_state_ptr
            );

            imu_read_angles( imu_wgyro_ptr, imu_magangle_ptr, 
                imu_state_ptr
            );
        }
    }  

};