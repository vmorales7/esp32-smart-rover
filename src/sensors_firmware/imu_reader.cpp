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
    
    // Variables internas del sistema para obtener angulos.
    static unsigned long imu_gyrolastMillis= 0;
    static float angle_speed= 0;
    static float delta_magangle= 0;
    static float current_magangle= 0;
    static float last_magangle= 0;

    void init(volatile uint8_t* imu_state_ptr){
        // Comenzamos omunicacion I2C
        Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
        
        if(!IMU.init()){
          Serial.println("MPU9250 does not respond");
        }
        else{
          Serial.println("MPU9250 is connected");
        }
        if(!IMU.initMagnetometer()){
          Serial.println("Magnetometer does not respond");
        }
        else{
          Serial.println("Magnetometer is connected");
        }
        Serial.println("Position you MPU9250 flat and don't move it - calibrating...");
        delay(500);
        IMU.autoOffsets();  //Calibracion de la IMU

        IMU.enableGyrDLPF();  // Filtrado pasabajos digital para minimar ruido de muestras
        IMU.setGyrDLPF(MPU9250_DLPF_5); // Nivel de delay para lectura de datos limpios aprox 34 ms
        IMU.setSampleRateDivider(5); // Division de sample rate en orden de normalizar
        IMU.setGyrRange(MPU9250_GYRO_RANGE_500); // Colocando maximo espectro de giro por segundo.

        IMU.setAccRange(MPU9250_ACC_RANGE_2G); // Set-up de rango de mediciones para el acelerometro
        IMU.enableAccDLPF(true); // Filtrado pasabajos para datos del acelerometro
        IMU.setAccDLPF(MPU9250_DLPF_4);  // filtrado de datos del acelerometro

        IMU.enableAccAxes(MPU9250_ENABLE_XY0); // Seleccionamos uso de solamente de ejes X e Y
        IMU.enableGyrAxes(MPU9250_ENABLE_00Z); // Solo obtenemos velocidad angular en eje Z (giro entre X e Y)

        IMU.setMagOpMode(AK8963_CONT_MODE_100HZ); // Rate de envio de datos del magnetometro

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
        xyzFloat gValue = IMU.getGValues();

        // Creacion de variables temporales para obtener velocidad
        current_xaccel= gValue.x;
        current_yaccel= gValue.y;

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
        volatile float* imu_wgyro_ptr,
        volatile float* imu_magangle_ptr,
        volatile uint8_t* imu_state_ptr
    ){
      if (*imu_state_ptr!=ACTIVE) return;
        unsigned long imu_gyroMillis = millis();
        float imudt = (imu_gyroMillis - imu_gyrolastMillis) * MS_TO_S;  // Tiempo (s) transcurrido desde la ultima actualización
        
        xyzFloat gyr = IMU.getGyrValues();
        xyzFloat magValue = IMU.getMagValues();

        angle_speed= gyr.z;
        current_magangle- magValue.z;

        delta_magangle= (current_magangle-last_magangle);

        *imu_wgyro_ptr= angle_speed;
        *imu_magangle_ptr= delta_magangle;

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
        volatile float* imu_xaccel_ptr  = &ctx_ptr->imu_ptr->x_accel;
        volatile float* imu_yaccel_ptr  = &ctx_ptr->imu_ptr->y_accel;
        volatile float* imu_xspeed_ptr  = &ctx_ptr->imu_ptr->x_speed;
        volatile float* imu_yspeed_ptr  = &ctx_ptr->imu_ptr->y_speed;
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