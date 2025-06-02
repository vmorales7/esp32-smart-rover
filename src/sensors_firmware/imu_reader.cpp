#include "imu_reader.h"
//nuevos cambios

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

namespace IMUSensor{

    // Variables internas del sistema para aceleracion.
    static unsigned long imu_accellastMillis = 0;
    static float current_yaccel=0;
    static float ref_accel_tolerance= 0.1f;
    
    // Variables internas del sistema para obtener angulos.
    static float angle_speed= 0;
    static float delta_orientation= 0;
    static float current_orientation= 0;
    static float last_orientation= 0;

void reset(
        volatile float& imu_yaccel,
        volatile float& imu_wgyro,
        volatile float& imu_orientation
    ){
        imu_yaccel= 0.0f;
        imu_wgyro= 0.0f;
        imu_orientation= 0.0f;
    };

    void init(
        volatile float& imu_yaccel,
        volatile float& imu_wgyro,
        volatile float& imu_orientation,
        volatile uint8_t imu_state){
        // Comenzamos omunicacion I2C
        Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
        // inicializamos el sensor
        bno.begin();

        reset(imu_yaccel, imu_wgyro, imu_orientation);
    };

    void set_state(uint8_t imu_mode, volatile uint8_t imu_state){
        Serial.println(imu_state);
        if (imu_mode==ACTIVE){
            imu_state=ACTIVE;
        }
        Serial.println(imu_state);
    }; 

    void imu_read_data(
        volatile float& imu_yaccel,
        volatile float& imu_wgyro,
        volatile float& imu_orientation,
        volatile uint8_t imu_state
    ){  //Serial.println(imu_state);
        if (imu_state!=ACTIVE) return; //solo se lee si el sistema esta activo
        //Serial.println("Ahora si entramos a la funcion!");
        unsigned long imu_accelMillis = millis();
        float imudt = (imu_accelMillis - imu_accellastMillis) * MS_TO_S;  // Tiempo (s) transcurrido desde la ultima actualización
        sensors_event_t accel_data;
        sensors_event_t gyro_data;
        sensors_event_t orientation_data;
        bno.getEvent(&accel_data, Adafruit_BNO055::VECTOR_LINEARACCEL);
        bno.getEvent(&gyro_data, Adafruit_BNO055::VECTOR_GYROSCOPE);
        bno.getEvent(&orientation_data, Adafruit_BNO055::VECTOR_EULER);

        // Creacion de variables temporales para obtener velocidad
        if (accel_data.acceleration.y>ref_accel_tolerance | accel_data.acceleration.y<-ref_accel_tolerance){
            current_yaccel= -(accel_data.acceleration.y+0.13);
        };
        angle_speed= gyro_data.gyro.z;
        current_orientation= orientation_data.orientation.x; 

        delta_orientation= (current_orientation-last_orientation);

        imu_yaccel= current_yaccel;
        imu_wgyro= angle_speed;
        imu_orientation= delta_orientation;

        Serial.print("accel: ");
        Serial.println(current_yaccel);

        last_orientation= current_orientation;
        imu_accellastMillis = imu_accelMillis;
    };

     void Task_IMUData(void* pvParameters) {
        // Datos de RTOS
        TickType_t xLastWakeTime = xTaskGetTickCount();
        const TickType_t period = pdMS_TO_TICKS(IMU_READ_PERIOD_MS);
    
        // Cast del parámetro entregado a GlobalContext
        GlobalContext* ctx = static_cast<GlobalContext*>(pvParameters);
        volatile SystemStates& sts = *ctx->systems_ptr;
        volatile SensorsData& sens = *ctx->sensors_ptr;
        Serial.println("setting up with RTOS");
        for (;;) {
            vTaskDelayUntil(&xLastWakeTime, period);
            imu_read_data(sens.imu_ay, sens.imu_wz, sens.imu_theta,
                sts.imu);
        }
    }  
};