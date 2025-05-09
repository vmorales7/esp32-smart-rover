#include "distance_sensors.h"

namespace DistanceSensors {

    void init_sensor(uint8_t trig_pin, uint8_t echo_pin) {
        pinMode(trig_pin, OUTPUT);
        pinMode(echo_pin, INPUT);
        digitalWrite(trig_pin, LOW);
    }


    uint8_t read_distance(uint8_t trig_pin, uint8_t echo_pin) {
        // Se generá el trig hacia el sensor
        digitalWrite(trig_pin, LOW);
        delayMicroseconds(2);
        digitalWrite(trig_pin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trig_pin, LOW);

        // Se espera el echo desde el sensor
        uint32_t duration = pulseIn(echo_pin, HIGH, US_PULSE_TIMEOUT_US);

        // Se verifica que no haya timeout, en cuyo caso se devuelve distancia max
        if (duration == 0) return (uint8_t) US_MAX_DISTANCE_CM;

        // Cálculo de la distancia
        return (uint8_t) (duration * US_CM_PER_US);
    }


    void reset_system(volatile DistanceSensorData* data_ptr) {
        // Reset de la estructura DistanceSensorData
        data_ptr->obstacle_detected   = false;
        data_ptr->us_left_obstacle    = false;
        data_ptr->us_mid_obstacle     = false;
        data_ptr->us_right_obstacle   = false;
        data_ptr->us_left_distance    = US_MAX_DISTANCE_CM;
        data_ptr->us_mid_distance     = US_MAX_DISTANCE_CM;
        data_ptr->us_right_distance   = US_MAX_DISTANCE_CM;
    }

    
    void init_system(volatile uint8_t* distance_state_ptr, volatile DistanceSensorData* data_ptr) {
        // Configuración de pines de sensores ultrasónicos
        init_sensor(US_LEFT_TRIG_PIN, US_LEFT_ECHO_PIN);
        init_sensor(US_MID_TRIG_PIN, US_MID_ECHO_PIN);
        init_sensor(US_RIGHT_TRIG_PIN, US_RIGHT_ECHO_PIN);

        // Reset de la estructura DistanceSensorData
        reset_system(data_ptr);

        // Dejar sensores inactivos por defecto
        *distance_state_ptr = INACTIVE;
    }


    void set_state(uint8_t mode, volatile uint8_t* distance_state_ptr) {
        *distance_state_ptr = (mode == ACTIVE) ? ACTIVE : INACTIVE;
    }


    void check_sensor_obstacle(
        uint8_t trig_pin, uint8_t echo_pin,
        volatile uint8_t* distance_ptr,
        volatile bool* sensor_obstacle_flag_ptr,
        volatile bool* global_obstacle_flag_ptr,
        volatile uint8_t* distance_state_ptr
    ) {
        if (*distance_state_ptr != ACTIVE) return;  // Solo leer si el sistema está activo
        uint8_t d = read_distance(trig_pin, echo_pin);
        if (d < OBSTACLE_THRESHOLD_CM) {
            // Repetir la lectura para evitar falsos positivos
            d = read_distance(trig_pin, echo_pin);
            *distance_ptr = d;
            bool obstacle_flag = (d < OBSTACLE_THRESHOLD_CM);
            *sensor_obstacle_flag_ptr = obstacle_flag;
            *global_obstacle_flag_ptr = obstacle_flag;
        } else {
            *distance_ptr = d;
            *sensor_obstacle_flag_ptr = false;
        }
    }


    void Task_CheckLeftObstacle(void* pvParameters) {
        vTaskDelay(pdMS_TO_TICKS(0));  // desfase inicial
        const TickType_t period = pdMS_TO_TICKS(US_SENSOR_READ_PERIOD_MS);
        TickType_t xLastWakeTime = xTaskGetTickCount();

        GlobalContext* ctx = static_cast<GlobalContext*>(pvParameters);
        for (;;) {
            vTaskDelayUntil(&xLastWakeTime, period);
            check_sensor_obstacle(
                US_LEFT_TRIG_PIN, US_LEFT_ECHO_PIN,
                &ctx->distance_ptr->us_left_distance,
                &ctx->distance_ptr->us_left_obstacle,
                &ctx->distance_ptr->obstacle_detected,
                &ctx->systems_ptr->distance
            );            
        }
    }

    void Task_CheckMidObstacle(void* pvParameters) {
        vTaskDelay(pdMS_TO_TICKS(50));  // desfase inicial
        const TickType_t period = pdMS_TO_TICKS(US_SENSOR_READ_PERIOD_MS);
        TickType_t xLastWakeTime = xTaskGetTickCount();

        GlobalContext* ctx = static_cast<GlobalContext*>(pvParameters);
        for (;;) {
            vTaskDelayUntil(&xLastWakeTime, period);
            check_sensor_obstacle(
                US_MID_TRIG_PIN, US_MID_ECHO_PIN,
                &ctx->distance_ptr->us_mid_distance,
                &ctx->distance_ptr->us_mid_obstacle,
                &ctx->distance_ptr->obstacle_detected,
                &ctx->systems_ptr->distance
            );
        }
    }

    void Task_CheckRightObstacle(void* pvParameters) {
        vTaskDelay(pdMS_TO_TICKS(100));  // desfase inicial
        const TickType_t period = pdMS_TO_TICKS(US_SENSOR_READ_PERIOD_MS);
        TickType_t xLastWakeTime = xTaskGetTickCount();

        GlobalContext* ctx = static_cast<GlobalContext*>(pvParameters);
        for (;;) {
            vTaskDelayUntil(&xLastWakeTime, period);
            check_sensor_obstacle(
                US_RIGHT_TRIG_PIN, US_RIGHT_ECHO_PIN,
                &ctx->distance_ptr->us_right_distance,
                &ctx->distance_ptr->us_right_obstacle,
                &ctx->distance_ptr->obstacle_detected,
                &ctx->systems_ptr->distance
            );            
        }
    }
    
    
}


        // bool ir_check_obstacle(uint8_t ir_pin) {
    //     bool current = digitalRead(ir_pin);
    //     unsigned long start_time = micros();
    //     // Debounce: esperar que el valor se mantenga estable
    //     while ((micros() - start_time) < IR_DEBOUNCE_TIME_US) {
    //         if (digitalRead(ir_pin) != current) {
    //             return 0; // rebote → ignorar lectura
    //         }
    //     }
    //     // Si la señal es LOW estable (active low), hay obstáculo
    //     return current == LOW ? true : false;
    // }
    

    // void ir_read_sensors(
    //     volatile bool* ir_left_obstacle_ptr,
    //     volatile bool* ir_right_obstacle_ptr,
    //     volatile uint8_t* distance_state_ptr
    // ) {
    //     if (*distance_state_ptr != ACTIVE) return;
    
    //     bool left_obstacle = ir_check_obstacle(IR_LEFT_SENSOR_PIN);
    //     bool right_obstacle = ir_check_obstacle(IR_RIGHT_SENSOR_PIN);
    
    //     *ir_left_obstacle_ptr = left_obstacle;
    //     *ir_right_obstacle_ptr = right_obstacle;
    // }


    // void Task_LateralObstacleDetect(void* pvParameters) {
    //     const TickType_t period = pdMS_TO_TICKS(IR_SENSOR_READ_PERIOD_MS);
    //     TickType_t xLastWakeTime = xTaskGetTickCount();
    
    //     GlobalContext* ctx_ptr = static_cast<GlobalContext*>(pvParameters);
    //     volatile uint8_t* distance_state_ptr     = &ctx_ptr->systems_ptr->distance;
    //     volatile bool* ir_left_obstacle_ptr      = &ctx_ptr->distance_ptr->ir_left_obstacle;
    //     volatile bool* ir_right_obstacle_ptr     = &ctx_ptr->distance_ptr->ir_right_obstacle;
    
    //     for (;;) {
    //         vTaskDelayUntil(&xLastWakeTime, period);
    //         ir_read_sensors(
    //             ir_left_obstacle_ptr,
    //             ir_right_obstacle_ptr,
    //             distance_state_ptr
    //         );
    //     }
    // } 