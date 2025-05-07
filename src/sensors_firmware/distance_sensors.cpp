#include "distance_sensors.h"
//hola
namespace DistanceSensors {

    void init(volatile uint8_t* distance_state_ptr) {
        // IR
        pinMode(IR_LEFT_SENSOR_PIN, INPUT);
        pinMode(IR_RIGHT_SENSOR_PIN, INPUT);

        // US izquierda
        pinMode(US_LEFT_TRIG_PIN, OUTPUT);
        pinMode(US_LEFT_ECHO_PIN, INPUT);
        digitalWrite(US_LEFT_TRIG_PIN, LOW);

        // US derecha
        pinMode(US_RIGHT_TRIG_PIN, OUTPUT);
        pinMode(US_RIGHT_ECHO_PIN, INPUT);
        digitalWrite(US_RIGHT_TRIG_PIN, LOW);

        // Dejar sensores inactivos por defecto
        *distance_state_ptr = INACTIVE;
    }


    void set_state(uint8_t mode, volatile uint8_t* distance_state_ptr) {
        *distance_state_ptr = (mode == ACTIVE) ? ACTIVE : INACTIVE;
    }


    uint8_t us_read_distance(
        uint8_t trig_pin, uint8_t echo_pin
    ) {
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


    void us_check_obstacle(
        volatile bool* obstacle_detected_ptr,
        volatile uint8_t* us_left_distance_ptr, volatile uint8_t* us_right_distance_ptr,
        volatile uint8_t* distance_state_ptr
    ) {
        if (*distance_state_ptr != ACTIVE) return;
    
        // Leer sensor izquierdo
        uint8_t left_distance = us_read_distance(US_LEFT_TRIG_PIN, US_LEFT_ECHO_PIN);
        if (left_distance < OBSTACLE_THRESHOLD_CM) {
            left_distance = us_read_distance(US_LEFT_TRIG_PIN, US_LEFT_ECHO_PIN); // segunda lectura para debounce
            if (left_distance < OBSTACLE_THRESHOLD_CM) {
                *obstacle_detected_ptr = true;
                *us_left_distance_ptr = left_distance;
                *us_right_distance_ptr = US_MAX_DISTANCE_CM;  // No leído
                return;
            }
        }
        *us_left_distance_ptr = left_distance;
    
        // Leer sensor derecho solo si el izquierdo no detectó
        uint8_t right_distance = us_read_distance(US_RIGHT_TRIG_PIN, US_RIGHT_ECHO_PIN);
        if (right_distance < OBSTACLE_THRESHOLD_CM) {
            right_distance = us_read_distance(US_RIGHT_TRIG_PIN, US_RIGHT_ECHO_PIN);
            if (right_distance < OBSTACLE_THRESHOLD_CM) {
                *obstacle_detected_ptr = true;
                *us_right_distance_ptr = right_distance;
                return;
            }
        }
        // Si llegaste aquí, no se confirmó obstáculo en ningún sensor
        *us_right_distance_ptr = right_distance;
        *obstacle_detected_ptr = false; 
    }
    

    bool ir_check_obstacle(uint8_t ir_pin) {
        bool current = digitalRead(ir_pin);
        unsigned long start_time = micros();
        // Debounce: esperar que el valor se mantenga estable
        while ((micros() - start_time) < IR_DEBOUNCE_TIME_US) {
            if (digitalRead(ir_pin) != current) {
                return 0; // rebote → ignorar lectura
            }
        }
        // Si la señal es LOW estable (active low), hay obstáculo
        return current == LOW ? true : false;
    }
    

    void ir_read_sensors(
        volatile bool* ir_left_obstacle_ptr,
        volatile bool* ir_right_obstacle_ptr,
        volatile uint8_t* distance_state_ptr
    ) {
        if (*distance_state_ptr != ACTIVE) return;
    
        bool left_obstacle = ir_check_obstacle(IR_LEFT_SENSOR_PIN);
        bool right_obstacle = ir_check_obstacle(IR_RIGHT_SENSOR_PIN);
    
        *ir_left_obstacle_ptr = left_obstacle;
        *ir_right_obstacle_ptr = right_obstacle;
    }


    void Task_FrontObstacleDetect(void* pvParameters) {
        const TickType_t period = pdMS_TO_TICKS(US_SENSOR_READ_PERIOD_MS);
        TickType_t xLastWakeTime = xTaskGetTickCount();
    
        GlobalContext* ctx_ptr = static_cast<GlobalContext*>(pvParameters);
        volatile uint8_t* distance_state_ptr   = &ctx_ptr->systems_ptr->distance;
        volatile bool* obstacle_detected_ptr   = &ctx_ptr->distance_ptr->obstacle_detected;
        volatile uint8_t* left_distance_ptr    = &ctx_ptr->distance_ptr->us_left_distance;
        volatile uint8_t* right_distance_ptr   = &ctx_ptr->distance_ptr->us_right_distance;
    
        for (;;) {
            vTaskDelayUntil(&xLastWakeTime, period);
            us_check_obstacle(
                obstacle_detected_ptr,
                left_distance_ptr,
                right_distance_ptr,
                distance_state_ptr
            );
        }
    }
    

    void Task_LateralObstacleDetect(void* pvParameters) {
        const TickType_t period = pdMS_TO_TICKS(IR_SENSOR_READ_PERIOD_MS);
        TickType_t xLastWakeTime = xTaskGetTickCount();
    
        GlobalContext* ctx_ptr = static_cast<GlobalContext*>(pvParameters);
        volatile uint8_t* distance_state_ptr     = &ctx_ptr->systems_ptr->distance;
        volatile bool* ir_left_obstacle_ptr      = &ctx_ptr->distance_ptr->ir_left_obstacle;
        volatile bool* ir_right_obstacle_ptr     = &ctx_ptr->distance_ptr->ir_right_obstacle;
    
        for (;;) {
            vTaskDelayUntil(&xLastWakeTime, period);
            ir_read_sensors(
                ir_left_obstacle_ptr,
                ir_right_obstacle_ptr,
                distance_state_ptr
            );
        }
    } 
    
}
