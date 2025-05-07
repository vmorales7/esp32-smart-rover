#include "distance_sensors.h"

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

    uint8_t us_read_distance(
        uint8_t trig_pin, uint8_t echo_pin, 
        volatile uint8_t* distance_state_ptr
    ) {
        if (*distance_state_ptr != ACTIVE) return (uint8_t) US_MAX_DISTANCE_CM;

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
        uint8_t left_distance = us_read_distance(US_LEFT_TRIG_PIN, US_LEFT_ECHO_PIN, distance_state_ptr);
        if (left_distance < OBSTACLE_THRESHOLD_CM) {
            left_distance = us_read_distance(US_LEFT_TRIG_PIN, US_LEFT_ECHO_PIN, distance_state_ptr); // segunda lectura para debounce
            if (left_distance < OBSTACLE_THRESHOLD_CM) {
                *obstacle_detected_ptr = true;
                *us_left_distance_ptr = left_distance;
                *us_right_distance_ptr = US_MAX_DISTANCE_CM;  // No leído
                return;
            }
        }
        *us_left_distance_ptr = left_distance;
    
        // Leer sensor derecho solo si el izquierdo no detectó
        uint8_t right_distance = us_read_distance(US_RIGHT_TRIG_PIN, US_RIGHT_ECHO_PIN, distance_state_ptr);
        if (right_distance < OBSTACLE_THRESHOLD_CM) {
            right_distance = us_read_distance(US_RIGHT_TRIG_PIN, US_RIGHT_ECHO_PIN, distance_state_ptr);
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
    
    void update_ir_state(
        volatile bool* obstacle_detected_ptr,
        volatile uint8_t* distance_state_ptr
    ) {
        if (*distance_state_ptr != ACTIVE) return;

        const bool current = digitalRead(IR_LEFT_SENSOR_PIN);
        const unsigned long start_time = micros();
        
        // Se esperará suficiente tiempo para asegurar que la lectura sea limpia (debounce)
        while ((micros() - start_time) < IR_DEBOUNCE_TIME_US) {
            if (digitalRead(IR_LEFT_SENSOR_PIN) != current) return;  // rebote detectado
        }
        // Si no se detectó un rebote, podemos marcar la lectura
        if (*obstacle_detected_ptr != current) *obstacle_detected_ptr = current;
    }

    void clear_obstacle_detection(volatile DistanceSensorData* data) {
        data->obstacle_detected = false;
    }

    void set_state(uint8_t mode, volatile uint8_t* distance_state_ptr) {
        *distance_state_ptr = (mode == ACTIVE) ? ACTIVE : INACTIVE;
    }

    void Task_DistanceSensors(void* pvParameters) {
        const TickType_t period = pdMS_TO_TICKS(IR_SENSOR_READ_PERIOD_MS);
        TickType_t xLastWakeTime = xTaskGetTickCount();
    
        GlobalContext* ctx_ptr = static_cast<GlobalContext*>(pvParameters);
        volatile uint8_t* distance_state_ptr = &ctx_ptr->systems_ptr->distance;
        volatile bool* obstacle_detected_ptr = &ctx_ptr->distance_ptr->obstacle_detected;
        volatile uint8_t* left_distance_ptr = &ctx_ptr->distance_ptr->us_left_distance;
        volatile uint8_t* right_distance_ptr = &ctx_ptr->distance_ptr->us_right_distance;
    
        for (;;) {
            vTaskDelayUntil(&xLastWakeTime, period);
            update_ir_state(obstacle_detected_ptr, distance_state_ptr);
            update_ultrasonic_distances(distance_state_ptr, left_distance_ptr, right_distance_ptr); // si también deseas incluir esto
        }
    }
    
}
