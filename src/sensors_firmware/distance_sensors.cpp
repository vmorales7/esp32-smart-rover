#include "distance_sensors.h"

namespace DistanceSensors {

    void init() {
        // IR
        pinMode(IR_SENSOR_PIN, INPUT);

        // US izquierda
        pinMode(US_LEFT_TRIG_PIN, OUTPUT);
        pinMode(US_LEFT_ECHO_PIN, INPUT);
        digitalWrite(US_LEFT_TRIG_PIN, LOW);

        // US derecha
        pinMode(US_RIGHT_TRIG_PIN, OUTPUT);
        pinMode(US_RIGHT_ECHO_PIN, INPUT);
        digitalWrite(US_RIGHT_TRIG_PIN, LOW);
    }

    uint8_t us_read_distance(uint8_t trig_pin, uint8_t echo_pin) {
        digitalWrite(trig_pin, LOW);
        delayMicroseconds(2);
        digitalWrite(trig_pin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trig_pin, LOW);

        uint32_t duration = pulseIn(echo_pin, HIGH, US_PULSE_TIMEOUT_US);
        //Serial.print("Duración: "); Serial.println(duration);
        if (duration == 0) return (uint8_t) US_MAX_DISTANCE_CM;
        float distance_f = duration * US_CM_PER_US;
        return (uint8_t) distance_f;
    }

    void update_ultrasonic_distances(volatile DistanceSensorData* data) {
        data->left_distance = us_read_distance(US_LEFT_TRIG_PIN, US_LEFT_ECHO_PIN);
        data->right_distance = us_read_distance(US_RIGHT_TRIG_PIN, US_RIGHT_ECHO_PIN);
    }

    void update_ir_state(volatile DistanceSensorData* data) {
        const bool current = digitalRead(IR_SENSOR_PIN);
        const unsigned long start_time = micros();
        
        // Se esperará suficiente tiempo para asegurar que la lectura sea limpia (debounce)
        while ((micros() - start_time) < IR_DEBOUNCE_TIME_US) {
            if (digitalRead(IR_SENSOR_PIN) != current) return;  // rebote detectado
        }
        // Si no se detectó un rebote, podemos marcar la lectura
        if (data->obstacle_detected != current) data->obstacle_detected = current;
    }

    void clear_obstacle_detection(volatile DistanceSensorData* data) {
        data->obstacle_detected = false;
    }

    void Task_DistanceSensors(void* pvParameters) {
        const TickType_t period = pdMS_TO_TICKS(IR_SENSOR_READ_PERIOD_MS);
        TickType_t xLastWakeTime = xTaskGetTickCount();
        volatile DistanceSensorData* data_ptr = static_cast<DistanceSensorData*>(pvParameters);
        for (;;) {
            vTaskDelayUntil(&xLastWakeTime, period);
            update_ir_state(data_ptr);
        }
    }
}
