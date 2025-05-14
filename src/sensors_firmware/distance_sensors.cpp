#include "distance_sensors.h"

namespace DistanceSensors {

    void init_sensor(const uint8_t trig_pin, const uint8_t echo_pin) {
        pinMode(trig_pin, OUTPUT);
        pinMode(echo_pin, INPUT);
        digitalWrite(trig_pin, LOW);
    }


    uint8_t read_distance(const uint8_t trig_pin, const uint8_t echo_pin) {
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


    void reset_system(volatile DistanceSensorData& distance_data, volatile uint8_t& distance_state) {
        // Reset de la estructura DistanceSensorData
        distance_data.obstacle_detected   = false;
        distance_data.us_left_obstacle    = false;
        distance_data.us_mid_obstacle     = false;
        distance_data.us_right_obstacle   = false;
        distance_data.us_left_distance    = US_MAX_DISTANCE_CM;
        distance_data.us_mid_distance     = US_MAX_DISTANCE_CM;
        distance_data.us_right_distance   = US_MAX_DISTANCE_CM;
        set_state(INACTIVE, distance_state);
    }

    
    void init_system(volatile uint8_t& distance_state, volatile DistanceSensorData& distance_data) {
        // Configuración de pines de sensores ultrasónicos
        init_sensor(US_LEFT_TRIG_PIN, US_LEFT_ECHO_PIN);
        init_sensor(US_MID_TRIG_PIN, US_MID_ECHO_PIN);
        init_sensor(US_RIGHT_TRIG_PIN, US_RIGHT_ECHO_PIN);

        // Reset de la estructura DistanceSensorData
        reset_system(distance_data, distance_state);
    }


    void set_state(const uint8_t new_mode, volatile uint8_t& distance_state) {
        distance_state = (new_mode == ACTIVE) ? ACTIVE : INACTIVE;
    }


    bool check_sensor_obstacle(
        const uint8_t trig_pin, const uint8_t echo_pin,
        volatile uint8_t& distance,
        volatile bool& sensor_obstacle_flag,
        volatile bool& global_obstacle_flag,
        volatile uint8_t& distance_state
    ) {
        if (distance_state != ACTIVE) return false;  // Solo leer si el sistema está activo
        uint8_t d = read_distance(trig_pin, echo_pin);
        // Serial.println(d);
        bool obstacle_flag = false;

        // Si parece haber obstáculo, confirmar con una segunda lectura
        if (d < OBSTACLE_THRESHOLD_CM) {
            // Repetir la lectura para evitar falsos positivos
            delay(10); // Necesitamos un delay mínimo para que funcione bien la segunda lectura
            d = read_distance(trig_pin, echo_pin);
            obstacle_flag = (d < OBSTACLE_THRESHOLD_CM);
            if (obstacle_flag) global_obstacle_flag = true;
        }
        sensor_obstacle_flag = obstacle_flag;
        distance = d;
        return obstacle_flag;
    }

    bool update_global_obstacle_flag(volatile DistanceSensorData& data) {
        data.obstacle_detected =
            data.us_left_obstacle ||
            data.us_mid_obstacle ||
            data.us_right_obstacle;
        return data.obstacle_detected;
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
                ctx->distance_ptr->us_left_distance,
                ctx->distance_ptr->us_left_obstacle,
                ctx->distance_ptr->obstacle_detected,
                ctx->systems_ptr->distance
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
                ctx->distance_ptr->us_mid_distance,
                ctx->distance_ptr->us_mid_obstacle,
                ctx->distance_ptr->obstacle_detected,
                ctx->systems_ptr->distance
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
                ctx->distance_ptr->us_right_distance,
                ctx->distance_ptr->us_right_obstacle,
                ctx->distance_ptr->obstacle_detected,
                ctx->systems_ptr->distance
            );            
        }
    }

    void Task_UpdateObstacleFlag(void* pvParameters) {
        TickType_t xLastWakeTime = xTaskGetTickCount();
        const TickType_t period = pdMS_TO_TICKS(25);

        GlobalContext* ctx = static_cast<GlobalContext*>(pvParameters);

        for (;;) {
            vTaskDelayUntil(&xLastWakeTime, period);
            DistanceSensors::update_global_obstacle_flag(*ctx->distance_ptr);
        }
    }
    
}
