#include "distance_sensors.h"

namespace DistanceSensors {

static uint32_t last_obstacle_time = 0; // Tiempo del último obstáculo detectado

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


void reset_system(
    volatile uint8_t& left_dist, volatile bool& left_obst,
    volatile uint8_t& mid_dist, volatile bool& mid_obst,
    volatile uint8_t& right_dist, volatile bool& right_obst,
    volatile bool& obstacle
) {
    obstacle   = false;
    left_obst  = false;
    mid_obst   = false;
    right_obst = false;
    left_dist  = US_MAX_DISTANCE_CM;
    mid_dist   = US_MAX_DISTANCE_CM;
    right_dist = US_MAX_DISTANCE_CM;
}


void clear_flags(
    volatile bool& left_obst, volatile bool& mid_obst, volatile bool& right_obst, volatile bool& obstacle
) {
    obstacle   = false;
    left_obst  = false;
    mid_obst   = false;
    right_obst = false;
}


void init(
    volatile uint8_t& left_dist, volatile bool& left_obst,
    volatile uint8_t& mid_dist, volatile bool& mid_obst,
    volatile uint8_t& right_dist, volatile bool& right_obst,
    volatile bool& obstacle, volatile uint8_t& distance_state
) {
    // Configuración de pines de sensores ultrasónicos
    init_sensor(US_LEFT_TRIG_PIN, US_LEFT_ECHO_PIN);
    init_sensor(US_MID_TRIG_PIN, US_MID_ECHO_PIN);
    init_sensor(US_RIGHT_TRIG_PIN, US_RIGHT_ECHO_PIN);

    // Reset de la estructura DistanceSensorData
    reset_system(left_dist, left_obst, mid_dist, mid_obst, right_dist, right_obst, obstacle);
}


void set_state(
    const uint8_t new_mode, volatile uint8_t& distance_state,
    volatile bool& left_obst, volatile bool& mid_obst, volatile bool& right_obst,
    volatile bool& obstacle
) {
    if (new_mode == distance_state) return;
    distance_state = (new_mode == ACTIVE) ? ACTIVE : INACTIVE;
    if (distance_state == INACTIVE) {
        clear_flags(left_obst, mid_obst, right_obst, obstacle);
    }
}


bool check_sensor_obstacle(
    const uint8_t trig_pin, const uint8_t echo_pin,
    volatile uint8_t& distance,
    volatile bool& sensor_obstacle_flag,
    volatile bool& global_obstacle_flag,
    const uint8_t distance_state
) {
    if (distance_state != ACTIVE) return false;  // Solo leer si el sistema está activo
    uint8_t d = read_distance(trig_pin, echo_pin);
    // Serial.println(d);
    bool obstacle_flag = false;

    // Si parece haber obstáculo, confirmar con una segunda lectura
    if (d < OBSTACLE_THRESHOLD_CM) {
        // Repetir la lectura para evitar falsos positivos
        delay(US_WAIT_TIME_MS); // Necesitamos un delay mínimo para que funcione bien la segunda lectura
        d = read_distance(trig_pin, echo_pin);
        obstacle_flag = (d < OBSTACLE_THRESHOLD_CM);
        if (obstacle_flag) {
            global_obstacle_flag = true;
            last_obstacle_time = millis();
        }
    }
    sensor_obstacle_flag = obstacle_flag;
    distance = d;
    return obstacle_flag;
}


bool compute_global_obstacle_flag(
    const bool left_obst, const bool mid_obst, const bool right_obst
) {
    return left_obst || mid_obst || right_obst;
}


bool update_global_obstacle_flag(
    const bool left_obst, const bool mid_obst, const bool right_obst, volatile bool& obstacle
) { 
    const bool found = compute_global_obstacle_flag(left_obst, mid_obst, right_obst);
    const uint32_t now = millis();
    if (!found) {
        // Solo permitir liberar la bandera si pasó suficiente tiempo sin obstáculo
        if (obstacle && (now - last_obstacle_time > OBSTACLE_DEBOUNCE_MS)) {
            obstacle = false;
        }
    }
    return obstacle;
}


void Task_CheckObstacle(void* pvParameters) {
    // Configuración inicial de la tarea
    vTaskDelay(pdMS_TO_TICKS(7));  // desfase inicial
    const TickType_t period = pdMS_TO_TICKS(OBSTACLE_CHECK_PERIOD_MS);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // Obtener el contexto global
    GlobalContext* ctx = static_cast<GlobalContext*>(pvParameters);
    volatile SystemStates& sts = *ctx->systems_ptr;
    volatile SensorsData& sens = *ctx->sensors_ptr;

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period); // Sincroniza el ciclo completo
        // Sensor IZQUIERDO
        check_sensor_obstacle(
            US_LEFT_TRIG_PIN, US_LEFT_ECHO_PIN,
            sens.us_left_dist, sens.us_left_obst, sens.us_obstacle, sts.distance
        );
        vTaskDelay(pdMS_TO_TICKS(US_WAIT_TIME_MS)); // Pequeña pausa entre sensores
        // Sensor CENTRAL
        check_sensor_obstacle(
            US_MID_TRIG_PIN, US_MID_ECHO_PIN,
            sens.us_mid_dist, sens.us_mid_obst, sens.us_obstacle, sts.distance
        );
        vTaskDelay(pdMS_TO_TICKS(US_WAIT_TIME_MS)); // Pequeña pausa entre sensores
        // Sensor DERECHO
        check_sensor_obstacle(
            US_RIGHT_TRIG_PIN, US_RIGHT_ECHO_PIN,
            sens.us_right_dist, sens.us_right_obst, sens.us_obstacle, sts.distance
        );
    }
}


bool force_check_all_sensors(GlobalContext* ctx) {
    volatile SystemStates& sts = *ctx->systems_ptr;
    volatile SensorsData& sens = *ctx->sensors_ptr;
    volatile TaskHandlers& handlers = *ctx->rtos_task_ptr;

    // 1. Suspender tareas de sensores (para acceso exclusivo a pines) 
    vTaskSuspend(handlers.obstacle_handle);

    // 2. Espera para estabilizar hardware
    vTaskDelay(pdMS_TO_TICKS(US_WAIT_TIME_MS + US_PULSE_TIMEOUT_US / 1000.0f));

    // 3. Guardar el estado previo de los sensores
    const uint8_t prev_state = sts.distance;

    // 4. Habilitar temporalmente si estaba desactivado
    set_state(ACTIVE, sts.distance, sens.us_left_obst, sens.us_mid_obst, sens.us_right_obst, sens.us_obstacle);

    // 5. Limpiar bandera global si corresponde (puede quedar en true por debounce)
    update_global_obstacle_flag(
        sens.us_left_obst, sens.us_mid_obst, sens.us_right_obst, sens.us_obstacle);

    // 6. Se revisan los sensores, si alguno detecta obstáculo la flag global de obstáculo es true
    check_sensor_obstacle(
        US_MID_TRIG_PIN, US_MID_ECHO_PIN, sens.us_mid_dist, sens.us_mid_obst, sens.us_obstacle, sts.distance);
    vTaskDelay(pdMS_TO_TICKS(US_WAIT_TIME_MS)); 
    check_sensor_obstacle(
        US_LEFT_TRIG_PIN, US_LEFT_ECHO_PIN, sens.us_left_dist, sens.us_left_obst, sens.us_obstacle, sts.distance);
    vTaskDelay(pdMS_TO_TICKS(US_WAIT_TIME_MS)); 
    check_sensor_obstacle(
        US_RIGHT_TRIG_PIN, US_RIGHT_ECHO_PIN, sens.us_right_dist, sens.us_right_obst, sens.us_obstacle, sts.distance);
    vTaskDelay(pdMS_TO_TICKS(US_WAIT_TIME_MS)); 

    // 7. Bandera global actualizada
    const bool obstaculo = sens.us_obstacle;

    // 8. Volver al estado original si era INACTIVE
    if (prev_state == INACTIVE) {
        set_state(INACTIVE, sts.distance, sens.us_left_obst, sens.us_mid_obst, sens.us_right_obst, sens.us_obstacle);
    }

    // 9. Reanudar tareas periódicas
    vTaskResume(handlers.obstacle_handle);

    return obstaculo;
}

} // namespace DistanceSensors
