#include "project_config.h"
#include "sensors_firmware/distance_sensors.h"
#warning "Compilando main_us_sensor_basic.cpp"


    bool check_sensor_obstacle(
        const uint8_t trig_pin, const uint8_t echo_pin,
        volatile uint8_t& distance,
        volatile bool& sensor_obstacle_flag,
        volatile bool& global_obstacle_flag,
        volatile uint8_t& distance_state
    ) {
        if (distance_state != ACTIVE) return false;  // Solo leer si el sistema está activo
        uint8_t d = DistanceSensors::read_distance(trig_pin, echo_pin);
        bool obstacle_flag = (d < OBSTACLE_THRESHOLD_CM);
        sensor_obstacle_flag = obstacle_flag;
        if (obstacle_flag) global_obstacle_flag = true;
        distance = d;
        return obstacle_flag;
    }


// ====================== VARIABLES GLOBALES ======================
volatile uint8_t distance_state = INACTIVE;
volatile DistanceSensorData distance_data = {0};

// ====================== SETUP Y LOOP ======================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Test: Sensor Ultrasónico Central");
    DistanceSensors::init_system(distance_state, distance_data);
    DistanceSensors::set_state(ACTIVE, distance_state);
}

void loop() {
    // Ejecutar lectura de sensor izquierdo
    bool obstacle_now = DistanceSensors::check_sensor_obstacle(
        US_MID_TRIG_PIN, US_MID_ECHO_PIN,
        distance_data.us_mid_distance,
        distance_data.us_mid_obstacle,
        distance_data.obstacle_detected,
        distance_state
    );
    // uint8_t d = read_distance(trig_pin, echo_pin);

    // Mostrar resultados
    if (obstacle_now) {
        Serial.print("Obstáculo detectado a ");
        Serial.print(distance_data.us_mid_distance);
        Serial.println(" cm");
    } else {
        Serial.println("Área despejada.");
    }

    // // Pruebas
    // if (obstacle_now) Serial.println(distance_data.us_mid_distance);

    delay(500);  // Tiempo entre lecturas
}
