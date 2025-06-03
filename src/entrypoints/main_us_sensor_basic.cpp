#include "vehicle_os/general_config.h"
#include "sensors_firmware/distance_sensors.h"
#warning "Compilando main_us_sensor_basic.cpp"


// ====================== VARIABLES GLOBALES ======================
volatile uint8_t distance_state = INACTIVE;
volatile uint8_t left_dist = 0;
volatile bool left_obst = false;
volatile uint8_t mid_dist = 0;
volatile bool mid_obst = false;
volatile uint8_t right_dist = 0;
volatile bool right_obst = false;
volatile bool obstacle_detected = false;

// ====================== SETUP Y LOOP ======================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Test: Sensor Ultrasónico Central");
    DistanceSensors::init(
        left_dist, left_obst, mid_dist, mid_obst, right_dist, right_obst, obstacle_detected, distance_state);
    DistanceSensors::set_state(ACTIVE, distance_state, left_obst, mid_obst, right_obst, obstacle_detected);
}

void loop() {
    // Ejecutar lectura de sensor izquierdo
    bool obstacle_now = DistanceSensors::check_sensor_obstacle(
        US_MID_TRIG_PIN, US_MID_ECHO_PIN, mid_dist, mid_obst, obstacle_detected, distance_state);
    // uint8_t d = read_distance(trig_pin, echo_pin);

    // Mostrar resultados
    if (obstacle_now) {
        Serial.print("Obstáculo detectado a ");
        Serial.print(mid_dist);
        Serial.println(" cm");
    } else {
        Serial.println("Área despejada.");
    }

    // // Pruebas
    // if (obstacle_now) Serial.println(us_mid_distance);

    delay(500);  // Tiempo entre lecturas
}
