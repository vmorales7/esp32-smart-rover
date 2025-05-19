#include "project_config.h"
#include "sensors_firmware/distance_sensors.h"
#warning "Compilando main_us_sensor_basic.cpp"


// ====================== VARIABLES GLOBALES ======================
volatile uint8_t distance_state = INACTIVE;
volatile DistanceSensorData distances = {0};

// ====================== SETUP Y LOOP ======================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Test: Sensor Ultrasónico Central");
    DistanceSensors::init(
        distances.left_dist, distances.left_obst, 
        distances.mid_dist, distances.mid_obst, 
        distances.right_dist, distances.right_obst, 
        distances.obstacle_detected, distance_state
    );
    DistanceSensors::set_state(
        ACTIVE, distance_state,
        distances.left_dist, distances.left_obst,
        distances.mid_dist, distances.mid_obst,
        distances.right_dist, distances.right_obst,
        distances.obstacle_detected
    );
}

void loop() {
    // Ejecutar lectura de sensor izquierdo
    bool obstacle_now = DistanceSensors::check_sensor_obstacle(
        US_MID_TRIG_PIN, US_MID_ECHO_PIN,
        distances.mid_dist,
        distances.mid_obst,
        distances.obstacle_detected,
        distance_state
    );

    // Mostrar resultados
    if (obstacle_now) {
        Serial.print("Obstáculo detectado a ");
        Serial.print(distances.mid_dist);
        Serial.println(" cm");
    } else {
        Serial.println("Área despejada.");
    }

    // // Pruebas
    // if (obstacle_now) Serial.println(distances.us_mid_distance);

    delay(500);  // Tiempo entre lecturas
}
