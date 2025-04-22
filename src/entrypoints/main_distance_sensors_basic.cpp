#include "project_config.h"
#include "sensors_firmware/distance_sensors.h"
#warning "Compilando main_distance_sensors_basic.cpp"

// ====================== VARIABLES GLOBALES ======================

volatile DistanceSensorData distance_data = {
    .obstacle_detected = false,
    .left_distance = US_MAX_DISTANCE_CM,
    .right_distance = US_MAX_DISTANCE_CM
};

// ====================== SETUP Y LOOP ======================

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Test: Sensor Ultrasónico Izquierdo");

    DistanceSensors::init();
}

void loop() {
    // Actualiza solo el sensor izquierdo
    uint8_t distancia_actual = DistanceSensors::us_read_distance(
        US_LEFT_TRIG_PIN, US_LEFT_ECHO_PIN
    );
    distance_data.left_distance = distancia_actual;

    // Verifica si hay obstáculo cerca (< 30 cm)
    //Serial.println(distancia_actual);
    if (distancia_actual < OBSTACLE_THRESHOLD_CM) {
        if (!distance_data.obstacle_detected) {
            distance_data.obstacle_detected = true;
            Serial.print("Obstáculo detectado a ");
            Serial.print(distancia_actual);
            Serial.println(" cm");
        } else {
            // Si sigue habiendo obstáculo, actualizar distancia y volver a mostrar
            Serial.print("Distancia obstáculo: ");
            Serial.print(distancia_actual);
            Serial.println(" cm");
        }
    } else {
        // Si antes había obstáculo y ahora no, lo notificamos
        if (distance_data.obstacle_detected) {
            distance_data.obstacle_detected = false;
            Serial.println("Obstáculo despejado.");
        }
    }
    delay(500);  // Tiempo entre lecturas
}
