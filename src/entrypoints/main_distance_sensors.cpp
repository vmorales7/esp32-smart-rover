#include "project_config.h"
#include "sensors_firmware/distance_sensors.h"
#warning "Compilando main_distance_sensors.cpp"


// ====================== VARIABLES GLOBALES ======================

volatile DistanceSensorData sensor_data = {
    .obstacle_detected = false,
    .left_distance = 0,
    .right_distance = 0
};

// ====================== SETUP Y LOOP ======================

void setup() {
    Serial.begin(115200);
    DistanceSensors::init();
    Serial.println("Test sensores IR + US LEFT iniciado");
}

void loop() {
    delay(2000);  // Leer cada 2 segundos

    // Leer estado del sensor IR
    DistanceSensors::update_ir_state(&sensor_data);
    Serial.print("Sensor IR: ");
    Serial.println(sensor_data.obstacle_detected ? "OBSTÁCULO DETECTADO" : "LIBRE");

    // Leer sensor US izquierdo
    uint8_t dist = DistanceSensors::us_read_distance(US_LEFT_TRIG_PIN, US_LEFT_ECHO_PIN);

    Serial.print("Sensor US LEFT: ");
    if (dist == US_ERROR_VALUE) {
        Serial.println("ERROR (sin eco o timeout)");
    } else if (dist > US_MAX_DISTANCE_CM) {
        Serial.println("NO DETECTADO (fuera de rango)");
    } else {
        Serial.print(dist);
        Serial.println(" cm");
    }
    Serial.println("------------------------------");
}
