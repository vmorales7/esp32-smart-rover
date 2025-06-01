#include "project_config.h"
#include "motor_drive/motor_controller.h"
#include "sensors_firmware/encoder_reader.h"
#include "sensors_firmware/distance_sensors.h"
#warning "Compilando main_avance1.cpp"

// ====================== VARIABLES GLOBALES ======================
volatile SystemStates sts;
volatile SensorsData sens;
volatile ControllerData ctrl;
constexpr uint16_t PRINT_PERIOD_MS = 500;

// ====================== FUNCIONES AUXILIARES ======================

void print_encoder_state() {
    Serial.print("Steps L/R: ");
    Serial.print(sens.enc_stepsL);
    Serial.print(" / ");
    Serial.print(sens.enc_stepsR);
    Serial.print(" | wL: ");
    Serial.print(sens.enc_wL, 3);
    Serial.print(" rad/s | wR: ");
    Serial.println(sens.enc_wR, 3);
}

void ejecutar_fase_con_obstaculo(const char* msg, float dutyL, float dutyR, uint32_t duracion_ms) {
    Serial.println(msg);
    
    uint32_t tiempo_acumulado = 0;
    uint32_t t_anterior = millis();
    uint32_t t_print = millis();

    bool en_movimiento = false;

    while (tiempo_acumulado < duracion_ms) {
        // Leer distancia
        uint8_t distancia = DistanceSensors::read_distance(US_MID_TRIG_PIN, US_MID_ECHO_PIN);
        sens.us_mid_dist = distancia;

        // Condición de obstáculo
        bool obstaculo = distancia < OBSTACLE_THRESHOLD_CM;
        sens.us_obstacle = obstaculo;

        if (obstaculo) {
            if (en_movimiento) {
                Serial.print("Obstáculo detectado a ");
                Serial.print(distancia);
                Serial.println(" cm — deteniendo motores");
                MotorController::set_motors_duty(0.0f, 0.0f, ctrl.duty_L, ctrl.duty_R, sts.motors);
                en_movimiento = false;
            }
        } else {
            if (!en_movimiento) {
                Serial.println("Obstáculo despejado — reanudando movimiento");
                MotorController::set_motors_duty(dutyL, dutyR, ctrl.duty_L, ctrl.duty_R, sts.motors);
                t_anterior = millis(); // reinicia referencia temporal tras pausa
                en_movimiento = true;
            }

            // Avance normal
            uint32_t t_actual = millis();
            tiempo_acumulado += (t_actual - t_anterior);
            t_anterior = t_actual;
        }
        EncoderReader::update_encoder_data(sens.enc_stepsL, sens.enc_stepsR, sens.enc_wL, sens.enc_wR, sts.encoders);
        if (millis() - t_print >= PRINT_PERIOD_MS) {
            print_encoder_state();
            t_print += PRINT_PERIOD_MS;
        }
        delay(10);
    }
    // Detener al final de la fase
    MotorController::set_motors_duty(0.0f, 0.0f, ctrl.duty_L, ctrl.duty_R, sts.motors);
}


// ====================== SETUP Y LOOP ======================

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Test: duty + encoder + obstacle");

    // Inicialización de motor y encoder
    MotorController::init(sts.motors, ctrl.duty_L, ctrl.duty_R);
    MotorController::set_motors_mode(MotorMode::ACTIVE, sts.motors, ctrl.duty_L, ctrl.duty_R);
    DistanceSensors::init(sens.us_left_dist, sens.us_left_obst, sens.us_mid_dist, sens.us_mid_obst, 
        sens.us_right_dist, sens.us_right_obst, sens.us_obstacle, sts.distance
    );  
    DistanceSensors::set_state(ACTIVE, sts.distance, sens.us_left_dist, sens.us_left_obst, 
        sens.us_mid_dist, sens.us_mid_obst, sens.us_right_dist, sens.us_right_obst, sens.us_obstacle
    );  
    EncoderReader::init(sens.enc_stepsL, sens.enc_stepsR, sens.enc_wL, sens.enc_wR, sts.encoders);
    EncoderReader::resume(sts.encoders);
    
    Serial.println();
    
    // Secuencia de prueba
    ejecutar_fase_con_obstaculo("Avanzando recto (50%)", 0.5f, 0.5f, 5000);
    delay(1000);
    ejecutar_fase_con_obstaculo("Girando en el lugar (izq, 60%)", 0.0f, 0.6f, 2000);
    delay(1000);
    ejecutar_fase_con_obstaculo("Avanzando recto (50%)", 0.5f, 0.5f, 5000);
    delay(1000);
    Serial.println("Secuencia completada. Motores en IDLE.");
    MotorController::set_motors_mode(MotorMode::IDLE, sts.motors, ctrl.duty_L, ctrl.duty_R);
}

void loop() {
    // Nada
}
