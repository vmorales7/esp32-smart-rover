#include "encoder_reader.h"

namespace EncoderReader {

// Variables internas del sistema
static ESP32Encoder encoderLeft;
static ESP32Encoder encoderRight;

static long lastCountLeft = 0;
static long lastCountRight = 0;
static uint32_t lastMillis = 0;

// Variables para el filtro EMA 
static float filteredWL = 0.0f;
static float filteredWR = 0.0f;


void init(
    volatile float& phi_L, volatile float& phi_R, 
    volatile float& w_L, volatile float& w_R, 
    volatile uint8_t& encoder_state
) {
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    if constexpr (ENCODER_MODE == EncoderMode::HALF_QUAD) {
        encoderLeft.attachHalfQuad(ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN);
        encoderRight.attachHalfQuad(ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN);
    } else if constexpr (ENCODER_MODE == EncoderMode::FULL_QUAD) {
        encoderLeft.attachFullQuad(ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN);
        encoderRight.attachFullQuad(ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN);
    }

    // Dejar detenida la cuenta y reiniciarla
    encoderLeft.pauseCount();
    encoderRight.pauseCount();
    encoderLeft.clearCount();
    encoderRight.clearCount();

    // Se inicializan las variables de conteo y las memorias privadas
    phi_L = 0.0f;
    phi_R = 0.0f;
    lastCountLeft = 0;
    lastCountRight = 0;
    lastMillis = millis();

    // Todas las velocidades en cero
    filteredWL = 0.0f;
    filteredWR = 0.0f;
    w_L = 0.0f;
    w_R = 0.0f;

    // Actualizar el estado del encoder
    encoder_state = INACTIVE;
}


void update_encoder_data(
    volatile float& phi_L, volatile float& phi_R, 
    volatile float& w_L, volatile float& w_R, 
    const uint8_t encoder_state
) {
    if (encoder_state != ACTIVE) return; // Solo actualizar si estaba activo

    // Paso de tiempo
    const uint32_t currentMillis = millis();
    const float dt = (currentMillis - lastMillis) * MS_TO_S;  // Tiempo (s) transcurrido desde la ultima actualización
    if (dt < 0.001f) return; // si es menor a 1ms esperamos hasta la siguiente llamada

    // Lectura de la cuenta que se lleva internamente mediante ESP32Encoder
    int64_t currentCountLeft = encoderLeft.getCount();
    int64_t currentCountRight = encoderRight.getCount();

    // Modificación para corregir el sentido positivo hacia adelante
    if constexpr (INVERT_ENCODER_LEFT) currentCountLeft = -currentCountLeft;
    if constexpr (INVERT_ENCODER_RIGHT) currentCountRight = -currentCountRight;

    // Cambio en el giro desde la ultima lectura
    const float delta_L = (currentCountLeft - lastCountLeft) * RAD_PER_PULSE; 
    const float delta_R = (currentCountRight - lastCountRight) * RAD_PER_PULSE;

    // Se acumula el giro sobre el valor que se tenía antes
    phi_L += delta_L;
    phi_R += delta_R;

    // La velocidad es instantánea así que se actualiza con el factor para convertir en radianes
    const float rawWL = delta_L / dt;
    const float rawWR = delta_R / dt;

    // Se utiliza un filtro EMA (exponential moving average)
    if constexpr (USE_VELOCITY_FILTER) {
        filteredWL = EMA_ALPHA * rawWL + (1.0f - EMA_ALPHA) * filteredWL;
        filteredWR = EMA_ALPHA * rawWR + (1.0f - EMA_ALPHA) * filteredWR;
        w_L = filteredWL;
        w_R = filteredWR;
    } else {
        w_L = rawWL;
        w_R = rawWR;
    }
    
    // Guardar datos para la próxima iteración
    lastCountLeft = currentCountLeft;
    lastCountRight = currentCountRight;
    lastMillis = currentMillis;
}


void pause(
    volatile float& phi_L, volatile float& phi_R, 
    volatile float& w_L, volatile float& w_R,
    volatile uint8_t& encoder_state
) {
    if (encoder_state != ACTIVE) return; // Solo pausar si estaba activo antes

    // Detener el conteo
    encoderLeft.pauseCount();
    encoderRight.pauseCount();

    // Última actualización antes de pausar (para no perder pasos entre ticks)
    update_encoder_data(phi_L, phi_R, w_L, w_R, encoder_state);

    // Reset hardware y control interno 
    // Importante que no se limpien las variables phi_R/L ya que esto solo lo puede hacer pose_estimator
    encoderLeft.clearCount();
    encoderRight.clearCount();
    lastCountLeft = 0;
    lastCountRight = 0;

    // Dejar en cero las mediciones de velocidad
    filteredWL = 0.0f;
    filteredWR = 0.0f;
    w_L = 0.0f;
    w_R = 0.0f;

    // Actualizar el estado del encoder
    encoder_state = INACTIVE;
}


void resume(volatile uint8_t& encoder_state) {
    if (encoder_state != ACTIVE) { // Se modifica solo si no estaba activo ya
        lastMillis = millis();
        encoderLeft.resumeCount();
        encoderRight.resumeCount();
        encoder_state = ACTIVE;
    }
}


void Task_EncoderUpdate(void* pvParameters) {
    // Configuración RTOS
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(ENCODER_READ_PERIOD_MS);

    // Recuperar variables globales
    GlobalContext* ctx_ptr = static_cast<GlobalContext*>(pvParameters);
    volatile SystemStates& sts = *ctx_ptr->systems_ptr;
    volatile SensorsData& sens = *ctx_ptr->sensors_ptr;
    volatile PoseData& pose = *ctx_ptr->pose_ptr;

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        update_encoder_data(sens.enc_phiL, sens.enc_phiR, sens.enc_wL, sens.enc_wR, sts.encoders);
        if (pose.estimator_type == PoseEstimatorType::ENCODER) { 
            // En caso de que no se haya implementado el sensor fusion, se pasa directo al dato de pose
            pose.w_L = sens.enc_wL;
            pose.w_R = sens.enc_wR;
        }
    }
}
    
} // namespace EncoderReader
