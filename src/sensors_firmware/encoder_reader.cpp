#include "encoder_reader.h"

namespace EncoderReader {

    // Variables internas del sistema
    static ESP32Encoder encoderLeft;
    static ESP32Encoder encoderRight;

    static long lastCountLeft = 0;
    static long lastCountRight = 0;
    static unsigned long lastMillis = 0;

    // Variables para el filtro EMA 
    static float filteredWL = 0.0f;
    static float filteredWR = 0.0f;

    void init(volatile WheelsData& wheels_data, volatile uint8_t& encoder_state) {
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
        wheels_data.steps_left = 0;
        wheels_data.steps_right = 0;
        lastCountLeft = 0;
        lastCountRight = 0;
        lastMillis = millis();

        // Todas las velocidades en cero
        filteredWL = 0.0f;
        filteredWR = 0.0f;
        wheels_data.wL_measured = 0.0f;
        wheels_data.wR_measured = 0.0f;

        // Actualizar el estado del encoder
        encoder_state = INACTIVE;
    }

    void update_encoder_data(volatile WheelsData& wheels_data, volatile uint8_t& encoder_state) {

        // Paso de tiempo
        unsigned long currentMillis = millis();
        float dt = (currentMillis - lastMillis) * MS_TO_S;  // Tiempo (s) transcurrido desde la ultima actualización
        if (dt < 0.001f) return; // si es menor a 1ms esperamos hasta la siguiente llamada

        // Lectura de la cuenta que se lleva internamente mediante ESP32Encoder
        int64_t currentCountLeft = encoderLeft.getCount();
        int64_t currentCountRight = encoderRight.getCount();

        // Modificación para corregir el sentido positivo hacia adelante
        if constexpr (INVERT_ENCODER_LEFT) currentCountLeft = -currentCountLeft;
        if constexpr (INVERT_ENCODER_RIGHT) currentCountRight = -currentCountRight;

        // Cambio en el giro desde la ultima lectura
        int64_t deltaLeft = (currentCountLeft - lastCountLeft); 
        int64_t deltaRight = (currentCountRight - lastCountRight);

        // Se acumula el giro sobre el valor que se tenía antes
        wheels_data.steps_left += deltaLeft;
        wheels_data.steps_right += deltaRight;

        // La velocidad es instantánea así que se actualiza con el factor para convertir en radianes
        float rawWL = deltaLeft * RAD_PER_PULSE / dt;
        float rawWR = deltaRight * RAD_PER_PULSE / dt;
    
        // Se utiliza un filtro EMA (exponential moving average)
        if constexpr (USE_VELOCITY_FILTER) {
            filteredWL = EMA_ALPHA * rawWL + (1.0f - EMA_ALPHA) * filteredWL;
            filteredWR = EMA_ALPHA * rawWR + (1.0f - EMA_ALPHA) * filteredWR;
            wheels_data.wL_measured = filteredWL;
            wheels_data.wR_measured = filteredWR;
        } else {
            wheels_data.wL_measured = rawWL;
            wheels_data.wR_measured = rawWR;
        }
        
        // Guardar datos para la próxima iteración
        lastCountLeft = currentCountLeft;
        lastCountRight = currentCountRight;
        lastMillis = currentMillis;
    }

    void pause(volatile WheelsData& wheels_data, volatile uint8_t& encoder_state) {
        if (encoder_state != ACTIVE) return; // Solo pausar si estaba activo antes

        // Detener el conteo
        encoderLeft.pauseCount();
        encoderRight.pauseCount();

        // Última actualización antes de pausar (para no perder pasos entre ticks)
        update_encoder_data(wheels_data, encoder_state);

        // Reset hardware y control interno 
        // Importante que no se limpien las variables steps_R/L_ptr ya que esto solo lo puede hacer pose_estimator
        encoderLeft.clearCount();
        encoderRight.clearCount();
        lastCountLeft = 0;
        lastCountRight = 0;

        // Dejar en cero las mediciones de velocidad
        filteredWL = 0.0f;
        filteredWR = 0.0f;
        wheels_data.wL_measured = 0.0f;
        wheels_data.wR_measured = 0.0f;

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
        // Cosas de RTOS
        TickType_t xLastWakeTime = xTaskGetTickCount();
        const TickType_t period = pdMS_TO_TICKS(ENCODER_READ_PERIOD_MS);

        // Recuperamos los inputs pasados a la tarea de RTOS
        GlobalContext* ctx_ptr = static_cast<GlobalContext*>(pvParameters);
        volatile uint8_t* encoder_state_ptr = &ctx_ptr->systems_ptr->encoders;
        volatile WheelsData* wheels_data_ptr = ctx_ptr->wheels_ptr;
        for (;;) {
            vTaskDelayUntil(&xLastWakeTime, period);
            // Solo continuar si encoder activo
            if (*encoder_state_ptr == ACTIVE) update_encoder_data(*wheels_data_ptr, *encoder_state_ptr); 
        }
    }
    
}
