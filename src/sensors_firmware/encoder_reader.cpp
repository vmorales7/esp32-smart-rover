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

    void init(
        volatile uint8_t* encoder_state_ptr,
        volatile int64_t* steps_left_ptr, volatile int64_t* steps_right_ptr,
        volatile float* wL_measured_ptr, volatile float* wR_measured_ptr
    ) {
        ESP32Encoder::useInternalWeakPullResistors = puType::up;
        if constexpr (ENCODER_MODE == EncoderMode::HALF_QUAD) {
            encoderLeft.attachHalfQuad(ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN);
            encoderRight.attachHalfQuad(ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN);
        } else if constexpr (ENCODER_MODE == EncoderMode::FULL_QUAD) {
            encoderLeft.attachFullQuad(ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN);
            encoderRight.attachFullQuad(ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN);
        }
        lastMillis = millis();
        pause(encoder_state_ptr, steps_left_ptr, steps_right_ptr, wL_measured_ptr, wR_measured_ptr);
        *steps_left_ptr = 0; // Única instancia en que encoder_reader modifica estos contadores
        *steps_right_ptr = 0;
    }


    void pause(
        volatile uint8_t* encoder_state_ptr,
        volatile int64_t* steps_left_ptr, volatile int64_t* steps_right_ptr,
        volatile float* wL_measured_ptr, volatile float* wR_measured_ptr
    ) {
        if (*encoder_state_ptr != ACTIVE) return; // Solo pausar si estaba activo antes

        // Detener el conteo
        encoderLeft.pauseCount();
        encoderRight.pauseCount();

        // Actualizar delta pendiente antes de pausar
        update_encoder_data(
            encoder_state_ptr, steps_left_ptr, steps_right_ptr, wL_measured_ptr, wR_measured_ptr
        );

        // Reset hardware y control interno 
        // Importante que no se limpien las variables steps_R/L_ptr ya que esto solo lo puede hacer pose_estimator
        encoderLeft.clearCount();
        encoderRight.clearCount();
        lastCountLeft = 0;
        lastCountRight = 0;
        lastMillis = millis();

        // Dejar en cero las mediciones de velocidad
        filteredWL = 0.0f;
        filteredWR = 0.0f;
        *wL_measured_ptr = 0.0f;
        *wR_measured_ptr = 0.0f;

        // Actualizar el estado del encoder
        *encoder_state_ptr = INACTIVE;
    }


    void resume(volatile uint8_t* encoder_state_ptr) {
        if (*encoder_state_ptr != ACTIVE) { // Se modifica solo si no estaba activo ya
            lastMillis = millis();
            encoderLeft.resumeCount();
            encoderRight.resumeCount();
            *encoder_state_ptr = ACTIVE;
        }
    }


    void update_encoder_data(
        volatile uint8_t* encoder_state_ptr,
        volatile int64_t* steps_left_ptr, volatile int64_t* steps_right_ptr,
        volatile float* wL_measured_ptr, volatile float* wR_measured_ptr
    ) {
        if (*encoder_state_ptr != ACTIVE) return; // Solo continuar si encoder activo

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
        int64_t deltaRight = (currentCountRight - lastCountRight) ;

        // Se acumula el giro sobre el valor que se tenía antes
        *steps_left_ptr += deltaLeft;
        *steps_right_ptr += deltaRight;

        // La velocidad es instantánea así que se actualiza con el factor para convertir en radianes
        float rawWL = deltaLeft * RAD_PER_PULSE / dt;
        float rawWR = deltaRight * RAD_PER_PULSE / dt;
    
        // Se utiliza un filtro EMA (exponential moving average)
        if constexpr (USE_VELOCITY_FILTER) {
            filteredWL = EMA_ALPHA * rawWL + (1.0f - EMA_ALPHA) * filteredWL;
            filteredWR = EMA_ALPHA * rawWR + (1.0f - EMA_ALPHA) * filteredWR;
            *wL_measured_ptr = filteredWL;
            *wR_measured_ptr = filteredWR;
        } else {
            *wL_measured_ptr = rawWL;
            *wR_measured_ptr = rawWR;
        }
        
        // Guardar datos para la próxima iteración
        lastCountLeft = currentCountLeft;
        lastCountRight = currentCountRight;
        lastMillis = currentMillis;
    }

    
    void Task_EncoderUpdate(void* pvParameters) {
        // Cosas de RTOS
        TickType_t xLastWakeTime = xTaskGetTickCount();
        const TickType_t period = pdMS_TO_TICKS(ENCODER_READ_PERIOD_MS);

        // Recuperamos los inputs pasados a la tarea de RTOS
        GlobalContext* ctx_ptr = static_cast<GlobalContext*>(pvParameters);
        volatile uint8_t* encoder_state_ptr = &ctx_ptr->systems_ptr->encoder;
        volatile int64_t* steps_left_ptr    = &ctx_ptr->wheels_ptr->steps_left;
        volatile int64_t* steps_right_ptr   = &ctx_ptr->wheels_ptr->steps_right;
        volatile float* wL_measured_ptr     = &ctx_ptr->wheels_ptr->wL_measured;
        volatile float* wR_measured_ptr     = &ctx_ptr->wheels_ptr->wR_measured;
        for (;;) {
            vTaskDelayUntil(&xLastWakeTime, period);
            update_encoder_data(
                encoder_state_ptr, steps_left_ptr, steps_right_ptr, wL_measured_ptr, wR_measured_ptr
            );
        }
    }
    
}
