#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include "project_config.h"

/* ---------------- Constantes y variables sistema ------------------*/

constexpr float ANGLE_NAVIGATION_TOLERANCE = 30.0f * (2*PI / 180.0);
constexpr float ANGLE_ROTATION_TOLERANCE = 3.0f * (2*PI / 180.0);
constexpr float DISTANCE_TOLERANCE = 0.05f; // 5 cm

// Ganancias del PID de alfa
constexpr float KP_ALPHA = 3.0f;  // Ganancia proporcional
constexpr float KI_ALPHA = 0.2f;  // Ganancia integral
constexpr float KD_ALPHA = 0.0f;  // Ganancia derivativa
constexpr float KW_ALPHA = 0.1f / KI_ALPHA; // Ganancia anti-windup

// Ganancias de PI de rho
constexpr float KP_RHO = 0.8f;  // Ganancia proporcional (0.7)
constexpr float KI_RHO = 0.1f;  // Ganancia integral (0.1)
constexpr float KW_RHO = 0.1f / KI_RHO; // Ganancia anti-windup

// Parámetros del controlador tipo backstepping
constexpr float K1 = 2.0f; 
constexpr float K2 = 3.0f; 
constexpr float K3 = 0.5f; 

// Estructura de datos para la velocidad deseada y referencias de ruedas
struct VelocityData {
    float v;    // Velocidad lineal deseada [m/s]
    float w;    // Velocidad angular deseada [rad/s]
    float wL;   // Velocidad angular rueda izquierda [rad/s]
    float wR;   // Velocidad angular rueda derecha [rad/s]
};

constexpr float MIN_POS_DT = 0.001f; // Tiempo mínimo entre actualizaciones del controlador de posición

enum MovingState : uint8_t {
    KIN_STOPPING = 0U, // Vehículo en proceso de detenerse
    KIN_REACHED,       // Vehículo detenido
    KIN_ALIGNING,      // Vehículo alineando hacia el objetivo
    KIN_MOVING         // Vehículo alineando hacia el objetivo
};


/* ---------------- Funciones del sistema ------------------*/

namespace PositionController {

    /**
     * @brief Inicializa el controlador de posición.
     *
     * Establece todas las referencias (v_ref, w_ref, wL_ref, wR_ref) en cero
     * y configura el modo del controlador como SPEED_REF_INACTIVE.
     *
     * @param w_L_ref Variable con referencia de rueda izquierda [rad/s]
     * @param w_R_ref Variable con referencia de rueda derecha [rad/s]
     * @param control_mode Variable con modo actual del controlador de posición
     */
    void init(
        volatile PositionControlMode& control_mode,
        volatile float& w_L_ref, volatile float& w_R_ref
    );

    /**
     * @brief Cambia el modo de operación del controlador de posición. 
     * Deja las referecias de velocidad en cero al ser llamada.
     *
     * @param new_mode Nuevo modo deseado (SPEED_REF_INACTIVE, SPEED_REF_MANUAL, SPEED_REF_AUTO)
     * @param control_mode Variable con estado del controlador de posición
     * @param w_L_ref Variable con referencia de rueda izquierda [rad/s]
     * @param w_R_ref Variable con referencia de rueda derecha [rad/s]
     */
    void set_control_mode(
        const PositionControlMode new_mode,
        volatile PositionControlMode& control_mode,
        volatile float& w_L_ref, volatile float& w_R_ref
    );

    /**
     * @brief Asigna referencias de velocidad angular a cada rueda, saturando en los límites técnicos.
     * Si el modo del controlador está en SPEED_REF_INACTIVE, no se hace nada.
     *
     * @param w_L Valor deseado para rueda izquierda [rad/s]
     * @param w_R Valor deseado para rueda derecha [rad/s]
     * @param w_L_ref_global Variable con referencia izquierda
     * @param w_R_ref_global Variable con referencia derecha
     * @param control_mode Variable con modo del controlador
     */
    void set_wheel_speed_ref(
        const float w_L, const float w_R,
        volatile float& w_L_ref_global, volatile float& w_R_ref_global,
        volatile PositionControlMode& control_mode
    );

    /**
     * @brief Controlador de posición básico: navega hacia un punto (x_d, y_d) usando control en (ρ, α).
     * 
     * @param x Posición actual en X [m].
     * @param y Posición actual en Y [m].
     * @param theta Orientación actual [rad].
     * @param x_d Coordenada X del objetivo [m].
     * @param y_d Coordenada Y del objetivo [m].
     * @param wL_ref Referencia de velocidad angular para la rueda izquierda [rad/s].
     * @param wR_ref Referencia de velocidad angular para la rueda derecha [rad/s].
     * @param control_mode Modo actual del controlador (debe ser SPEED_REF_AUTO_BASIC).
     */
    uint8_t update_control_pid(
        const float x, const float y, const float theta,
        const float x_d, const float y_d, const float theta_d,
        volatile float& wL_ref, volatile float& wR_ref,
        volatile PositionControlMode& control_mode
    );

    /**
     * @brief Controlador avanzado: utiliza errores (e1, e2, e3) en marco del vehículo para navegar al objetivo.
     * 
     * @param x Posición actual en X [m].
     * @param y Posición actual en Y [m].
     * @param theta Orientación actual [rad].
     * @param x_d Coordenada X del objetivo [m].
     * @param y_d Coordenada Y del objetivo [m].
     * @param wL_ref Referencia de velocidad angular para la rueda izquierda [rad/s].
     * @param wR_ref Referencia de velocidad angular para la rueda derecha [rad/s].
     * @param control_mode Modo actual del controlador (debe ser SPEED_REF_AUTO_ADVANCED).
     */
    uint8_t update_control_backstepping(
        const float x, const float y, const float theta,
        const float x_d, const float y_d, const float theta_d,
        volatile float& wL_ref, volatile float& wR_ref,
        volatile PositionControlMode& control_mode
    );

    /**
     * @brief Calcula la velocidad angular de referencia de una rueda,
     *        a partir de v_ref y w_ref.
     *
     * @param v_ref Velocidad lineal de referencia global [m/s]
     * @param w_ref Velocidad angular de referencia global [rad/s]
     * @param wheel_id WHEEL_LEFT (0) o WHEEL_RIGHT (1)
     * @return Velocidad angular de referencia para la rueda [rad/s]
     */
    float compute_wheel_speed(
        const float v_ref, 
        const float w_ref, 
        const uint8_t wheel_id
    );

    /**
     * @brief Actualiza las referencias de velocidad angular de las ruedas según el estado actual y el objetivo, 
     *        considerando el modo de control activo (PID/Backstepping, desplazamiento o giro).
     *
     * Esta función selecciona y ejecuta el controlador de posición adecuado (PID o Backstepping) según el modo solicitado.
     * Retorna true únicamente si el objetivo ha sido alcanzado y el vehículo está efectivamente detenido 
     * (tanto la velocidad lineal como la angular están por debajo de los umbrales mínimos).
     *
     * @param x             Posición actual en X [m].
     * @param y             Posición actual en Y [m].
     * @param theta         Orientación actual [rad].
     * @param x_d           Coordenada X del objetivo [m].
     * @param y_d           Coordenada Y del objetivo [m].
     * @param theta_d       Orientación objetivo [rad] (utilizada en modos de giro).
     * @param v             Velocidad lineal estimada del vehículo [m/s].
     * @param w             Velocidad angular estimada del vehículo [rad/s].
     * @param wL_ref        Referencia de velocidad angular para la rueda izquierda [rad/s].
     * @param wR_ref        Referencia de velocidad angular para la rueda derecha [rad/s].
     * @param moving_state  Estado de movimiento actual del vehículo (STOPPING, REACHED, ALIGNING, MOVING).
     * @param control_mode  Modo de control activo (MOVE_PID, TURN_PID, MOVE_BACKS, TURN_BACKS, etc.).
     * @return              Retorna true si se alcanzó el objetivo y el vehículo está detenido
     */
    bool update_control(
        const float x, const float y, const float theta,
        const float x_d, const float y_d, const float theta_d,
        volatile float& v, volatile float& w,
        volatile float& wL_ref, volatile float& wR_ref,
        volatile uint8_t& moving_state,
        volatile PositionControlMode& control_mode
    );

    /**
     * @brief Detiene el movimiento del vehículo fijando las referencias de velocidad de rueda a cero,
     *        y cambia el modo de control de posición a manual.
     *
     * Esta función se utiliza para forzar el vehículo a detenerse inmediatamente, fijando las referencias
     * de velocidad de rueda a cero en modo manual. Posteriormente, verifica si tanto la velocidad lineal
     * como la angular estimadas están por debajo de los umbrales mínimos configurados para considerar
     * que el vehículo está efectivamente detenido.
     *
     * @param v              Velocidad lineal estimada del vehículo [m/s].
     * @param w              Velocidad angular estimada del vehículo [rad/s].
     * @param w_L_ref        Referencia de velocidad angular para la rueda izquierda [rad/s].
     * @param w_R_ref        Referencia de velocidad angular para la rueda derecha [rad/s].
     * @param moving_state   Estado de movimiento actual del vehículo (se ajusta a KIN_STOPPING en la función).
     * @param control_mode   Modo actual de control de posición (se ajusta a MANUAL en la función).
     * @return true si ambas velocidades están bajo los umbrales de detención; false en caso contrario.
     */
    bool stop_movement(
        volatile float& v, volatile float& w,
        volatile float& w_L_ref, volatile float& w_R_ref,
        volatile uint8_t& moving_state,
        volatile PositionControlMode& control_mode
    );

    /**
     * @brief Escala proporcionalmente las referencias de velocidad lineal y angular para asegurar que ninguna rueda supere su velocidad máxima.
     *
     * Dado un par de referencias (v, w), calcula las velocidades angulares de cada rueda
     * y, si alguna excede el límite físico (`WM_NOM`), escala ambas referencias proporcionalmente
     * para garantizar que el límite no se supere. Devuelve una estructura con los valores ajustados
     * y las velocidades angulares finales de cada rueda.
     *
     * @param v_raw Velocidad lineal de referencia antes de la saturación [m/s].
     * @param w_raw Velocidad angular de referencia antes de la saturación [rad/s].
     * @return VelocityData Estructura con los valores finales de (v, w, wL, wR) tras la saturación.
     */
    VelocityData constrain_velocity(float v_raw, float w_raw);

    /**
     * @brief Actualiza el estado de movimiento global del sistema y guarda el último estado.
     *
     * Esta función se encarga de modificar la variable global de estado de movimiento (por ejemplo, `KIN_MOVING`, `KIN_STOPPING`, etc.)
     * y sincroniza además la variable estática `last_moving_state` para mantener un historial consistente. 
     * Se debe utilizar SIEMPRE que se cambie el estado de movimiento del vehículo desde cualquier controlador.
     *
     * @param new_state Estado de movimiento a asignar (de tipo MovingState).
     * @param moving_state Referencia a la variable global de estado de movimiento a actualizar.
     */
    void set_moving_state(const uint8_t new_state, volatile uint8_t& moving_state);

    /**
     * @brief Tarea RTOS que ejecuta periódicamente el controlador de posición,
     *        seleccionando el modo de control (básico o avanzado) y actualizando las referencias de rueda.
     * 
     * @param pvParameters Puntero a un GlobalContext con referencias al estado del sistema.
     */
    void Task_PositionControl(void* pvParameters);

    /**
     * @brief Normaliza cualquier ángulo al rango (-π, π].
     * @param angle Ángulo en radianes.
     * @return Ángulo normalizado en (-π, π].
     */
    float wrap_to_pi(float angle);
}

#endif // POSITION_CONTROLLER_H
