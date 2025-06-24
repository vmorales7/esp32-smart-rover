#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include "vehicle_os/general_config.h"

/* ---------------- Constantes y variables sistema ------------------*/

// Tolerancias para control de posición: evitar oscilaciones
constexpr float DISTANCE_TOLERANCE = 0.07f;                 // 5 cm de tolerancia
constexpr float ANGLE_TOLERANCE = 2.0f * DEG_TO_RAD;        // 3 grados en radianes
constexpr float MAX_ANGLE_DEVIATION = 30.0f * DEG_TO_RAD;   // 30 grados en radianes

// Thresholds para considerar que el vehículo está detenido
constexpr float V_MAX = WM_NOM * WHEEL_RADIUS;
constexpr float W_MAX = WM_NOM * WHEEL_RADIUS / WHEEL_TO_MID_DISTANCE;
constexpr float V_STOP_THRESHOLD = 0.01 * V_MAX;   // 10 cm en 30 segundos
constexpr float W_STOP_THRESHOLD = 0.02 * W_MAX;   // 1 vuelta en 30 segundos

// Ganancias del PID de alfa
constexpr float KP_ALPHA = 2.0f;  // Ganancia proporcional
constexpr float KI_ALPHA = 0.5f;  // Ganancia integral
constexpr float KD_ALPHA = 0.0f;  // Ganancia derivativa
constexpr float KW_ALPHA = 0.2f / KI_ALPHA; // Ganancia anti-windup
constexpr float INTEGRAL_ALPHA_MAX = 0.5 * V_MAX / KI_ALPHA; // Clamping del integrador

// Ganancias de PI de rho
constexpr float KP_RHO = 0.7f;  // Ganancia proporcional (0.7)
constexpr float KI_RHO = 0.1f;  // Ganancia integral (0.1)
constexpr float KW_RHO = 0.1f / KI_RHO;  // Ganancia anti-windup (0.1/K_I_RHO)
constexpr float INTEGRAL_RHO_MAX = 0.5 * V_MAX / KI_RHO; // Clamping del integrador (0.5 * V_MAX / KI_RHO)

// Parámetros del controlador tipo backstepping
constexpr float K1 = 2.5f; 
constexpr float K2 = 3.0f; //3.0
constexpr float K3 = 0.1f; // 0.5

// Estructura de datos para poder tener múltiples return
struct VelocityData {
    float v;    // Velocidad lineal deseada [m/s]
    float w;    // Velocidad angular deseada [rad/s]
    float wL;   // Velocidad angular rueda izquierda [rad/s]
    float wR;   // Velocidad angular rueda derecha [rad/s]
};

// Enum auxiliar para el modo de movimiento del vehículo
enum class MovingState : uint8_t {
    IDLE,
    STOPPED,  // Vehículo detenido
    ROTATING, // Vehículo alineando hacia el objetivo
    ADVANCING, // Vehículo alineando hacia el objetivo
    STOPPING // Vehículo en proceso de detenerse
};

/* ---------------- Funciones del sistema ------------------*/

namespace PositionController {

/**
 * @brief Inicializa el controlador de posición.
 *
 * Establece todas las referencias (v_ref, w_ref, wL_ref, wR_ref) en cero
 * y configura el modo del controlador como SPEED_REF_INACTIVE.
 *
 * @param control_mode Variable con modo actual del controlador de posición
 * @param x_d_global Referencia global de coordenada X del waypoint [m]
 * @param y_d_global Referencia global de coordenada Y del waypoint [m]
 * @param theta_d_global Referencia global de orientación del waypoint [rad]
 * @param waypoint_reached Referencia a bandera que indica si se ha alcanzado un waypoint
 * @param w_L_ref Variable con referencia de rueda izquierda [rad/s]
 * @param w_R_ref Variable con referencia de rueda derecha [rad/s]
 */
void init(
    volatile PositionControlMode& control_mode,
    volatile float& x_d_global, volatile float& y_d_global, volatile float& theta_d_global,
    volatile bool& waypoint_reached,
    volatile float& w_L_ref, volatile float& w_R_ref
);

/**
 * @brief Cambia el modo de operación del controlador de posición. 
 *
 * @param new_mode Nuevo modo deseado (SPEED_REF_INACTIVE, SPEED_REF_MANUAL, SPEED_REF_AUTO)
 * @param control_mode Variable con estado del controlador de posición
 * @param wL_ref Variable con referencia de velocidad angular para la rueda izquierda [rad/s]
 * @param wR_ref Variable con referencia de velocidad angular para la rueda derecha [rad/s]
 * @return true si se cambió el modo correctamente, false si ya estaba en ese modo.
 */
bool set_control_mode(
    const PositionControlMode new_mode,
    volatile PositionControlMode& control_mode,
    volatile float& wL_ref, volatile float& wR_ref
);

/**
 * @brief Cambia el tipo de controlador utilizado para la posición (PID o BACKS).
 *
 * @param new_type Nuevo tipo de controlador deseado (PID, BACKS)
 * @param controller_type Variable con estado del controlador de posición
 * @return true si se cambió el tipo correctamente, false si ya estaba en ese tipo.
 */
bool set_controller_type(
    const ControlType new_type,
    volatile ControlType& controller_type
);

/**
 * @brief Establece un waypoint (punto objetivo) en coordenadas globales.
 * 
 * Esta función actualiza las coordenadas del waypoint y reinicia el estado del PID, 
 * y reinicia la bandera de waypoint alcanzado.
 * 
 * @param x_d Coordenada X del waypoint [m].
 * @param y_d Coordenada Y del waypoint [m].
 * @param theta_d Orientación del waypoint [rad].
 * @param x_d_global Referencia global de coordenada X del waypoint [m].
 * @param y_d_global Referencia global de coordenada Y del waypoint [m].
 * @param theta_d_global Referencia global de orientación del waypoint [rad].
 * @param waypoint_reached Referencia a bandera que indica si se ha alcanzado el waypoint.
 * @param control_mode Referencia al modo actual del controlador (debe ser MOVE o ROTATE).
 */
bool set_waypoint(
    const float x_d, const float y_d, const float theta_d,
    volatile float& x_d_global, volatile float& y_d_global, volatile float& theta_d_global,
    volatile bool& waypoint_reached,
    volatile PositionControlMode& control_mode
);

/** 
 * @brief Establece un waypoint diferencial basado en la distancia y el cambio de orientación.
 * 
 * @param dist Distancia al waypoint diferencial [m].
 * @param delta_theta Cambio de orientación al waypoint diferencial [rad].
 * @param x_d_global Referencia global de coordenada X del waypoint [m].
 * @param y_d_global Referencia global de coordenada Y del waypoint [m].
 * @param theta_d_global Referencia global de orientación del waypoint [rad].
 * @param waypoint_reached Referencia a bandera que indica si se ha alcanzado el waypoint.
 * @param control_mode Modo actual del controlador (debe ser MOVE o ROTATE).
 */
bool set_diferential_waypoint(
    const float dist, const float delta_theta,
    volatile float& x_d_global, volatile float& y_d_global, volatile float& theta_d_global,
    volatile bool& waypoint_reached,
    const PositionControlMode control_mode
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
    const PositionControlMode control_mode
);

/**
 * @brief Controlador de posición básico: navega hacia un punto (x_d, y_d) usando control en (ρ, α).
 * 
 * @param x Posición actual en X [m].
 * @param y Posición actual en Y [m].
 * @param theta Orientación actual [rad].
 * @param x_d Coordenada X del objetivo [m].
 * @param y_d Coordenada Y del objetivo [m].
 * @param theta_d Orientación objetivo [rad].
 * @param wL_ref Referencia de velocidad angular para la rueda izquierda [rad/s].
 * @param wR_ref Referencia de velocidad angular para la rueda derecha [rad/s].
 * @param control_mode Modo actual del controlador (debe ser MOVE o ROTATE).
 * @return Estado de movimiento actualizado (STOPPING, REACHED, ALIGNING, MOVING).
 */
MovingState update_control_pid(
    const float x, const float y, const float theta,
    const float x_d, const float y_d, const float theta_d,
    volatile float& wL_ref, volatile float& wR_ref,
    const PositionControlMode control_mode
);

/**
 * @brief Reinicia el estado del PID, incluyendo las integrales y el último error.
 *
 * Esta función se utiliza para reiniciar los estados internos del controlador PID,
 * asegurando que no haya acumulación de errores previos que puedan afectar el control actual.
 */
void reset_pid_state();

/**
 * @brief Reinicia el estado del Backstepping, incluyendo toos los errores.
 *
 * Esta función se utiliza para reiniciar los estados internos del controlador Backstepping,
 * asegurando que no haya acumulación de errores previos que puedan afectar el control actual.
 */
void reset_backs();

/**
 * @brief Controlador avanzado: utiliza errores (e1, e2, e3) en marco del vehículo para navegar al objetivo.
 * 
 * @param x Posición actual en X [m].
 * @param y Posición actual en Y [m].
 * @param theta Orientación actual [rad].
 * @param x_d Coordenada X del objetivo [m].
 * @param y_d Coordenada Y del objetivo [m].
 * @param theta_d Orientación objetivo [rad].
 * @param wL_ref Referencia de velocidad angular para la rueda izquierda [rad/s].
 * @param wR_ref Referencia de velocidad angular para la rueda derecha [rad/s].
 * @param control_mode Modo actual del controlador (debe ser SPEED_REF_AUTO_ADVANCED).
 * @return Estado de movimiento actualizado (STOPPING, REACHED, ALIGNING, MOVING).
 */
MovingState update_control_backstepping(
    const float x, const float y, const float theta,
    const float x_d, const float y_d, const float theta_d,
    volatile float& wL_ref, volatile float& wR_ref,
    const PositionControlMode control_mode
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
 * @param control_mode  Modo de control activo (MOVE_PID, TURN_PID, MOVE_BACKS, TURN_BACKS, etc.).
 * @return              Retorna true si se alcanzó el objetivo y el vehículo está detenido
 */
bool update_control(
    const float x, const float y, const float theta,
    const float x_d, const float y_d, const float theta_d,
    const float v, const float w,
    volatile float& wL_ref, volatile float& wR_ref,
    volatile bool& waypoint_reached,
    const ControlType controller_type,
    const PositionControlMode control_mode
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
 * @param control_mode   Modo actual de control de posición (se ajusta a MANUAL en la función).
 * @return true si ambas velocidades están bajo los umbrales de detención; false en caso contrario.
 */
bool stop_movement(
    const float v, const float w,
    volatile float& w_L_ref, volatile float& w_R_ref,
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
VelocityData constrain_velocity(const float v_raw, const float w_raw);

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
