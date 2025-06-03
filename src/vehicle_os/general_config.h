#ifndef CONFIG_H
#define CONFIG_H

#include "Arduino.h"

/* -------------- Definiciones de los pines de la ESP32 --------------*/

// Para control del L298N
constexpr uint8_t MOTOR_LEFT_PWM_PIN  = 5;  // Azul
constexpr uint8_t MOTOR_LEFT_DIR_PIN1 = 17; // Blanco
constexpr uint8_t MOTOR_LEFT_DIR_PIN2 = 16; // Verde

constexpr uint8_t MOTOR_RIGHT_PWM_PIN  = 2;  // Azul
constexpr uint8_t MOTOR_RIGHT_DIR_PIN1 = 0; // Blanco
constexpr uint8_t MOTOR_RIGHT_DIR_PIN2 = 4; // Verde

// Para los encoders
// Azul = Vcc & Negro = GND 
constexpr uint8_t ENCODER_LEFT_A_PIN  = 14; // Amarillo
constexpr uint8_t ENCODER_LEFT_B_PIN  = 27; // Verde
constexpr uint8_t ENCODER_RIGHT_A_PIN = 18; // Amarillo
constexpr uint8_t ENCODER_RIGHT_B_PIN = 19; // Verde

// Sensores ultrasónicos
constexpr uint8_t US_LEFT_TRIG_PIN = 25;
constexpr uint8_t US_LEFT_ECHO_PIN = 26;

constexpr uint8_t US_MID_TRIG_PIN = 32;  
constexpr uint8_t US_MID_ECHO_PIN = 35;

constexpr uint8_t US_RIGHT_TRIG_PIN = 22;
constexpr uint8_t US_RIGHT_ECHO_PIN = 23;

// I2C para IMU
constexpr uint8_t IMU_SDA_PIN = 21;
constexpr uint8_t IMU_SCL_PIN = 13;



/* -------------- Constantes generales --------------*/

// Parámetros físicos del vehículo
constexpr float WHEEL_RADIUS = 0.067f / 2.0f;    // en metros
constexpr float WHEELS_SEPARATION = 0.194f;                // distancia entre ruedas (L)
constexpr float WHEEL_TO_MID_DISTANCE = WHEELS_SEPARATION / 2.0f;   // media distancia entre ruedas (L/2)
constexpr float WM_NOM = 215.0 * 2*PI/60.0; // rad/s nominales bajo carga = 22.51 (29.3 sin carga)

// Auxiliares
constexpr uint8_t WHEEL_LEFT  = 0U;
constexpr uint8_t WHEEL_RIGHT = 1U;

constexpr uint8_t ACTIVE   = 1U;
constexpr uint8_t INACTIVE = 0U;

constexpr bool SUCCESS = true;
constexpr bool ERROR   = false;

constexpr float MS_TO_S = 0.001f;

// Puntos de trayectoria
constexpr uint8_t MAX_TRAJECTORY_POINTS = 100; // Define máximo de puntos
constexpr float NULL_WAYPOINT_XY = 99.9f;


/* ----------------------------- Constantes motores ----------------------------*/

// Modos de operación de los motores
enum class MotorMode : uint8_t {
    IDLE = 0U,    // Se dejan libres los motores, alta impedancia entre los bornes del motor
    ACTIVE = 1U,  // Se controla la velocidad con el duty y los pines de control
    AUTO = 2U,    // Modo de control automático
    BREAK = 3U    // Motores bloqueados, frena el motor
};

// Tipos de estimadores de pose
enum class PoseEstimatorType : uint8_t {
    ENCODER = 1U,
    IMU = 2U,
    COMPLEMENTARY  = 3U,
    KALMAN = 4U
};

// Modos de control de posición
enum class PositionControlMode : uint8_t {
    INACTIVE = 0U,
    MANUAL,
    ALIGN,
    MOVE,
    ROTATE
};

// Tipos de controladores
enum class ControlType : uint8_t {
    PID = 0U,
    BACKS
};

// Estados de la máquina de estados del sistema operativo del vehículo autónomo
enum class OS_State : uint8_t {
    INIT = 0,          // Estado inicial al encender
    IDLE,              // Espera sin movimiento
    STAND_BY,          // Espera sin movimiento
    ALIGN,
    MOVE,              // Desplazamiento hacia objetivo
    EVADE              // Evasión de obstáculo
};

// Estructura de punto
struct TargetPoint {
    float x;
    float y;
};

// Instrucciones posibles desde Firebase o interfaz web
enum class RemoteCommand : uint8_t {
    STOP = 0,
    START,
    IDLE
};

// Estados del sistema de evasión
enum class EvadeState : uint8_t {
    IDLE = 0,
    SELECT_DIR,
    WAIT_ALIGN,
    WAIT_ADVANCE,
    WAIT_FREE_PATH,
    WAIT_REJOIN,
    FAIL,
    FINISHED
};


/* -------------- Tiempos de poleo para tareas RTOS --------------*/

constexpr uint16_t WHEEL_CONTROL_PERIOD_MS = 10;
constexpr uint16_t ENCODER_READ_PERIOD_MS = 10;
constexpr uint16_t IMU_READ_PERIOD_MS = 10;
constexpr uint16_t OBSTACLE_CHECK_PERIOD_MS = 200;
constexpr uint16_t POSE_ESTIMATOR_PERIOD_MS = 10; 
constexpr uint16_t POSITION_CONTROL_PERIOD_MS = 200;
constexpr uint16_t OS_UPDATE_PERIOD_MS = 50; 

constexpr uint16_t BASIC_STACK_SIZE = 2048; // Tamaño de stack básico para tareas RTOS


/* -------------------- Estructuras con la data del sistema --------------------*/

/**
 * @brief Indica el estado de activación de los distintos subsistemas del vehículo autónomo.
 *
 * Esta estructura es usada como bandera de control para suspender o reanudar módulos en ejecución
 * de forma segura desde la lógica central (`vehicle_os`) o al cambiar entre estados operacionales.
 *
 * Solo debe ser modificada por `vehicle_os` u otras tareas de sistema.
 */
struct SystemStates {
    /// Estado del subsistema de motores (MOTOR_IDLE, MOTOR_ACTIVE, etc.).
    /// Define el comportamiento general del controlador de motores y PWM.
    MotorMode motors;

    /// Estado del lector de encoders (ACTIVE o INACTIVE).
    /// Si está inactivo, los encoders se pausan y la velocidad se congela en 0.
    uint8_t encoders;

    /// Estado del lector de IMU (ACTIVE o INACTIVE).
    /// Aún no implementado, pero reservado para integración futura.
    uint8_t imu;

    /// Estado del módulo de sensores de distancia (ultrasonido + IR).
    /// Controla la ejecución periódica de `distance_sensors`.
    uint8_t distance;

    /// Estado del estimador de pose (ACTIVE o INACTIVE).
    /// Determina si se actualiza la pose del vehículo en cada ciclo.
    uint8_t pose;

    /// Estado del controlador de posición (ACTIVE o INACTIVE).
    /// Define si se está generando v_ref/w_ref activamente.
    PositionControlMode position;

    // Constructor por defecto
    SystemStates() :
        motors(MotorMode::IDLE), 
        encoders(INACTIVE), imu(INACTIVE), distance(INACTIVE), pose(INACTIVE),
        position(PositionControlMode::INACTIVE)
    {}
};

struct SensorsData{
    /// Ángulo acumulados del encoder.
    /// Se actualiza periódicamente por el módulo `encoder_reader`.
    /// Solo una lectura por medio del estimador de pose puede reiniciar su valor para evitar perder datos.
    float enc_phiL;
    float enc_phiR;

    /// Velocidad angular medida de las ruedas [rad/s] mediante el encoder.
    /// Calculada a partir del número de pasos y el intervalo de muestreo.
    float enc_wL;
    float enc_wR;

    uint8_t us_left_dist;      ///< Distancia medida por sensor US izquierdo [cm]
    uint8_t us_mid_dist;       ///< Distancia medida por sensor US medio/frontal [cm]
    uint8_t us_right_dist;     ///< Distancia medida por sensor US derecho [cm]

    bool us_left_obst;         ///< true si el sensor izquierdo detecta obstáculo (< threshold)
    bool us_mid_obst;          ///< true si el sensor medio detecta obstáculo
    bool us_right_obst;        ///< true si el sensor derecho detecta obstáculo

    bool us_obstacle;          ///< true si cualquier sensor US detecta obstáculo (< threshold)

    float imu_acc; ///< aceleración en el eje x
    float imu_w; ///< velocidad angular con respecto al eje z
    float imu_theta; ///< ángulo rotación respecto al norte magnetico de output total (orientacion absoluta)

    // Constructor por defecto
    SensorsData()
        : enc_phiL(0.0f), enc_phiR(0.0f),
          enc_wL(0.0f), enc_wR(0.0f),
          us_left_dist(0), us_mid_dist(0), us_right_dist(0),
          us_left_obst(false), us_mid_obst(false), us_right_obst(false),
          imu_acc(0.0f), imu_w(0.0f), imu_theta(0.0f)
    {}
};

struct PoseData {

    // Tipo de estimador de pose utilizado
    PoseEstimatorType estimator_type;

    /// Posición actual en el eje X [m]
    float x;

    /// Posición actual en el eje Y [m]
    float y;

    /// Orientación actual del vehículo respecto al eje X [rad].
    /// θ = 0 indica orientación hacia el eje X positivo.
    float theta;

    /// Velocidad lineal actual del vehículo [m/s].
    /// Calculada por el estimador de pose.
    float v;

    /// Velocidad angular actual del vehículo [rad/s].
    /// Calculada por el estimador de pose.
    float w;

    /// Velocidad de ruedas
    float w_L;
    float w_R;

    PoseData()
        : estimator_type(PoseEstimatorType::COMPLEMENTARY),
          x(0.0f), y(0.0f), theta(0.0f),
          v(0.0f), w(0.0f),
          w_L(0.0f), w_R(0.0f)
    {}
};

struct ControllerData {
    /// Velocidad angular de las ruedas [rad/s] calculada por el controlador de posición.
    float w_L_ref;
    float w_R_ref;

    /// Duty aplicado al PWM de los motores (normalizado en [-1, 1]). 
    /// Calculado por el controlador de rueda.
    /// Su signo determina el signo del voltaje del motor.
    float duty_L;
    float duty_R;

    /// Posición objetivo en el eje X [m].
    float x_d;

    /// Posición objetivo en el eje Y [m].
    float y_d;

    /// Orientación del objetivo respecto a los ejes de referencia [rad].
    float theta_d;

    /// Tipo de controlador utilizado para la posición. Puede ser PID o BACKS (Backstepping).
    ControlType controller_type;

    /// Bandera que indica si alcanzó el objetivo actual
    bool waypoint_reached;

    // Constructor por defecto
    ControllerData() 
        : w_L_ref(0.0f), w_R_ref(0.0f),
          duty_L(0.0f), duty_R(0.0f),
          x_d(0.0f), y_d(0.0f), theta_d(0.0f),
          controller_type(ControlType::PID),
          waypoint_reached(false)
    {}
};

/**
 * @brief Contiene la información de alto nivel relacionada con la operación del vehículo autónomo.
 *
 * Esta estructura centraliza el estado actual del sistema, la trayectoria de navegación cargada,
 * y la última instrucción remota recibida (por ejemplo, desde Firebase). Se utiliza por la máquina
 * de estados principal (`vehicle_os`) para coordinar la lógica de navegación y comportamiento del vehículo.
 *
 * Campos clave:
 * - current_state: Estado operativo actual del sistema (ej. INIT, NAVIGATING, etc.)
 * - trajectory: Lista de puntos objetivo a seguir, en orden.
 * - total_targets: Cantidad de puntos cargados actualmente en `trajectory`.
 * - last_remote_command: Última instrucción recibida por interfaz remota (START, STOP, etc.)
 */
struct OperationData {
    OS_State state;
    uint8_t total_targets;
    TargetPoint trajectory[MAX_TRAJECTORY_POINTS];
    RemoteCommand last_command;
    char last_log[64];  // <-- Aquí el log de transición

    // Constructor por defecto
    OperationData() : 
    state(OS_State::INIT),
    total_targets(0),
    last_command(RemoteCommand::STOP)
    {
        for (uint8_t i = 0; i < MAX_TRAJECTORY_POINTS; ++i) {
            trajectory[i] = {NULL_WAYPOINT_XY, NULL_WAYPOINT_XY};
        }
        last_log[0] = '\0'; // String vacío al inicio
    }
};

/**
 * @brief Estructura que almacena los manejadores de tareas RTOS del vehículo.
 */
struct TaskHandlers {
    TaskHandle_t wheels_handle;
    TaskHandle_t obstacle_handle;  ///< Manejador de la tarea de chequeo de obstáculos
    TaskHandle_t encoder_handle;   ///< Manejador de la tarea de lectura de encoders
    TaskHandle_t imu_handle;       ///< Manejador de la tarea de lectura de IMU
    TaskHandle_t pose_handle;      ///< Manejador de la tarea de estimación de pose
    TaskHandle_t position_handle;  ///< Manejador de la tarea de control de posición
    TaskHandle_t os_handle;        ///< Manejador de la tarea de operación del sistema

    // Constructor por defecto
    TaskHandlers()
        : wheels_handle(nullptr),
          obstacle_handle(nullptr),
          encoder_handle(nullptr),
          imu_handle(nullptr),
          pose_handle(nullptr),
          position_handle(nullptr),
          os_handle(nullptr)
    {}
};

/**
 * @brief Contexto de evasión del vehículo autónomo.
 *
 * Esta estructura mantiene el estado y parámetros necesarios para realizar maniobras de evasión
 * cuando se detecta un obstáculo. Permite al vehículo intentar esquivar obstáculos a ambos lados
 * y retomar su trayectoria original si es posible.
 */
struct EvadeContext {
    bool include_evade = true; // Indica si se usa el controlador de evasión
    EvadeState state = EvadeState::IDLE;
    int8_t direction = 0;          // +1: izquierda, -1: derecha
    float current_angle = 0.0f;     // Ángulo de evasión actual [rad]
    bool tried_both_sides = false; // Flag que indica si se intentó evasión a ambos lados
    TargetPoint saved_waypoint = {0.0f, 0.0f};
};

/**
 * @brief Estructura global de contexto que agrupa punteros a las principales estructuras de estado del sistema.
 * Esta estructura se utiliza para facilitar el paso de datos compartidos entre diferentes tareas RTOS o módulos del sistema.
 */
struct GlobalContext {
    volatile SystemStates* systems_ptr;
    volatile SensorsData* sensors_ptr;
    volatile PoseData* pose_ptr;
    volatile ControllerData* control_ptr;
    volatile OperationData* os_ptr;
    TaskHandlers* rtos_task_ptr;
    volatile EvadeContext* evade_ptr;
};

#endif
