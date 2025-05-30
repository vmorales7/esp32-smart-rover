#ifndef DISTANCE_SENSORS_H
#define DISTANCE_SENSORS_H

#include "project_config.h"

/* -------------------- Parámetros del sensor ultrasónico (HC-SR04) -------------------- */

/**
 * @brief Velocidad del sonido dividida por 2 en cm/µs.
 * 
 * Se usa para convertir el tiempo de ida y vuelta del pulso de ultrasonido
 * en una distancia unidireccional (desde el sensor al objeto).
 * 
 * 343 m/s → 0.0343 cm/µs → dividido por 2 = 0.01715 cm/µs
 */
constexpr float US_CM_PER_US = 0.01715f;

/**
 * @brief Umbral de detección de obstáculo [cm].
 * 
 * Si la distancia medida es menor a este valor, se considera que hay un obstáculo.
 */
constexpr uint8_t OBSTACLE_THRESHOLD_CM = 30;

/**
 * @brief Distancia máxima considerada válida para el sensor [cm].
 * 
 * Si la lectura supera este valor, se considera inválida o sin respuesta
 * y se devuelve US_MAX_DISTANCE_CM.
 */
constexpr uint8_t US_MAX_DISTANCE_CM = 100U;

/**
 * @brief Tiempo máximo de espera para el pulso de eco [µs].
 * 
 * Limita el tiempo de espera para evitar bloqueos prolongados si no hay reflexión.
 */
constexpr uint32_t US_PULSE_TIMEOUT_US = US_MAX_DISTANCE_CM / US_CM_PER_US;


/* -------------------- Módulo DistanceSensors -------------------- */

namespace DistanceSensors {

    /**
     * @brief Inicializa un sensor ultrasónico individual (TRIG + ECHO).
     * 
     * Configura los pines entregados como salida (TRIG) y entrada (ECHO),
     * y se asegura de dejar el pin TRIG en estado bajo por defecto.
     * 
     * @param trig_pin Número de pin conectado al TRIG del sensor.
     * @param echo_pin Número de pin conectado al ECHO del sensor.
     */
    void init_sensor(const uint8_t trig_pin, const uint8_t echo_pin);

    /**
     * @brief Realiza una lectura puntual de distancia desde un sensor ultrasónico.
     * 
     * Emite un pulso TRIG y mide el tiempo hasta la recepción del pulso ECHO.
     * 
     * @param trig_pin Pin TRIG del sensor.
     * @param echo_pin Pin ECHO del sensor.
     * @return Distancia estimada en cm. Retorna US_MAX_DISTANCE_CM en caso de timeout.
     */
    uint8_t read_distance(const uint8_t trig_pin, const uint8_t echo_pin);

    /**
     * @brief Reinicia el sistema de sensores de distancia con variables individuales.
     *
     * Esta función establece todas las distancias en el valor máximo permitido (`US_MAX_DISTANCE_CM`),
     * limpia todas las banderas de detección de obstáculos (izquierda, medio, derecha y global).
     *
     * @param left_dist Referencia a la distancia medida por el sensor izquierdo [cm].
     * @param left_obst Referencia al flag de obstáculo detectado por el sensor izquierdo.
     * @param mid_dist Referencia a la distancia medida por el sensor central [cm].
     * @param mid_obst Referencia al flag de obstáculo detectado por el sensor central.
     * @param right_dist Referencia a la distancia medida por el sensor derecho [cm].
     * @param right_obst Referencia al flag de obstáculo detectado por el sensor derecho.
     * @param obstacle Referencia al flag de obstáculo global (combinado).
     */
    void reset_system(
        volatile uint8_t& left_dist, volatile bool& left_obst,
        volatile uint8_t& mid_dist, volatile bool& mid_obst,
        volatile uint8_t& right_dist, volatile bool& right_obst,
        volatile bool& obstacle
    );

    /**
     * @brief Inicializa los sensores de distancia y reinicia la estructura de datos asociada.
     * 
     * Configura los pines TRIG y ECHO de los sensores ultrasónicos (izquierdo, medio y derecho).
     * Además, deja en estado INACTIVE el módulo y reinicia valores:
     * - Todas las distancias se establecen en `US_MAX_DISTANCE_CM`.
     * - Todas las banderas de obstáculos se ponen en `false`.
     * 
     * @param left_dist Referencia a la distancia medida por el sensor izquierdo [cm].
     * @param left_obst Referencia al flag de obstáculo detectado por el sensor izquierdo.
     * @param mid_dist Referencia a la distancia medida por el sensor central [cm].
     * @param mid_obst Referencia al flag de obstáculo detectado por el sensor central.
     * @param right_dist Referencia a la distancia medida por el sensor derecho [cm].
     * @param right_obst Referencia al flag de obstáculo detectado por el sensor derecho.
     * @param obstacle Referencia al flag de obstáculo global (combinado).
     * @param distance_state Referencia al estado del sistema de sensores (se establece como INACTIVE).
     */
    void init(
        volatile uint8_t& left_dist, volatile bool& left_obst,
        volatile uint8_t& mid_dist, volatile bool& mid_obst,
        volatile uint8_t& right_dist, volatile bool& right_obst,
        volatile bool& obstacle, volatile uint8_t& distance_state
    );

    /**
     * @brief Establece el estado operativo del sistema de sensores de distancia.
     *
     * Esta función cambia el estado del sistema de sensores a ACTIVE o INACTIVE.
     * Si el nuevo estado es INACTIVE, se ejecuta automáticamente un reinicio completo
     * de las distancias y banderas de detección de obstáculos individuales y global.
     *
     * @param new_mode Nuevo modo a establecer (ACTIVE o INACTIVE).
     * @param distance_state Referencia al estado actual del sistema de sensores.
     * @param left_dist Referencia a la distancia medida por el sensor izquierdo [cm].
     * @param left_obst Referencia al flag de obstáculo detectado por el sensor izquierdo.
     * @param mid_dist Referencia a la distancia medida por el sensor central [cm].
     * @param mid_obst Referencia al flag de obstáculo detectado por el sensor central.
     * @param right_dist Referencia a la distancia medida por el sensor derecho [cm].
     * @param right_obst Referencia al flag de obstáculo detectado por el sensor derecho.
     * @param obstacle Referencia al flag de obstáculo global (true si cualquiera de los sensores lo detecta).
     */
    void set_state(
        const uint8_t new_mode, volatile uint8_t& distance_state,
        volatile uint8_t& left_dist,  volatile bool& left_obst,
        volatile uint8_t& mid_dist,   volatile bool& mid_obst,
        volatile uint8_t& right_dist, volatile bool& right_obst,
        volatile bool& obstacle
    );

    /**
     * @brief Evalúa un único sensor ultrasónico y actualiza su distancia y estado de obstáculo.
     * 
     * Realiza doble lectura para mitigar errores y rebotes. Se actualizan tanto la distancia
     * como el flag de obstáculo si el sistema está en estado ACTIVE.
     * 
     * @param trig_pin Pin TRIG del sensor.
     * @param echo_pin Pin ECHO del sensor.
     * @param distance Variable global con la variable donde se almacena la distancia medida.
     * @param sensor_obstacle_flag Variable global con flag booleano de obstáculo detectado en el sensor.
     * @param global_obstacle_flag Variable global con flag booleano de obstáculo detectado.
     * @param distance_state Variable global con estado global del módulo de sensores.
     * @return Booleano que es true si hay un obstáculo
     */
    bool check_sensor_obstacle(
        const uint8_t trig_pin, const uint8_t echo_pin,
        volatile uint8_t& distance,
        volatile bool& sensor_obstacle_flag,
        volatile bool& global_obstacle_flag,
        volatile uint8_t& distance_state
    );

    /**
     * @brief Realiza la lectura de los tres sensores ultrasónicos frontales,
     *        actualizando las distancias y banderas de obstáculo para cada uno,
     *        y calcula el flag global de obstáculo.
     *
     * Esta función debe ser llamada solo si el sistema de sensores de distancia está activo.
     * Para cada sensor (izquierdo, medio y derecho) realiza la medición, actualiza la distancia
     * y la bandera individual de obstáculo. Finalmente, evalúa el estado global combinando
     * los resultados de los tres sensores.
     *
     * @param left_dist           Distancia medida por el sensor izquierdo [cm].
     * @param left_obst           Flag de obstáculo detectado por el sensor izquierdo.
     * @param mid_dist            Distancia medida por el sensor central [cm].
     * @param mid_obst            Flag de obstáculo detectado por el sensor central.
     * @param right_dist          Distancia medida por el sensor derecho [cm].
     * @param right_obst          Flag de obstáculo detectado por el sensor derecho.
     * @param global_obstacle_flag Bandera global de obstáculo, se actualiza según los tres sensores.
     * @param distance_state      Estado de activación del sistema de sensores de distancia (debe ser ACTIVE).
     * @return true si al menos uno de los tres sensores detecta un obstáculo, false en caso contrario.
     */
    bool check_all_sensors_obstacle(
        volatile uint8_t& left_dist, volatile bool& left_obst, 
        volatile uint8_t& mid_dist, volatile bool& mid_obst,
        volatile uint8_t& right_dist, volatile bool& right_obst,
        volatile bool& global_obstacle_flag,
        volatile uint8_t& distance_state
    );

    /**
     * @brief Calcula la presencia de un obstáculo global a partir de las banderas individuales.
     *
     * Esta función retorna `true` si al menos uno de los sensores ultrasónicos
     * (izquierdo, medio o derecho) ha detectado un obstáculo.
     *
     * @param left_obst Referencia al flag de obstáculo detectado por el sensor izquierdo.
     * @param mid_obst Referencia al flag de obstáculo detectado por el sensor central.
     * @param right_obst Referencia al flag de obstáculo detectado por el sensor derecho.
     * @return `true` si hay al menos un obstáculo detectado por alguno de los sensores.
     */
    bool compute_global_obstacle_flag(
        volatile bool& left_obst,
        volatile bool& mid_obst,
        volatile bool& right_obst
    );

    /**
     * @brief Actualiza el flag global de detección de obstáculos.
     *
     * Esta función calcula si existe un obstáculo en base a las banderas individuales
     * de los sensores (izquierdo, medio, derecho) y actualiza el flag global `obstacle`.
     *
     * @param left_obst Referencia al flag de obstáculo detectado por el sensor izquierdo.
     * @param mid_obst Referencia al flag de obstáculo detectado por el sensor central.
     * @param right_obst Referencia al flag de obstáculo detectado por el sensor derecho.
     * @param obstacle Referencia al flag global de obstáculo, que se actualiza como salida de la función.
     */
    void update_global_obstacle_flag(
        volatile bool& left_obst,
        volatile bool& mid_obst,
        volatile bool& right_obst,
        volatile bool& obstacle
    );

    /**
     * @brief Tarea RTOS que verifica periódicamente el sensor ultrasónico izquierdo.
     * 
     * Ejecuta `check_single_us_sensor()` con desfase inicial para distribuir la carga del sistema.
     * 
     * @param pvParameters Puntero al contexto global (cast a `GlobalContext*`).
     */
    void Task_CheckLeftObstacle(void* pvParameters);

    /**
     * @brief Tarea RTOS que verifica periódicamente el sensor ultrasónico central.
     * 
     * Ejecuta `check_single_us_sensor()` con desfase inicial para distribuir la carga del sistema.
     * 
     * @param pvParameters Puntero al contexto global (cast a `GlobalContext*`).
     */
    void Task_CheckMidObstacle(void* pvParameters);

    /**
     * @brief Tarea RTOS que verifica periódicamente el sensor ultrasónico derecho.
     * 
     * Ejecuta `check_single_us_sensor()` con desfase inicial para distribuir la carga del sistema.
     * 
     * @param pvParameters Puntero al contexto global (cast a `GlobalContext*`).
     */
    void Task_CheckRightObstacle(void* pvParameters);

}

#endif // DISTANCE_SENSORS_H
