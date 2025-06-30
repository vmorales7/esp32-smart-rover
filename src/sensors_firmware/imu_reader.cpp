#include "imu_reader.h"

// Instancia del sensor BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// Auxiliar
float wrap_to_pi(float angle_rad) {
    angle_rad = fmodf(angle_rad + PI, 2.0f * PI);
    if (angle_rad < 0.0f) angle_rad += 2.0f * PI;
    return angle_rad - PI;
}


namespace IMUSensor{

// Variables internas del sistema para aceleracion
static float last_theta = 0.0f;
static float acc_offset = 0.0f; 

bool init(
    volatile float& global_acc, volatile float& global_w, volatile float& global_theta, 
    volatile uint8_t& imu_state
){
    // Comenzamos comunicación I2C
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);

    // Inicializar el sensor BNO055
    const bool ok = bno.begin();
    if (ok) bno.setExtCrystalUse(true);

    // Inicializar las variables globales
    global_acc = 0.0f;
    global_w = 0.0f;
    global_theta = 0.0f; // Única vez que se manipula el valor de theta

    // Es necesario esperar tiempo para la primera lectura?
    delay(10);

    // Valor incial de orientación para tener la referencia
    sensors_event_t orientation_data;
    bno.getEvent(&orientation_data, Adafruit_BNO055::VECTOR_EULER);
    last_theta = wrap_to_pi(orientation_data.orientation.x *DEG_TO_RAD); // Guardar el valor inicial de theta

    // Valor inicial de aceleración para considerar el offset
    // resync_acceleration_offset();

    // Inicializar el estado del sistema
    imu_state = INACTIVE; 

    return ok;
};


void set_state(
    const uint8_t new_state, volatile uint8_t& imu_state,
    volatile float& imu_acc, volatile float& imu_w, volatile float& imu_theta
){
    if (new_state == imu_state) return; // si el estado no cambio, no se hace nada

    // Si se pasa a INACTIVE, se hace una ultima lectura de datos
    if (new_state == INACTIVE) {
        read_data(imu_acc, imu_w, imu_theta, imu_state); 
        imu_acc = 0.0f;
        imu_w = 0.0f;
    } else if (new_state == ACTIVE) {
        // Actualizar el offset de aceleración?
    }
    imu_state = new_state; // se actualiza el estado del sistema
}; 


void read_data(
    volatile float& global_acc, volatile float& global_w, volatile float& global_theta, 
    const uint8_t imu_state
){  
    if (imu_state != ACTIVE) return; //solo se lee si el sistema esta activo

    // Obtener datos del IMU
    sensors_event_t accel_data;
    sensors_event_t gyro_data;
    sensors_event_t orientation_data;
    bno.getEvent(&accel_data, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&gyro_data, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&orientation_data, Adafruit_BNO055::VECTOR_EULER);

    // Recuperar datos de aceleración 
    float acc = 0.0f;
    if (ACCEL_DIR == IMUdir::Y) acc = accel_data.acceleration.y; // Aceleración en el eje Y
    else if (ACCEL_DIR == IMUdir::X) acc = accel_data.acceleration.x; // Aceleración en el eje X
    
    // Corrección de la aceleración por ruido y dirección
    if (abs(acc) < ACCEL_TOLERANCE) acc = 0.0f;
    else acc = ((INVERT_SIGN) ? -acc : acc) - acc_offset; // Invertir el signo si es necesario y añadir el offset

    // Se usa la velocidad pura, la corrección se hace en el estimador de pose
    float w = gyro_data.gyro.z;
    const float abs_w = fabsf(w);
    w = (abs_w < W_TOLERANCE) ? 0.0f : w; 

    // Orientacion del IMU
    const float current_theta = orientation_data.orientation.x*DEG_TO_RAD;
    float delta_theta = wrap_to_pi(current_theta - last_theta);
    const float abs_delta_theta = fabsf(delta_theta);
    delta_theta = (abs_delta_theta < THETA_TOLERANCE || abs_delta_theta > MAX_DELTA_THETA) ? 0.0f : delta_theta; 

    Serial.println(delta_theta*RAD_TO_DEG);

    // Actualizar las variables globales
    global_acc = acc;
    global_w = w;
    global_theta -= delta_theta;

    // Actualizar el último valor de theta
    last_theta = current_theta;
}

void resync_acceleration_offset() {
    sensors_event_t accel_data;
    bno.getEvent(&accel_data, Adafruit_BNO055::VECTOR_LINEARACCEL);
    if (ACCEL_DIR == IMUdir::Y) acc_offset = accel_data.acceleration.y; // Aceleración en el eje Y
    else if (ACCEL_DIR == IMUdir::X) acc_offset = accel_data.acceleration.x; // Aceleración en el eje X
}

void Task_IMUData(void* pvParameters) {
    // Datos de RTOS
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(IMU_READ_PERIOD_MS);

    // Cast del parámetro entregado a GlobalContext
    GlobalContext* ctx = static_cast<GlobalContext*>(pvParameters);
    volatile SystemStates& sts = *ctx->systems_ptr;
    volatile SensorsData& sens = *ctx->sensors_ptr;

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        read_data(sens.imu_acc, sens.imu_w, sens.imu_theta, sts.imu);
    }
}  

} // namespace IMUSensor
