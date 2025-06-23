#include "vehicle_OS/vehicle_os.h"
#warning "Compilando main_avance3.cpp"

// ====================== VARIABLES GLOBALES ======================
volatile SystemStates sts;
volatile SensorsData sens;
volatile ControllerData ctrl;
volatile PoseData pose;
volatile OperationData op;
TaskHandlers tasks;
volatile EvadeContext evade;

GlobalContext ctx = {
    .systems_ptr     = &sts,
    .sensors_ptr     = &sens,
    .pose_ptr        = &pose,
    .control_ptr     = &ctrl,
    .os_ptr          = &op, 
    .rtos_task_ptr   = &tasks,
    .evade_ptr       = &evade
};

constexpr ControlType CONTROLLER_TYPE = ControlType::BACKS;// Tipo de controlador a utilizar
constexpr PoseEstimatorType POSE_ESTIMATOR_TYPE = PoseEstimatorType::COMPLEMENTARY;
const bool INCLUDE_EVADE = true; // Habilita el controlador de evasión


void Task_PrintWheelSpeedRef(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(500);
    GlobalContext* ctx_ptr = static_cast<GlobalContext*>(pvParameters);
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        Serial.printf("Vel. L: %.2f, Vel. R: %.2f\n", 
            ctx_ptr->control_ptr->w_L_ref, ctx_ptr->control_ptr->w_R_ref);
    }
}

// ====================== TAREA: Avance con trayectoria ======================
void setup() {
    Serial.begin(115200);
    delay(1000); 
    Serial.println();
    Serial.println("Iniciando prueba Avance 3...");
    Serial.println();
    delay(5000);

    // Inicializar hardware y lanzar todas las tareas RTOS
    OS::enter_init(&ctx);

    // Inicializa la trayectoria (puedes agregar tus puntos de prueba aquí)
    OS::add_waypoint(1.0, 0.0, op);
    OS::add_waypoint(1.0, 1.0, op);
    OS::add_waypoint(0.0, 2.0, op);
    OS::add_waypoint(0.0, 0.0, op);

    // Configurar control de evasión y estimador de pose
    ctrl.controller_type = CONTROLLER_TYPE;
    evade.include_evade = INCLUDE_EVADE;
    pose.estimator_type = POSE_ESTIMATOR_TYPE; 
    
    // xTaskCreatePinnedToCore(
    //     Task_PrintWheelSpeedRef, "PrintWheelSpeedRef", 2048, &ctx, 1, nullptr, 0);
    
}

void loop() {
    // Nada aquí, todo lo maneja el RTOS
}
