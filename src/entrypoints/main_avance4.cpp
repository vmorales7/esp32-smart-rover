#include "vehicle_OS/vehicle_os.h"
#warning "Compilando main_avance4.cpp"

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

// ====================== CONSTANTES DE CONFIGURACIÓN ======================

constexpr ControlType CONTROLLER_TYPE = ControlType::BACKS;// Tipo de controlador a utilizar
constexpr PoseEstimatorType POSE_ESTIMATOR_TYPE = PoseEstimatorType::COMPLEMENTARY;
const bool INCLUDE_EVADE = true; // Habilita el controlador de evasión

// =================== Operación del sistema ====================

void setup() {
    // Inicializar hardware y lanzar todas las tareas RTOS
    OS::enter_init(&ctx);

    // Configurar control de evasión y estimador de pose
    op.fb_controller_type = CONTROLLER_TYPE;
    evade.include_evade = INCLUDE_EVADE;
    pose.estimator_type = POSE_ESTIMATOR_TYPE; 
    
    
}

void loop() {
    // Nada aquí, todo lo maneja el RTOS
}
