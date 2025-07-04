#include "vehicle_OS/vehicle_os.h"
#warning "Compilando main_avance4.cpp"

#include <esp_heap_caps.h>

// ====================== VARIABLES GLOBALES ======================
// Inicializar variables globales
volatile SystemStates sts;
volatile SensorsData sens;
volatile ControllerData ctrl;
volatile PoseData pose;
volatile OperationData op;
TaskHandlers tasks;
volatile EvadeContext evade;
GlobalContext ctx;

// ====================== CONSTANTES DE CONFIGURACIÓN ======================

constexpr ControlType CONTROLLER_TYPE = ControlType::PID;// Tipo de controlador a utilizar
constexpr PoseEstimatorType POSE_ESTIMATOR_TYPE = PoseEstimatorType::ENCODER;
const bool INCLUDE_EVADE = true; // Habilita el controlador de evasión

// =================== Operación del sistema ====================

void setup() 
{
    // Llenar la estructura de contexto global
    ctx.systems_ptr    = &sts;
    ctx.sensors_ptr    = &sens;
    ctx.pose_ptr       = &pose;
    ctx.control_ptr    = &ctrl;
    ctx.os_ptr         = &op; 
    ctx.rtos_task_ptr  = &tasks;
    ctx.evade_ptr      = &evade;

    // Inicializar hardware y lanzar todas las tareas RTOS
    OS::enter_init(&ctx);

    // Configurar control de evasión y estimador de pose
    op.fb_controller_type = CONTROLLER_TYPE;
    evade.include_evade = INCLUDE_EVADE;
    pose.estimator_type = POSE_ESTIMATOR_TYPE; 
}

void loop() {}
