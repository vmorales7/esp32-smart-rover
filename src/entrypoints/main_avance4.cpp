#include "vehicle_OS/vehicle_os.h"
#warning "Compilando main_avance4.cpp"

GlobalContext* ctx_ptr = nullptr;
constexpr ControlType CONTROLLER_TYPE = ControlType::PID;// Tipo de controlador a utilizar
constexpr PoseEstimatorType POSE_ESTIMATOR_TYPE = PoseEstimatorType::COMPLEMENTARY;
const bool INCLUDE_EVADE = true; // Habilita el controlador de evasión

void setup() {
    // Inicializar el contexto global
    ctx_ptr = new GlobalContext;
    ctx_ptr->systems_ptr   = new SystemStates;
    ctx_ptr->sensors_ptr   = new SensorsData;
    ctx_ptr->pose_ptr      = new PoseData;
    ctx_ptr->control_ptr   = new ControllerData;
    ctx_ptr->os_ptr        = new OperationData;
    ctx_ptr->rtos_task_ptr = new TaskHandlers;
    ctx_ptr->evade_ptr     = new EvadeContext;

    // Configurar control de evasión y estimador de pose
    ctx_ptr->os_ptr->fb_controller_type = CONTROLLER_TYPE;
    ctx_ptr->evade_ptr->include_evade = INCLUDE_EVADE;
    ctx_ptr->pose_ptr->estimator_type = POSE_ESTIMATOR_TYPE; 

    // Poner en operación el sistema operativo del vehículo
    OS::enter_init(ctx_ptr);
}

void loop() {}