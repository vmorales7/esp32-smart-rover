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

const bool include_evade = true; // Habilita el controlador de evasión

// ====================== TAREA: Avance con trayectoria ======================
void setup() {
    Serial.begin(115200);

    // Inicializa la trayectoria (puedes agregar tus puntos de prueba aquí)
    OS::add_waypoint(0.5, 0.0, op);  // Avanza 0.5m en X
    OS::add_waypoint(0.5, 0.5, op);  // Gira y avanza en Y
    OS::add_waypoint(1.0, 0.0, op);  // Avanza en diagonal

    // Decidir si incluir el controlador de evasión
    evade.include_evade = include_evade;

    // Inicializar hardware y lanzar todas las tareas RTOS
    OS::enter_init(&ctx);
}

void loop() {
    // Nada aquí, todo lo maneja el RTOS
}
