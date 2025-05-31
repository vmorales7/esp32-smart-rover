#include "vehicle_OS/vehicle_os.h"
#warning "Compilando main_avance3.cpp"

// Variables globales
SystemStates        system_states;
OperationData       operation_data;
KinematicState      kinematic_state;
WheelsData          wheels_data;
IMUSensorData       imu_data;
DistanceSensorData  distance_data;

// Contexto global
GlobalContext ctx = {
    &system_states,
    &operation_data,
    &kinematic_state,
    &wheels_data,
    &imu_data,
    &distance_data
};

void setup() {
    Serial.begin(115200);

    // Inicializa la trayectoria (puedes agregar tus puntos de prueba aquí)
    OS::add_waypoint(operation_data, 0.5, 0.0);  // Avanza 0.5m en X
    OS::add_waypoint(operation_data, 0.5, 0.5);  // Gira y avanza en Y

    // Inicializar hardware y lanzar todas las tareas RTOS
    OS::enter_init(&ctx);

    // Listo para operar, todo lo demás lo hace el RTOS
}

void loop() {
    // Nada aquí, todo lo maneja el RTOS
}
