#include "vehicle_os/general_config.h"
#include "vehicle_os/vehicle_os.h"
#warning "Compilando main_debug.cpp"

#include "communications/firebase_comm.h"

void setup() {
    Serial.begin(115200);
    delay(5000);
    Serial.println("\n== Iniciando test completo Firebase ==");

    // Tiene que quedar adentro
    volatile SystemStates sts;
    volatile SensorsData sens;
    volatile ControllerData ctrl;
    volatile PoseData pose;
    volatile OperationData op;
    volatile EvadeContext evade;
    TaskHandlers tasks;
    GlobalContext ctx = {
        .systems_ptr     = &sts,
        .sensors_ptr     = &sens,
        .pose_ptr        = &pose,
        .control_ptr     = &ctrl,
        .os_ptr          = &op, 
        .rtos_task_ptr   = &tasks,
        .evade_ptr       = &evade
    };

    // Iniciar WiFi y sincronizar hora
    begin_wifi();
    init_time();
    FirebaseComm::ConnectFirebase();

    delay(1000);
    FirebaseComm::ready();

    // Limpiar registros anteriores
    FB_State state_clear = FirebaseComm::ClearAllLogs(state_clear);
    while (state_clear == FB_State::PENDING) {
        FirebaseComm::ready();
        delay(200);
        state_clear = FirebaseComm::ClearAllLogs(state_clear);
    }
    Serial.printf("Resultado ClearAllLogs: %d\n", (int)state_clear);

    FB_State state_pending = FirebaseComm::ClearPendingWaypoints(state_pending);
    while (state_pending == FB_State::PENDING) {
        FirebaseComm::ready();
        delay(200);
        state_pending = FirebaseComm::ClearPendingWaypoints(state_pending);
    }
    Serial.printf("Resultado ClearPending: %d\n", (int)state_pending);

    // Enviar datos dummy
    for (int i = 0; i < 5; i++) {
        Serial.printf("\n== Enviando set dummy #%d ==\n", i);
        uint64_t now = get_unix_timestamp();
        uint64_t input_ts   = now - 300 + i * 10;
        uint64_t start_ts   = now - 250 + i * 10;
        uint64_t reached_ts = now + i * 10;

        // Push a status
        FirebaseComm::PushStatus(
            1, "Dummy test status",
            1.0f + i, 2.0f + i, 20.0f + i, 21.0f + i,
            input_ts, 10.0f + i, 20.0f + i,
            0, 0.1f * i, 0.2f * i
        );
        delay(1000);

        // Push a reached
        FirebaseComm::PushReachedWaypoint(
            input_ts, 10.0f + i, 20.0f + i,
            start_ts, reached_ts, true,
            10.0f + i + 0.1f, 20.0f + i + 0.1f,
            0, 0.1f * i, 0.2f * i
        );
        delay(1000);

        // Push a pending
        uint64_t ts_now = get_unix_timestamp();
        JsonDocument doc;
        doc["input_timestamp"] = ts_now;
        doc["wp_x"] = 10.0f + i;
        doc["wp_y"] = 20.0f + i;
        AsyncResult aux;
        FirebaseComm::SetJson("/waypoints_pending/" + String(ts_now), doc, aux);
        delay(1000);
    }

    Serial.println("\n== Solicitud y lectura de waypoint pendiente ==");

    FirebaseComm::RequestPendingWaypoint();
    delay(1000);

    volatile float x = 0.0f, y = 0.0f;
    volatile uint64_t ts = 0;
    FB_Get_Result res = FB_Get_Result::NO_RESULT;

    while (res == FB_Get_Result::NO_RESULT) {
        FirebaseComm::ready();
        res = FirebaseComm::ProcessPendingWaypoint(x, y, ts);
        delay(200);
    }

    Serial.printf("Lectura Waypoint: Estado = %d | x = %.2f, y = %.2f, ts = %llu\n",
                  (int)res, x, y, ts);
}

void loop() {
    FirebaseComm::ready();
    delay(500);
}
