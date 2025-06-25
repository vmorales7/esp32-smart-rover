#include "communications/firebase_comm.h"

#warning "Compilando main_firebase_basic.cpp"

AsyncResult async_aux;

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n\n== Iniciando test de Firebase ==");

    begin_wifi(); 
    init_time(); // Sincronizar hora con NTP
    FirebaseComm::ConnectFirebase(); // Iniciar conexión Firebase

    for (int i = 0; i < 5; i++) {

        Serial.printf("== Enviando datos dummy %d ==\n", i);

        // Timestamps base
        uint64_t now = get_unix_timestamp();
        uint64_t input_ts   = now - 300;
        uint64_t start_ts   = now - 250;
        uint64_t reached_ts = now;

        // Push STATUS_LOG
        FirebaseComm::PushStatus(
            1, "Test dummy status",                     // state, log
            1.0f + i, 2.0f + i, 20.0f + i, 21.0f + i,    // pos_x, pos_y, wL, wR
            input_ts, 10.0f, 20.0f,                     // wp_input_ts, wp_x, wp_y
            0, 0.1f * i, 0.2f * i                       // controller_type, iae, rmse
        );
        delay(1000);

        // Push WAYPOINT_REACHED
        FirebaseComm::PushReachedWaypoint(
            input_ts, 10.0f + i, 20.0f + i,              // input_timestamp, wp_x, wp_y
            start_ts, reached_ts, true,
            10.0f + i + 0.1f, 20.0f + i + 0.1f,          // pos_x, pos_y
            0, 0.1f * i, 0.2f * i                        // controller_type, iae, rmse
        );
        delay(1000);

        // Push WAYPOINT_PENDING
        now = get_unix_timestamp();
        JsonDocument doc;
        doc["input_timestamp"] = now;
        doc["wp_x"] = (10.0f + i);
        doc["wp_y"] = (20.0f + i);
        String path = "/waypoints_pending/" + String(now);
        FirebaseComm::SetJson(path, doc, async_aux);
        delay(1000);
    }

    Serial.println("== Datos dummy enviados ==");
}

void loop() {
    FirebaseComm::ready(); // Mantener Firebase activo

    static bool read_once = false;
    if (!read_once) {
        FirebaseComm::RequestPendingWaypoint();
        delay(1500);  // esperar respuesta inicial

        volatile float x = 0.0f, y = 0.0f;
        volatile uint64_t ts = 0;
        FB_Get_Result result = FB_Get_Result::NO_RESULT;

        while (true){
            FirebaseComm::ready();
            result = FirebaseComm::ProcessPendingWaypoint(x, y, ts);
            if (result != FB_Get_Result::NO_RESULT) break;
            delay(100);
        }

        Serial.printf("Lectura Waypoint: estado = %d | x = %.2f, y = %.2f, ts = %llu\n",
                      (int)result, x, y, ts);
        read_once = true;
    }

    delay(1000);
}

