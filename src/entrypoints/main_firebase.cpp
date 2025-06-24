#include "communications/firebase_comm.h"

#warning "Compilando main_firebase.cpp"

uint32_t timestamp;
AsyncResult async_aux;

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n\n== Iniciando test de Firebase ==");

    begin_wifi(); 
    init_time(); // Sincronizar hora con NTP
    FirebaseComm::SetupFirebaseConnect(); // Iniciar conexión Firebase

    for (int i = 0; i < 5; i++) {

        // Push STATUS_LOG
        timestamp = get_unix_timestamp();
        FirebaseComm::PushStatus(
            1.0f + i, 2.0f + i, 20.0f + i, 21.0f + i, 
            1, "Test dummy status", 0, 0.1f * i, 0.2f * i, 
            10.0f, 20.0f, timestamp - 300
        );
        delay(1000);
        FB_Push_Result r1 = FB_Push_Result::NOT_READY;
        while (true) {
            FirebaseComm::ready();
            r1 = FirebaseComm::ProcessPush(FB_PushType::STATUS);
            if (r1 != FB_Push_Result::NOT_READY) break;
            delay(100);
        }  

        // Push WAYPOINT_REACHED
        FirebaseComm::PushReachedWaypoint(
            10.0f, 20.0f, timestamp - 300,
            1.0f + i, 2.0f + i,
            timestamp - 250, timestamp, 50
        );
        delay(1000);
        FB_Push_Result r2 = FB_Push_Result::NOT_READY;
        while (true) {
            FirebaseComm::ready();
            r2 = FirebaseComm::ProcessPush(FB_PushType::REACHED);
            if (r2 != FB_Push_Result::NOT_READY) break;
            delay(100);
        }

        // Revisar los push
        Serial.printf("Status push = %d, Reached push = %d\n", (int)r1, (int)r2);

        // Push WAYPOINT_PENDING
        timestamp = get_unix_timestamp();
        JsonDocument doc;
        doc["input_timestamp"] = timestamp;
        doc["input_time"] = timestamp_to_string(timestamp);
        doc["wp_x"] = (10.0f + i);
        doc["wp_y"] = (20.0f + i);
        String path = "/waypoints_pending/" + String(timestamp);
        FirebaseComm::SetJson(path, doc, async_aux);
        delay(1000);        
        
    }
    Serial.println("== Datos dummy enviados ==");
}

void loop() {
    FirebaseComm::ready(); // Mantener Firebase listo

    static bool read_once = false;
    if (!read_once) {
        FirebaseComm::RequestPendingWaypoint();
        delay(1500);  // esperar que responda
        TargetPoint target;
        FB_Get_Result result = FirebaseComm::ProcessPendingWaypoint(target);
        while (true){
            FirebaseComm::ready();
            result = FirebaseComm::ProcessPendingWaypoint(target);
            if (result != FB_Get_Result::NO_RESULT) break; // salir si ya hay resultado
            delay(100);
        }
        Serial.printf("Lectura Waypoint: estado = %d | x = %.2f, y = %.2f\n", (int)result, target.x, target.y);
        read_once = true;
    }
    delay(1000);
}
