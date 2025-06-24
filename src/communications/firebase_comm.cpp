#include "firebase_comm.h"

/* Las cosas a continuación podrían ir en el main para ser mas genérico pero quedará acá*/

// Definir el cliente asíncrono
SSL_CLIENT ssl_client;
using AsyncClient = AsyncClientClass;
AsyncClient async_client(ssl_client);

// Clase spara uso de Firebase
UserAuth user_auth(FB_DATABASE_API_KEY, FB_USER_EMAIL, FB_USER_PASSWORD, 3000);
FirebaseApp app;
RealtimeDatabase Database;


/* ---------- Namespace con las funciones ----------*/
namespace FirebaseComm {

AsyncResult async_commands;
AsyncResult async_status;
AsyncResult async_pending_waypoints;
AsyncResult async_reached_waypoints;

bool SetupClientObjects() {
    ssl_client.setInsecure();
    initializeApp(async_client, app, getAuth(user_auth), auth_debug_print, "authTask");
    
    app.getApp<RealtimeDatabase>(Database);
    Database.url(FB_DATABASE_URL);
    return ready();
}

bool ready() {
    const bool initialized = app.isInitialized();
    if (initialized) app.loop();
    return initialized && app.ready();
}


void SetJson(const String &path, const JsonDocument &doc, AsyncResult &result) {
    String json;
    serializeJson(doc, json);
    object_t obj(json);
    Database.set<object_t>(async_client, path, obj, result);
}

bool IsResultOk(AsyncResult &ares) {
    return ares.isResult() && !ares.isError();
}


void RequestCommands() {
    Database.get(async_client, "/commands", async_commands);
}

CommandProcessResult ProcessCommands(int &action, int &controller_type) {
    if (!async_commands.isResult()) {
        if (FB_DEBUG_MODE) Serial.println("No hay comandos disponibles o aún no se han recibido.");
        return CMD_ASYNC_NO_RESULT;
    }
    if (async_commands.isError()) {
        if (FB_DEBUG_MODE) Serial.println("Error al recibir comandos.");
        return CMD_ASYNC_ERROR;
    } 

    StaticJsonDocument<64> doc;
    DeserializationError err = deserializeJson(doc, async_commands.c_str());
    if (err) {
        if (FB_DEBUG_MODE) {
            Serial.print("JSON parse error: ");
            Serial.println(err.c_str());
        }
        return CMD_PARSE_ERROR;
    }

    if (!doc.containsKey("action") || !doc.containsKey("controller_type")) {
        if (FB_DEBUG_MODE) Serial.println("Faltan campos requeridos en el comando.");
        return CMD_MISSING_FIELDS;
    }

    action = doc["action"];
    controller_type = doc["controller_type"];
    return CMD_OK;
}


bool PushStatus(
    const float x, const float y, const float wL, const float wR,
    const uint8_t state, const char* log_msg, const uint8_t controller_type,
    const float rmse, const float iae,
    const float wp_x, const float wp_y,
    const uint32_t wp_input_ts
) {
    const uint32_t timestamp = get_unix_timestamp();
    const String time_str = timestamp_to_string(timestamp);
    const String wp_input_time = timestamp_to_string(wp_input_ts);

    const int rpm_L = round(wL * 60.0 / (2 * 3.14)) + 0.5;
    const int rpm_R = round(wR * 60.0 / (2 * 3.14)) + 0.5;

    StaticJsonDocument<384> doc;
    doc["timestamp"] = timestamp;
    doc["time"] = time_str;
    doc["state"] = state;
    doc["position_x"] = x;
    doc["position_y"] = y;
    doc["rpm_left"] = rpm_L;
    doc["rpm_right"] = rpm_R;
    doc["log_msg"] = log_msg;
    doc["controller_type"] = controller_type;
    doc["rmse"] = rmse;
    doc["iae"] = iae;

    doc["wp_x"] = wp_x;
    doc["wp_y"] = wp_y;
    doc["wp_input_timestamp"] = wp_input_ts;
    doc["wp_input_time"] = wp_input_time;

    String path = "/status_log/" + String(timestamp);
    SetJson(path, doc, async_status);
    return true;
}

PushStatusResult ProcessPushStatus(
    const float x, const float y, const float wL, const float wR,
    const uint8_t state, const char* log_msg, const uint8_t controller_type,
    const float rmse, const float iae,
    const float wp_x, const float wp_y,
    const uint32_t wp_input_ts
) {
    static uint8_t errors = 0;
    if (IsResultOk(async_status)) {
        if (FB_DEBUG_MODE) Serial.println("PushStatus: Éxito al subir el estado.");
        errors = 0;  
        return PUSH_STATUS_OK;
    }

    // Aún no termina (ni éxito ni error)
    if (!async_status.isResult() && !async_status.isError()) {
        return PUSH_STATUS_NOT_READY;
    }

    // Terminó pero hubo error
    errors += 1;
    if (errors < PUSH_STATUS_MAX_ERRORS) {
        if (FB_DEBUG_MODE) Serial.println("PushStatus: Error al subir el estado.");
        return PUSH_STATUS_ERROR;
    } else {
        if (FB_DEBUG_MODE) Serial.println("PushStatus: Se alcanzó el máximo de errores permitidos.");
        errors = 0;  // Reiniciar contador de errores
        return PUSH_STATUS_MAX_ERRORS;
    }
}

bool PushWaypointReached(
    const float wp_x, const float wp_y, const float x, const float y,
    const uint32_t input_timestamp, const uint32_t start_timestamp, const uint32_t reached_timestamp,
    const uint32_t trip_length_sec
) {
    const String input_time = timestamp_to_string(input_timestamp);
    const String start_time = timestamp_to_string(start_timestamp);
    const String reached_time = timestamp_to_string(reached_timestamp);

    StaticJsonDocument<384> doc;
    doc["input_timestamp"] = input_timestamp;
    doc["start_timestamp"] = start_timestamp;
    doc["reached_timestamp"] = reached_timestamp;

    doc["input_time"] = input_time;
    doc["start_time"] = start_time;
    doc["reached_time"] = reached_time;

    doc["trip_length_sec"] = trip_length_sec;

    doc["x"] = x;
    doc["y"] = y;

    doc["wp_x"] = wp_x;
    doc["wp_y"] = wp_y;

    String path = "/waypoints_reached/" + String(reached_timestamp);
    SetJson(path, doc, async_reached_waypoints);
    return true;
}

bool ProcessPushWaypointReached() {
    static uint8_t retries = 0;

    if (IsResultOk(async_reached_waypoints)) {
        retries = 0;
        return true;
    }

    // Aún no termina (ni éxito ni error)
    if (!async_reached_waypoints.isResult() && !async_reached_waypoints.isError()) {
        return false;
    }

    // Terminó con error → reintentar
    if (retries < max_retries) {
        retries++;
        if (FB_DEBUG_MODE)
            Serial.printf("Reintentando push #%d del waypoint alcanzado...\n", retries);

        PushWaypointReached(x, y, input_timestamp, start_timestamp, reached_timestamp, trip_length_sec);
        return false;
    }

    // Todos los intentos fallaron
    if (FB_DEBUG_MODE) Serial.println("No se pudo subir el waypoint alcanzado tras múltiples intentos.");

    retries = 0;
    return false;
}

bool RemoveWaypointPending(const uint32_t input_timestamp) {
    String path = "/waypoints_pending/" + String(input_timestamp);
    Database.remove(async_client, path, async_pending_waypoints);
    return true; 
}

void RequestOldestPendingWaypoint() {
    DatabaseOptions options;
    options.filter.orderBy("$key").limitToFirst(1);
    Database.get(async_client, "/waypoints_pending", options, async_pending_waypoints);
}

bool HasPendingWaypoints() {
    if (!IsResultOk(async_pending_waypoints)) return false;

    StaticJsonDocument<64> doc;
    DeserializationError err = deserializeJson(doc, async_pending_waypoints.c_str());
    if (err) {
        if (FB_DEBUG_MODE) {
            Serial.print("JSON parse error (HasPendingWaypoints): ");
            Serial.println(err.c_str());
        }
        return false;
    }
    JsonObject root = doc.as<JsonObject>();
    if (root.isNull() || root.size() == 0) return false;
    return true;
}

bool ProcessOldestPendingWaypoint(TargetPoint &target_out) {
    if (!IsResultOk(async_pending_waypoints)) return false;
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, async_pending_waypoints.c_str());
    if (err) {
        if (FB_DEBUG_MODE) {
            Serial.print("JSON parse error (ProcessOldestPendingWaypoint): ");
            Serial.println(err.c_str());
        }
        return false;
    }
    JsonObject root = doc.as<JsonObject>();
    if (root.isNull() || root.size() == 0) return false;

    for (JsonPair kv : root) {
        target_out.ts = String(kv.key().c_str()).toInt();
        JsonObject obj = kv.value().as<JsonObject>();
        target_out.x = obj["x"] | NULL_WAYPOINT_XY;
        target_out.y = obj["y"] | NULL_WAYPOINT_XY;
        return true;
    }
    return false;  // fallback si root existe pero no hay elementos válidos
}


// ========================
// Debug (opcional)
// ========================
void auth_debug_print(AsyncResult &aResult)
{
    if (aResult.isEvent()) {
        Firebase.printf("Event task: %s, msg: %s, code: %d\n", aResult.uid().c_str(), aResult.eventLog().message().c_str(), aResult.eventLog().code());
    }
    if (aResult.isDebug()) {
        Firebase.printf("Debug task: %s, msg: %s\n", aResult.uid().c_str(), aResult.debug().c_str());
    }
    if (aResult.isError()) {
        Firebase.printf("Error task: %s, msg: %s, code: %d\n", aResult.uid().c_str(), aResult.error().message().c_str(), aResult.error().code());
    }
}

} // namespace FirebaseComm
