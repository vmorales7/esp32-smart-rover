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

AsyncResult async_command;
AsyncResult async_status;
AsyncResult async_current_waypoint;
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

bool IsResultOk(AsyncResult &res) {
    return res.isResult() && !res.isError();
}

void RequestCommands() {
    Database.get(async_client, "/commands", async_command);
}

bool ProcessCommands(int &action, int &controller_type) {
    if (!IsResultOk(async_command)) return false;

    StaticJsonDocument<64> doc;
    DeserializationError err = deserializeJson(doc, async_command.c_str());

    if (err) {
        if(FB_DEBUG_MODE) {
            Serial.print("JSON parse error: ");
            Serial.println(err.c_str());
        }
        return false;
    }

    if (!doc.containsKey("action") || !doc.containsKey("controller_type"))
        return false;

    action = doc["action"];
    controller_type = doc["controller_type"];

    return true;
}

bool PushStatus(
    const float x, const float y, const float wL, const float wR, 
    const int state, const char* log_msg, const int controller_type,
    const float rmse, const float iae
) {
    const uint32_t timestamp = get_unix_timestamp();
    const String time_str = timestamp_to_string(timestamp);
    const int rpm_L = round(wL * 60.0 / (2 * 3.14)) + 0.5;
    const int rpm_R = round(wR * 60.0 / (2 * 3.14)) + 0.5;

    StaticJsonDocument<256> doc;
    doc["timestamp"] = timestamp;
    doc["time"] = time_str;
    doc["state"] = state;
    doc["x_position"] = x;
    doc["y_position"] = y;
    doc["rpm_left"] = rpm_L;
    doc["rpm_right"] = rpm_R;
    doc["log_msg"] = log_msg;
    doc["controller_type"] = controller_type;
    doc["rmse"] = rmse;
    doc["iae"] = iae;

    String path = "/status_log/" + String(timestamp);
    SetJson(path, doc, async_status);
    return true;
}

bool PushCurrentWaypoint(const uint32_t input_timestamp, const float x, const float y) {
    const uint32_t start_timestamp = get_unix_timestamp();
    const String input_time_str = timestamp_to_string(input_timestamp);
    const String start_time_str = timestamp_to_string(start_timestamp);

    StaticJsonDocument<256> doc;
    doc["input_timestamp"] = input_timestamp;
    doc["start_timestamp"] = start_timestamp;
    doc["input_time"] = input_time_str;
    doc["start_time"] = start_time_str;
    doc["x"] = x;
    doc["y"] = y;

    String path = "/waypoints_current";
    SetJson(path, doc, async_current_waypoint);
    return true;
}

bool RemoveWaypointPending(const uint32_t input_timestamp) {
    String path = "/waypoints_pending/" + String(input_timestamp);
    Database.remove(async_client, path, async_pending_waypoints);
    return true; 
}

bool PushWaypointReached(
    const uint32_t input_timestamp,
    const uint32_t start_timestamp,
    const float x, const float y,
    const String& input_time,
    const String& start_time,
    const int total_length_seg  // acumulador desde OS
) {
    const uint32_t reached_timestamp = get_unix_timestamp();
    const String reached_time = timestamp_to_string(reached_timestamp);
    const int trip_length_seg = reached_timestamp - start_timestamp;

    StaticJsonDocument<384> doc;
    doc["input_timestamp"] = input_timestamp;
    doc["start_timestamp"] = start_timestamp;
    doc["reached_timestamp"] = reached_timestamp;
    doc["input_time"] = input_time;
    doc["start_time"] = start_time;
    doc["reached_time"] = reached_time;
    doc["trip_length_seg"] = trip_length_seg;
    doc["total_length_seg"] = total_length_seg;
    doc["x"] = x;
    doc["y"] = y;

    String path = "/waypoints_reached/" + String(reached_timestamp);
    SetJson(path, doc, async_reached_waypoints);
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

bool ProcessOldestPendingWaypoint(uint32_t &input_time, float &x, float &y) {
    if (!IsResultOk(async_pending_waypoints)) return false;

    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, async_pending_waypoints.c_str());
    if (err) {
        if(FB_DEBUG_MODE) {
            Serial.print("JSON parse error (ProcessOldestPendingWaypoint): ");
            Serial.println(err.c_str());
        }
        return false;
    }

    JsonObject root = doc.as<JsonObject>();
    if (root.isNull()) return false;

    for (JsonPair kv : root) {
        input_time = String(kv.key().c_str()).toInt();
        JsonObject obj = kv.value().as<JsonObject>();
        x = obj["x"] | NULL_WAYPOINT_XY;
        y = obj["y"] | NULL_WAYPOINT_XY;
        return true;
    }

    return false;
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
