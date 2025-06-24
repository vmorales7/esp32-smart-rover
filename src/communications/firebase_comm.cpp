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


void RequestCommands() {
    Database.get(async_client, "/commands", async_commands);
    if (FB_DEBUG_MODE) Serial.println("RequestCommands: solicitud enviada.");
}

FB_Get_Result ProcessRequestCommands(int &action, int &controller_type) {
    if (!async_commands.isResult()) {
        if (FB_DEBUG_MODE) Serial.println("RequestCommands: aún no se han recibido.");
        return FB_Get_Result::NO_RESULT;
    }
    if (async_commands.isError()) {
        if (FB_DEBUG_MODE) Serial.println("RequestCommands: error al recibir.");
        return FB_Get_Result::ERROR;
    } 

    StaticJsonDocument<64> doc;
    DeserializationError err = deserializeJson(doc, async_commands.c_str());
    if (err) {
        if (FB_DEBUG_MODE) {
            Serial.print("RequestCommands: JSON parse error - ");
            Serial.println(err.c_str());
        }
        return FB_Get_Result::PARSE_ERROR;
    }

    if (!doc.containsKey("action") || !doc.containsKey("controller_type")) {
        if (FB_DEBUG_MODE) Serial.println("RequestCommands: Faltan campos requeridos.");
        return FB_Get_Result::MISSING_FIELDS;
    }

    action = doc["action"].as<int>();
    controller_type = doc["controller_type"].as<int>();
    return FB_Get_Result::OK;
}


void RequestPendingWaypoint() {
    DatabaseOptions options;
    options.filter.orderBy("$key").limitToFirst(1);
    Database.get(async_client, "/waypoints_pending", options, async_pending_waypoints);
    if (FB_DEBUG_MODE) Serial.println("RequestPendingWaypoint: solicitud enviada.");
}

FB_Get_Result ProcessPendingWaypoint(TargetPoint &target_out) {
    // Revisar si hay un resultado disponible
    if (!async_pending_waypoints.isResult()) {
        if (FB_DEBUG_MODE) Serial.println("RequestPendingWaypoint: no hay resultado disponible.");
        return FB_Get_Result::NO_RESULT;
    }
    // Hubo resultado, revisar si hay error
    if (async_pending_waypoints.isError()) {
        if (FB_DEBUG_MODE) Serial.println("RequestPendingWaypoint: error al obtener.");
        return FB_Get_Result::ERROR;
    }
    // No hay error, deserializar y revisar si hay datos
    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, async_pending_waypoints.c_str());
    if (err) {
        if (FB_DEBUG_MODE) {
            Serial.print("RequestPendingWaypoint: JSON parse error - ");
            Serial.println(err.c_str());
        }
        return FB_Get_Result::PARSE_ERROR;
    }
    // Obtener los datos del JSON y avisar si están completos
    JsonObject root = doc.as<JsonObject>();
    for (JsonPair kv : root) {
        JsonObject obj = kv.value().as<JsonObject>();
        if (!obj.containsKey("wp_x") || !obj.containsKey("wp_y") || !obj.containsKey("input_timestamp") || 
                    obj["wp_x"].isNull() || obj["wp_y"].isNull() || obj["input_timestamp"].isNull()) {
            if (FB_DEBUG_MODE) Serial.println("RequestPendingWaypoint: faltan campos requeridos.");
            return FB_Get_Result::MISSING_FIELDS;
        }
        target_out.x = obj["wp_x"];
        target_out.y = obj["wp_y"];
        target_out.ts = String(kv.key().c_str()).toInt();
        return FB_Get_Result::OK;
    }
}


void PushStatus(
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
    doc["pos_x"] = x;
    doc["pos_y"] = y;
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
}


void PushReachedWaypoint(
    const float wp_x, const float wp_y, const uint32_t input_timestamp, 
    const float pos_x, const float pos_y, 
    const uint32_t start_timestamp, const uint32_t reached_timestamp, const uint32_t trip_length_sec
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

    doc["pos_x"] = pos_x;
    doc["pos_y"] = pos_y;

    doc["wp_x"] = wp_x;
    doc["wp_y"] = wp_y;

    String path = "/waypoints_reached/" + String(reached_timestamp);
    SetJson(path, doc, async_reached_waypoints);
}


void RemovePendingWaypoint(const uint32_t input_timestamp) {
    String path = "/waypoints_pending/" + String(input_timestamp);
    Database.remove(async_client, path, async_pending_waypoints);
}


FB_Push_Result ProcessPush(const FB_PushType type) {
    static uint8_t status_errors = 0;
    static uint8_t reached_errors = 0;
    static uint8_t remove_errors = 0;

    AsyncResult* async_result_ptr = nullptr;
    uint8_t* error_counter = nullptr;
    const char* label = nullptr;
    uint8_t max_errors = 10;

    switch (type) {
        case FB_PushType::STATUS:
            async_result_ptr = &async_status;
            error_counter = &status_errors;
            label = "PushStatus";
            max_errors = PUSH_STATUS_MAX_ERRORS;
            break;
        case FB_PushType::REACHED:
            async_result_ptr = &async_reached_waypoints;
            error_counter = &reached_errors;
            label = "PushReachedWaypoint";
            max_errors = PUSH_REACHED_MAX_ERRORS;
            break;
        case FB_PushType::REMOVE_PENDING:
            async_result_ptr = &async_pending_waypoints;
            error_counter = &remove_errors;
            label = "RemovePendingWaypoint";
            max_errors = PUSH_REMOVE_MAX_ERRORS;
            break;
        default:
            return FB_Push_Result::FATAL_ERROR;
    }

    if (async_result_ptr->isResult()) {
        if (!async_result_ptr->isError()) {
            *error_counter = 0;
            if (FB_DEBUG_MODE) Serial.printf("%s: Éxito.\n", label);
            return FB_Push_Result::OK;
        } else {
            if (*error_counter < max_errors) {
                if (FB_DEBUG_MODE) Serial.printf("%s: Error al subir.\n", label);
                (*error_counter)++;
                return FB_Push_Result::ERROR;
            } else {
                if (FB_DEBUG_MODE) Serial.printf("%s: Se alcanzó el máximo de errores permitidos.\n", label);
                *error_counter = 0;
                return FB_Push_Result::FATAL_ERROR;
            }
        }
    } else {
        return FB_Push_Result::NOT_READY;
    }
}


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
