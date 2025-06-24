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


bool SetupFirebaseConnect() {
    if (FB_DEBUG_MODE) Serial.println("\nSetupFirebaseConnect: Iniciando conexión a Firebase...");
    ssl_client.setInsecure();
    initializeApp(async_client, app, getAuth(user_auth), auth_debug_print, "authTask");
    app.getApp<RealtimeDatabase>(Database);
    Database.url(FB_DATABASE_URL);
    
    // Esperar conexión y autenticación
    while (!ready()) {
        if (FB_DEBUG_MODE) Serial.print(".");
        delay(1000);
    }
    if (FB_DEBUG_MODE) Serial.println("\nSetupFirebaseConnect: Conexión a Firebase establecida.");
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

FB_Get_Result ProcessRequestCommands(volatile uint8_t &action, volatile uint8_t &controller_type) {
    // Revisar si hay un resultado disponible
    if (!async_commands.isResult()) {
        if (FB_DEBUG_MODE) Serial.println("RequestCommands: aún no se han recibido.");
        return FB_Get_Result::NO_RESULT;
    }
    // Hubo resultado, revisar si hay error
    if (async_commands.isError()) {
        if (FB_DEBUG_MODE) Serial.println("RequestCommands: error al recibir.");
        return FB_Get_Result::ERROR;
    } 
    // No hay error, deserializar y revisar si hay datos
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, async_commands.c_str());
    if (err) {
        if (FB_DEBUG_MODE) {
            Serial.print("RequestCommands: JSON parse error - ");
            Serial.println(err.c_str());
        }
        return FB_Get_Result::PARSE_ERROR;
    }
    // Obtener los datos del JSON y avisar si están completos
    JsonVariant v_action = doc["action"];
    JsonVariant v_ctrl   = doc["controller_type"];
    if (!v_action.is<double>() || !v_ctrl.is<double>()) {
        if (FB_DEBUG_MODE) Serial.println("RequestCommands: faltan campos requeridos o tipo incorrecto.");
        return FB_Get_Result::MISSING_FIELDS;
    }
    action = v_action.as<uint8_t>();
    controller_type = v_ctrl.as<uint8_t>();
    return FB_Get_Result::OK;
}


void RequestPendingWaypoint() {
    DatabaseOptions options;
    options.filter.orderBy("$key").limitToFirst(1);
    Database.get(async_client, "/waypoints_pending", options, async_pending_waypoints);
    if (FB_DEBUG_MODE) Serial.println("RequestPendingWaypoint: solicitud enviada.");
}

FB_Get_Result ProcessPendingWaypoint(
    volatile float &target_x, volatile float &target_y, volatile uint64_t &target_ts
) {
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
    JsonDocument doc;
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

        JsonVariant v_x  = obj["wp_x"];
        JsonVariant v_y  = obj["wp_y"];
        JsonVariant v_ts = obj["input_timestamp"];

        if (!v_x.is<double>() || !v_y.is<double>() || !v_ts.is<double>()) {
            if (FB_DEBUG_MODE) Serial.println("RequestPendingWaypoint: faltan campos.");
            return FB_Get_Result::MISSING_FIELDS;
        }

        target_x = v_x.as<float>();
        target_y = v_y.as<float>();
        target_ts = v_ts.as<uint64_t>();
        return FB_Get_Result::OK;
    }
    // Si no se encuentra ningún elemento válido
    if (FB_DEBUG_MODE) Serial.println("RequestPendingWaypoint: JSON vacío o sin elementos válidos.");
    return FB_Get_Result::MISSING_FIELDS;
}


void PushStatus(
    const uint8_t state, const char* log_msg, 
    const float x, const float y, const float wL, const float wR,
    const uint64_t wp_input_ts, const float wp_x, const float wp_y,
    const uint8_t controller_type, const float iae, const float rmse
) {
    const uint32_t timestamp = get_unix_timestamp();

    const int rpm_L = round(wL * 60.0 / (2 * 3.14)) + 0.5;
    const int rpm_R = round(wR * 60.0 / (2 * 3.14)) + 0.5;

    JsonDocument doc;
    doc["timestamp"] = timestamp;
    doc["state"] = state;
    doc["pos_x"] = x;
    doc["pos_y"] = y;
    doc["rpm_left"] = rpm_L;
    doc["rpm_right"] = rpm_R;
    doc["log_msg"] = log_msg;
    doc["wp_input_timestamp"] = wp_input_ts;
    doc["wp_x"] = wp_x;
    doc["wp_y"] = wp_y;
    doc["controller_type"] = controller_type;
    doc["iae"] = iae;
    doc["rmse"] = rmse;

    String path = "/status_log/" + String(timestamp);
    SetJson(path, doc, async_status);
}


void PushReachedWaypoint(
    const uint64_t input_timestamp, const float wp_x, const float wp_y,  
    const uint64_t start_timestamp, const uint64_t reached_timestamp,
    const float pos_x, const float pos_y, 
    const uint8_t controller_type, const float iae, const float rmse
) {
    JsonDocument doc;
    doc["input_timestamp"] = input_timestamp;
    doc["start_timestamp"] = start_timestamp;
    doc["reached_timestamp"] = reached_timestamp;

    doc["pos_x"] = pos_x;
    doc["pos_y"] = pos_y;

    doc["wp_x"] = wp_x;
    doc["wp_y"] = wp_y;

    doc["controller_type"] = controller_type;
    doc["iae"] = iae;
    doc["rmse"] = rmse;

    String path = "/waypoints_reached/" + String(reached_timestamp);
    SetJson(path, doc, async_reached_waypoints);
}


void RemovePendingWaypoint(const uint64_t input_timestamp) {
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

    FirebaseComm::ready();

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


void auth_debug_print(AsyncResult &aResult) {
    if (FB_DEBUG_MODE) {    
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
}

void Task_FirebasePushStatus(void *pvParameters) {
    // Configuración del periodo de muestreo
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(FB_PUSH_STATUS_PERIOD_MS);
    // Recuperar variables globales
    GlobalContext* ctx_ptr = static_cast<GlobalContext*>(pvParameters);
    volatile OperationData& os = *(ctx_ptr->os_ptr);
    volatile PoseData& pose = *(ctx_ptr->pose_ptr);
    volatile ControllerData& ctrl = *(ctx_ptr->control_ptr);
    // Ejecutar tarea periodicamente
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        if (os.state != OS_State::INIT && os.state != OS_State::IDLE) {
            PushStatus(
                static_cast<uint8_t>(os.state), (const char*) os.last_log,
                pose.x, pose.y, pose.w_L, pose.w_R,
                os.fb_waypoint_data.input_ts, os.fb_waypoint_data.wp_x, os.fb_waypoint_data.wp_y,
                static_cast<uint8_t>(ctrl.controller_type), ctrl.iae, ctrl.rmse
            );
        }
    }
}

} // namespace FirebaseComm
