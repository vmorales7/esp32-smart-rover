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

bool ConnectFirebase() {
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


FB_State UpdateCommands(
    volatile uint8_t &action, volatile uint8_t &controller_type, 
    volatile FB_State &fb_state
) {
    static bool request_in_flight = false;
    static uint32_t time_request_sent = 0;
    static uint8_t error_count = 0;

    if (!request_in_flight) {
        RequestCommands();
        request_in_flight = true;
        time_request_sent = millis();
        return FB_State::PENDING;;
    }

    // Ya se hizo un request, ahora intentamos procesar
    FB_Get_Result result = ProcessRequestCommands(action, controller_type);
    if (result == FB_Get_Result::OK) {
        request_in_flight = false;
        error_count = 0;
        return FB_State::OK;
    } 
    else if (result == FB_Get_Result::NO_RESULT) {
        if (millis() - time_request_sent > FB_COMMANDS_TIMEOUT_MS) {
            error_count++;
            request_in_flight = false;
        }
    } 
    else {
        error_count++;
        request_in_flight = false;
    }
    if (error_count >= FB_COMMANDS_MAX_ERRORS) {
        if (FB_DEBUG_MODE) Serial.println("UpdateCommands: se alcanzó el máximo de errores.");
        fb_state = FB_State::ERROR;
        error_count = 0;
        return FB_State::ERROR;
    }
    return FB_State::PENDING;
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

FB_State UpdatePendingWaypoint(
    volatile float &target_x, volatile float &target_y, volatile uint64_t &target_ts,
    volatile FB_State &fb_state
) {
    static bool request_in_flight = false;
    static uint32_t time_request_sent = 0;
    static uint8_t error_count = 0;

    // Si no hay solicitud en curso, iniciar una nueva y marcar el tiempo
    if (!request_in_flight) {
        FirebaseComm::RequestPendingWaypoint();
        request_in_flight = true;
        time_request_sent = millis();
        return FB_State::PENDING;
    }

    // Si ya hay una solicitud en curso, procesar el resultado
    FB_Get_Result result = FirebaseComm::ProcessPendingWaypoint(target_x, target_y, target_ts);

    // Resultado ok
    if (result == FB_Get_Result::OK) {
        request_in_flight = false;
        error_count = 0;
        // Validar que los datos no sean nulos o inválidos
        const bool invalid_data = (
            target_ts == NULL_TIMESTAMP || target_x == NULL_WAYPOINT_XY || target_y == NULL_WAYPOINT_XY);
        if (invalid_data) {
            if (FB_DEBUG_MODE) Serial.println("UpdatePendingWaypoint: datos nulos, removiendo...");
            FirebaseComm::RemovePendingWaypoint(target_ts); // Intentar remover el punto inválido
            return FB_State::PENDING; // Se vuelve a intentar en el siguiente ciclo
        }
        return FB_State::OK;
    }
    // No hay resultado aún
    else if (result == FB_Get_Result::NO_RESULT) {
        // Si hay timeout en la solicitud, incrementar el contador de errores
        if (millis() - time_request_sent > FB_PENDING_TIMEOUT_MS) {
            error_count++;
            request_in_flight = false;
        }
    } 
    // Resultado con error: se suma un error y se fuerza una nueva solicitud
    else {
        error_count++;
        request_in_flight = false;
    }
    // Verificar si se alcanzó el máximo de errores
    if (error_count >= FB_PENDING_MAX_ERRORS) {
        if (FB_DEBUG_MODE) Serial.println("UpdatePendingWaypoint: se alcanzó el máximo de errores.");
        fb_state = FB_State::ERROR;
        error_count = 0;
        request_in_flight = false;
        return FB_State::ERROR;
    }
    // Si no es error ni ok, se mantiene en espera
    return FB_State::PENDING;
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

FB_State ControlledPushReachedWaypoint(
    const uint64_t input_timestamp, const float wp_x, const float wp_y,  
    const uint64_t start_timestamp, const uint64_t reached_timestamp,
    const float pos_x, const float pos_y, 
    const uint8_t controller_type, const float iae, const float rmse,
    volatile FB_State &fb_state
) {
    static bool in_flight = false;
    static uint32_t time_sent = 0;
    static uint8_t error_count = 0;

    // Iniciar push si no hay operación en curso
    if (!in_flight) {
        PushReachedWaypoint(
            input_timestamp, wp_x, wp_y, start_timestamp, reached_timestamp,
            pos_x, pos_y, controller_type, iae, rmse
        );
        time_sent = millis();
        in_flight = true;
        return FB_State::PENDING;
    }

    // Si ya está en curso, revisar resultado
    if (async_reached_waypoints.isResult()) {
        if (!async_reached_waypoints.isError()) {
            error_count = 0;
            in_flight = false;
            return FB_State::OK;
        } else {
            error_count++;
            in_flight = false;
        }
    } else if (millis() - time_sent > FB_PUSH_REACHED_TIMEOUT_MS) {
        error_count++;
        in_flight = false;
    }

    // Verificar si se alcanzó el máximo de errores
    if (error_count >= FB_PUSH_REACHED_MAX_ERRORS) {
        error_count = 0;
        if (FB_DEBUG_MODE) Serial.println("ControlledPushReachedWaypoint: error permanente.");
        fb_state = FB_State::ERROR;
        return FB_State::ERROR;
    }
    return FB_State::PENDING;
}


void RemovePendingWaypoint(const uint64_t input_timestamp) {
    String path = "/waypoints_pending/" + String(input_timestamp);
    Database.remove(async_client, path, async_pending_waypoints);
}

FB_State ControlledRemovePendingWaypoint(
    const uint64_t input_timestamp, volatile FB_State &fb_state
) {
    static bool in_flight = false;
    static uint32_t time_sent = 0;
    static uint8_t error_count = 0;

    // Iniciar operación si no hay otra en curso
    if (!in_flight) {
        RemovePendingWaypoint(input_timestamp);
        time_sent = millis();
        in_flight = true;
        return FB_State::PENDING;
    }

    // Evaluar el resultado si ya se hizo la solicitud
    if (async_pending_waypoints.isResult()) {
        if (!async_pending_waypoints.isError()) {
            error_count = 0;
            in_flight = false;
            return FB_State::OK;
        } else {
            error_count++;
            in_flight = false;
        }
    } else if (millis() - time_sent > FB_PUSH_REMOVE_TIMEOUT_MS) {
        error_count++;
        in_flight = false;
    }

    // Evaluar si se alcanzó el máximo de errores
    if (error_count >= FB_PUSH_REMOVE_ERRORS) {
        error_count = 0;
        if (FB_DEBUG_MODE) Serial.println("ControlledRemovePendingWaypoint: error permanente.");
        fb_state = FB_State::ERROR;
        return FB_State::ERROR;
    }
    return FB_State::PENDING;
}


FB_State CompleteWaypoint(
    const uint64_t input_ts, const float wp_x, const float wp_y,
    const uint64_t start_ts, const uint64_t reached_ts,
    const float pos_x, const float pos_y,
    const uint8_t controller_type, const float iae, const float rmse,
    volatile FB_State& fb_state
) {
    static enum class InternalStep : uint8_t { PUSH_REACHED, REMOVE_PENDING } step = InternalStep::PUSH_REACHED;
    static FB_State push_state = FB_State::PENDING;
    static FB_State remove_state = FB_State::PENDING;

    switch (step) {
        case InternalStep::PUSH_REACHED:
            push_state = ControlledPushReachedWaypoint(
                input_ts, wp_x, wp_y,
                start_ts, reached_ts,
                pos_x, pos_y,
                controller_type, iae, rmse,
                fb_state
            );
            if (push_state == FB_State::OK) {
                step = InternalStep::REMOVE_PENDING;
            } else if (push_state == FB_State::ERROR) {
                step = InternalStep::PUSH_REACHED;
                return FB_State::ERROR;
            }
            return FB_State::PENDING;

        case InternalStep::REMOVE_PENDING:
            remove_state = ControlledRemovePendingWaypoint(input_ts, fb_state);
            if (remove_state == FB_State::OK) {
                step = InternalStep::PUSH_REACHED;
                return FB_State::OK;  // Se completó correctamente
            } else if (remove_state == FB_State::ERROR) {
                step = InternalStep::PUSH_REACHED;
                return FB_State::ERROR;
            }
            return FB_State::PENDING;
    }
    return FB_State::ERROR;
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

void Task_PushStatus(void *pvParameters) {
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

void Task_GetCommands(void *pvParameters) {
    // Configuración del periodo de muestreo
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(FB_GET_COMMANDS_PERIOD_MS);
    // Recuperar variables globales
    GlobalContext* ctx_ptr = static_cast<GlobalContext*>(pvParameters);
    volatile OperationData& os = *(ctx_ptr->os_ptr);
    // Ejecutar tarea periodicamente
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        uint8_t action = 0;
        uint8_t controller_type = 0;
        if (UpdateCommands(action, controller_type, os.fb_state) == FB_State::OK) {
            switch (action) {
                case 0: os.fb_last_command = UserCommand::STOP; break;
                case 1: os.fb_last_command = UserCommand::START; break;
                case 2: os.fb_last_command = UserCommand::IDLE; break;
                default:
                    os.fb_last_command = UserCommand::STOP;
                    os.fb_state = FB_State::ERROR;
                    if (FB_DEBUG_MODE) Serial.printf("Task_FirebaseGetCommands: acción inválida: %u\n", action);
                    break;
            }
            switch (controller_type) {
                case 0: os.fb_controller_type = ControlType::PID; break;
                case 1: os.fb_controller_type = ControlType::BACKS; break;
                default:
                    os.fb_controller_type = ControlType::PID;  // Valor seguro por defecto
                    os.fb_state = FB_State::ERROR;
                    if (FB_DEBUG_MODE) Serial.printf("Task_FirebaseGetCommands: controlador inválido: %u\n", controller_type);
                    break;
            }
        }
    }
}

void Task_Loop(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(FB_LOOP_PERIOD_MS);
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        ready(); // Llamar a ready() para mantener la conexión activa
    }
}

} // namespace FirebaseComm
