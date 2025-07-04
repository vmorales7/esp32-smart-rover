#include "firebase_comm.h"

namespace FirebaseComm {

// Objetos que se generan de forma diferida para evitar problemas de inicialización
static WiFiClientSecure*  ssl_client_ptr   = nullptr;
static AsyncClientClass*  async_client_ptr = nullptr;
static UserAuth*          user_auth_ptr    = nullptr;
static FirebaseApp*       app_ptr          = nullptr;
static RealtimeDatabase*  rtdb_ptr         = nullptr;

// Estados para operaciones asíncronas
static AsyncResult async_commands;
static AsyncResult async_status;
static AsyncResult async_pending_waypoints;
static AsyncResult async_reached_waypoints;

bool ConnectFirebase() {
    static bool already_initialized = false;
    if (already_initialized) return true;
    
    // Limpia instancias anteriores (seguro si no estaban inicializadas)
    if (FB_DEBUG_MODE) Serial.println("\n[Firebase] Iniciando conexión...");
    // ResetFirebase();

    // Crear instancias dinámicamente si aún no existen
    if (!ssl_client_ptr)   ssl_client_ptr   = new WiFiClientSecure();
    if (!async_client_ptr) async_client_ptr = new AsyncClientClass(*ssl_client_ptr);
    if (!user_auth_ptr)    user_auth_ptr    = new UserAuth(FB_DATABASE_API_KEY, FB_USER_EMAIL, FB_USER_PASSWORD, 3000);
    if (!app_ptr)          app_ptr          = new FirebaseApp();
    if (!rtdb_ptr)         rtdb_ptr         = new RealtimeDatabase();

    // Inicializar app y base de datos
    ssl_client_ptr->setInsecure();
    initializeApp(*async_client_ptr, *app_ptr, getAuth(*user_auth_ptr), auth_debug_print, "authTask");
    app_ptr->getApp<RealtimeDatabase>(*rtdb_ptr);
    rtdb_ptr->url(FB_DATABASE_URL);

    // Esperar conexión completa
    while (!ready()) {
        if (FB_DEBUG_MODE) Serial.print(".");
        delay(1000);
    }
    already_initialized = true;

    if (FB_DEBUG_MODE) Serial.println("\n[Firebase] Conexión establecida.");
    return true;
}

void ResetFirebase() {
    if (FB_DEBUG_MODE) Serial.println("[Firebase] Reiniciando objetos de Firebase...");

    if (rtdb_ptr) {
        delete rtdb_ptr;
        rtdb_ptr = nullptr;
    }
    if (app_ptr) {
        delete app_ptr;
        app_ptr = nullptr;
    }
    if (user_auth_ptr) {
        delete user_auth_ptr;
        user_auth_ptr = nullptr;
    }
    if (async_client_ptr) {
        delete async_client_ptr;
        async_client_ptr = nullptr;
    }
    if (ssl_client_ptr) {
        delete ssl_client_ptr;
        ssl_client_ptr = nullptr;
    }

    if (FB_DEBUG_MODE) Serial.println("[Firebase] Objetos eliminados.");
}

bool ready() {
    if (!app_ptr || !rtdb_ptr) return false;
    bool initialized = app_ptr->isInitialized();
    if (initialized) app_ptr->loop();
    return initialized && app_ptr->ready();
}


void SetJson(const String &path, const JsonDocument &doc, AsyncResult &result) {
    String json;
    if (serializeJson(doc, json) == 0) {
        if (FB_DEBUG_MODE) Serial.println("[Firebase] Error al serializar JSON.");
        return;
    }
    object_t obj(json);
    rtdb_ptr->set<object_t>(*async_client_ptr, path, obj, result);
}


void RequestCommands() {
    rtdb_ptr->get(*async_client_ptr, "/commands", async_commands);
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
        if (FB_DEBUG_MODE) {
            Serial.println("RequestCommands: error al recibir.");
            Serial.print(async_commands.error().code());
            Serial.print(" | ");
            Serial.println(async_commands.error().message().c_str());
        }
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
    if (FB_DEBUG_MODE) {
        Serial.printf("RequestCommands: datos recibidos correctamente -> action = %u, controller_type = %u\n", 
            action, controller_type);
    }
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
        return FB_State::PENDING;
    }

    // Ya se hizo un request, ahora intentamos procesar
    FB_Get_Result result = ProcessRequestCommands(action, controller_type);
    if (result == FB_Get_Result::OK) {
        request_in_flight = false;
        error_count = 0;
        return FB_State::OK;
    } 
    else if (result == FB_Get_Result::NO_RESULT) 
    {
        if (millis() - time_request_sent > FB_COMMANDS_TIMEOUT_MS) {
            error_count++;
            request_in_flight = false;
        }
    } 
    else 
    {   // Resultado con error: se suma un error y se fuerza una nueva solicitud
        if (FB_DEBUG_MODE) Serial.println("UpdateCommands: error al recibir comandos.");
        error_count++;
        request_in_flight = false;
    }

    if (error_count >= FB_COMMANDS_MAX_ERRORS) 
    {
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
    rtdb_ptr->get(*async_client_ptr, "/waypoints_pending", options, async_pending_waypoints);
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
    for (JsonPair kv : root) 
    {
        JsonObject obj = kv.value().as<JsonObject>();
        JsonVariant v_x  = obj["wp_x"];
        JsonVariant v_y  = obj["wp_y"];
        JsonVariant v_ts = obj["input_timestamp"];
        if (!v_x.is<double>() || !v_y.is<double>() || !v_ts.is<double>()) {
            if (FB_DEBUG_MODE) Serial.println("RequestPendingWaypoint: faltan campos.");
            return FB_Get_Result::MISSING_FIELDS;
        }
        // Si están los valores
        target_x = v_x.as<float>();
        target_y = v_y.as<float>();
        target_ts = v_ts.as<uint64_t>();
        if (FB_DEBUG_MODE) {
            Serial.printf("RequestPendingWaypoint: datos recibidos -> wp_x = %.2f, wp_y = %.2f, input_timestamp = %llu\n", 
                target_x, target_y, target_ts);
        }
        // Validar que los datos no sean nulos o inválidos
        const bool invalid_data = (
            target_ts == NULL_TIMESTAMP || target_x == NULL_WAYPOINT_XY || target_y == NULL_WAYPOINT_XY);
        if (invalid_data) {
            if (FB_DEBUG_MODE) Serial.println("RequestPendingWaypoint: datos nulos, removiendo...");
            String path = "/waypoints_pending/" + String(target_ts);
            AsyncResult async_dummy;
            rtdb_ptr->remove(*async_client_ptr, path, async_dummy);
            return FB_Get_Result::NO_RESULT; // Se vuelve a intentar en el siguiente ciclo
        }
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
        if (FB_DEBUG_MODE) Serial.println("RequestPendingWaypoint: se alcanzó el máximo de errores.");
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
    const uint64_t start_timestamp, const uint64_t end_timestamp, const bool reached_flag,
    const float pos_x, const float pos_y, 
    const uint8_t controller_type, const float iae, const float rmse
) {
    JsonDocument doc;
    doc["input_timestamp"] = input_timestamp;
    doc["wp_x"] = wp_x;
    doc["wp_y"] = wp_y;

    doc["start_timestamp"] = start_timestamp;
    doc["end_timestamp"] = end_timestamp;
    doc["reached"] = reached_flag;
    doc["pos_x"] = pos_x;
    doc["pos_y"] = pos_y;

    doc["controller_type"] = controller_type;
    doc["iae"] = iae;
    doc["rmse"] = rmse;

    String path = "/waypoints_finalized/" + String(end_timestamp);
    SetJson(path, doc, async_reached_waypoints);
}

FB_State ControlledPushReachedWaypoint(
    const uint64_t input_timestamp, const float wp_x, const float wp_y,  
    const uint64_t start_timestamp, const uint64_t end_timestamp, const bool reached_flag,
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
            input_timestamp, wp_x, wp_y, start_timestamp, end_timestamp,
            reached_flag, pos_x, pos_y, controller_type, iae, rmse
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
    rtdb_ptr->remove(*async_client_ptr, path, async_pending_waypoints);
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
    if (error_count >= FB_PUSH_REMOVE_MAX_ERRORS) {
        error_count = 0;
        if (FB_DEBUG_MODE) Serial.println("ControlledRemovePendingWaypoint: error permanente.");
        fb_state = FB_State::ERROR;
        return FB_State::ERROR;
    }
    return FB_State::PENDING;
}


FB_State CompleteWaypoint(
    const uint64_t input_ts, const float wp_x, const float wp_y,
    const uint64_t start_ts, const uint64_t end_ts, const bool reached_flag,
    const float pos_x, const float pos_y,
    const uint8_t controller_type, const float iae, const float rmse,
    volatile FB_State& fb_state
) {
    static FB_State push_state = FB_State::PENDING;
    static FB_State remove_state = FB_State::PENDING;

    // Llamar a ambos controlled en cada ciclo — ellos manejan sus propios in_flight y retries
    push_state = ControlledPushReachedWaypoint(
        input_ts, wp_x, wp_y, start_ts, end_ts, reached_flag,
        pos_x, pos_y, controller_type, iae, rmse, fb_state
    );
    remove_state = ControlledRemovePendingWaypoint(input_ts, fb_state);

    // Esperar a que ambas operaciones terminen
    if (push_state == FB_State::PENDING || remove_state == FB_State::PENDING) return FB_State::PENDING;

    // Evaluar resultado
    if (remove_state == FB_State::OK &&
        (push_state == FB_State::OK || push_state == FB_State::ERROR)) {
        fb_state = FB_State::OK;
        return FB_State::OK;
    }

    fb_state = FB_State::ERROR;
    return FB_State::ERROR;
}


void PushStatus(
    const uint8_t state, const char* log_msg, 
    const float x, const float y, const float wL, const float wR,
    const uint64_t wp_input_ts, const float wp_x, const float wp_y,
    const uint8_t controller_type, const float iae, const float rmse
) {
    const uint32_t timestamp = get_unix_timestamp();
    const int rpm_L = lround(wL * 60.0f / (2.0f * M_PI));
    const int rpm_R = lround(wR * 60.0f / (2.0f * M_PI));

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


void ForceCommandIdle() {
    JsonDocument doc;
    doc["action"] = 2;           // 2 = IDLE
    doc["controller_type"] = 0;  // 0 = PID
    String path = "/commands";
    AsyncResult dummy_result;
    SetJson(path, doc, dummy_result);  
    if (FB_DEBUG_MODE) Serial.println("[Firebase] Comando forzado a IDLE y PID en /commands");
}

FB_State ClearAllLogs(volatile FB_State& fb_state) {
    static bool request_sent = false;
    static bool status_done = false;
    static bool finalized_done = false;
    static uint32_t time_sent = 0;
    static uint8_t error_count = 0;

    // 1. Iniciar operación si no se ha enviado aún
    if (!request_sent) {
        if (FB_DEBUG_MODE) Serial.println("\nClearAllLogs: Enviando solicitudes de eliminación...");
        rtdb_ptr->remove(*async_client_ptr, "/status_log", async_status);
        rtdb_ptr->remove(*async_client_ptr, "/waypoints_finalized", async_reached_waypoints);
        request_sent   = true;
        status_done    = false;
        finalized_done = false;
        time_sent      = millis();
        return FB_State::PENDING;
    }

    // 2. Revisar resultado de /status_log
    if (!status_done && async_status.isResult()) {
        if (async_status.isError()) {
            if (FB_DEBUG_MODE) {
                Serial.print("ClearAllLogs: Error al eliminar /status_log -> ");
                Serial.println(async_status.error().message().c_str());
            }
            error_count++;
        } else {
            if (FB_DEBUG_MODE) Serial.println("ClearAllLogs: /status_log eliminado correctamente.");
        }
        status_done = true;
    }

    // 3. Revisar resultado de /waypoints_finalized
    if (!finalized_done && async_reached_waypoints.isResult()) {
        if (async_reached_waypoints.isError()) {
            if (FB_DEBUG_MODE) {
                Serial.print("ClearAllLogs: Error al eliminar /waypoints_finalized -> ");
                Serial.println(async_reached_waypoints.error().message().c_str());
            }
            error_count++;
        } else {
            if (FB_DEBUG_MODE) Serial.println("ClearAllLogs: /waypoints_finalized eliminado correctamente.");
        }
        finalized_done = true;
    }

    // 4. Evaluar si ambas terminaron
    if (status_done && finalized_done) {
        request_sent = false;

        if (error_count > FB_PUSH_CLEAR_MAX_ERRORS) {
            fb_state = FB_State::ERROR;
            error_count = 0;
            if (FB_DEBUG_MODE) Serial.println("ClearAllLogs: Eliminación fallida.");
            return FB_State::ERROR;
        } else {
            fb_state = FB_State::OK;
            if (FB_DEBUG_MODE) Serial.println("ClearAllLogs: Eliminación completada con éxito.");
            return FB_State::OK;
        }
    }

    // 5. Timeout global
    if (millis() - time_sent > FB_PUSH_CLEAR_TIMEOUT_MS) {
        error_count++;
        request_sent = false;

        if (FB_DEBUG_MODE) Serial.println("ClearAllLogs: Timeout alcanzado. Reintentando...");

        if (error_count >= FB_PUSH_CLEAR_MAX_ERRORS) {
            fb_state = FB_State::ERROR;
            error_count = 0;
            if (FB_DEBUG_MODE) Serial.println("ClearAllLogs: Demasiados errores acumulados. Abortando.");
            return FB_State::ERROR;
        }
    }

    return FB_State::PENDING;
}

FB_State ClearPendingWaypoints(volatile FB_State& fb_state) {
    static bool request_sent = false;
    static uint32_t time_sent = 0;
    static uint8_t error_count = 0;

    if (!request_sent) {
        if (FB_DEBUG_MODE) Serial.println("ClearPendingWaypoints: Enviando solicitud de eliminación...");
        rtdb_ptr->remove(*async_client_ptr, "/waypoints_pending", async_pending_waypoints);
        request_sent = true;
        time_sent = millis();
        return FB_State::PENDING;
    }
    if (async_pending_waypoints.isResult()) 
    {   // Ya se recibió un resultado, revisar si hay error
        request_sent = false;
        if (async_pending_waypoints.isError()) {
            error_count++;
            if (FB_DEBUG_MODE) {
                Serial.print("ClearPendingWaypoints: Error -> ");
                Serial.println(async_pending_waypoints.error().message().c_str());
            }
        } else {
            if (FB_DEBUG_MODE) Serial.println("ClearPendingWaypoints: Eliminación exitosa.");
            fb_state = FB_State::OK;
            error_count = 0;
            return FB_State::OK;
        }
    }
    else if (FB_DEBUG_MODE) 
    {   // Si aún no se ha recibido resultado
        Serial.println("ClearPendingWaypoints: Aún no se ha recibido resultado.");
    }

    if (millis() - time_sent > FB_PUSH_CLEAR_TIMEOUT_MS) {
        request_sent = false;
        error_count++;
        if (FB_DEBUG_MODE) Serial.println("ClearPendingWaypoints: Timeout alcanzado. Reintentando...");
    }
    if (error_count >= FB_PUSH_CLEAR_MAX_ERRORS) {
        error_count = 0;
        fb_state = FB_State::ERROR;
        if (FB_DEBUG_MODE) Serial.println("ClearPendingWaypoints: Demasiados errores. Abortando.");
        return FB_State::ERROR;
    }
    return FB_State::PENDING;
}

FB_State FullReset(volatile FB_State& fb_state) {
    // Estado interno de la operación
    static bool request_sent      = false;
    static bool status_done       = false;
    static bool finalized_done    = false;
    static bool pendings_done     = false;
    static bool command_idle_done = false;
    static uint32_t time_sent     = 0;
    static uint8_t error_count    = 0;

    // 1. Enviar solicitudes solo la primera vez
    if (!request_sent) {
        if (FB_DEBUG_MODE) Serial.println("\nFullReset: Enviando solicitudes de eliminación global...");
        rtdb_ptr->remove(*async_client_ptr, "/status_log", async_status);
        rtdb_ptr->remove(*async_client_ptr, "/waypoints_finalized", async_reached_waypoints);
        rtdb_ptr->remove(*async_client_ptr, "/waypoints_pending", async_pending_waypoints);
        request_sent      = true;
        status_done       = false;
        finalized_done    = false;
        pendings_done     = false;
        command_idle_done = false;
        time_sent         = millis();
        error_count       = 0;
        return FB_State::PENDING;
    }

    // 2. Revisar /status_log
    if (!status_done && async_status.isResult()) {
        if (async_status.isError()) {
            if (FB_DEBUG_MODE) {
                Serial.print("FullReset: Error al eliminar /status_log -> ");
                Serial.println(async_status.error().message().c_str());
            }
            error_count++;
        } else {
            if (FB_DEBUG_MODE) Serial.println("FullReset: /status_log eliminado correctamente.");
        }
        status_done = true;
    }

    // 3. Revisar /waypoints_finalized
    if (!finalized_done && async_reached_waypoints.isResult()) {
        if (async_reached_waypoints.isError()) {
            if (FB_DEBUG_MODE) {
                Serial.print("FullReset: Error al eliminar /waypoints_finalized -> ");
                Serial.println(async_reached_waypoints.error().message().c_str());
            }
            error_count++;
        } else {
            if (FB_DEBUG_MODE) Serial.println("FullReset: /waypoints_finalized eliminado correctamente.");
        }
        finalized_done = true;
    }

    // 4. Revisar /waypoints_pending
    if (!pendings_done && async_pending_waypoints.isResult()) {
        if (async_pending_waypoints.isError()) {
            if (FB_DEBUG_MODE) {
                Serial.print("FullReset: Error al eliminar /waypoints_pending -> ");
                Serial.println(async_pending_waypoints.error().message().c_str());
            }
            error_count++;
        } else {
            if (FB_DEBUG_MODE) Serial.println("FullReset: /waypoints_pending eliminado correctamente.");
        }
        pendings_done = true;
    }

    // 5. Cuando todo esté eliminado, forzar el comando a IDLE (solo una vez)
    if (status_done && finalized_done && pendings_done && !command_idle_done) {
        ForceCommandIdle(); // fire-and-forget, la interfaz se actualiza de inmediato
        command_idle_done = true;
        if (FB_DEBUG_MODE) Serial.println("FullReset: /commands forzado a IDLE.");
        // Puedes decidir si aquí retornas PENDING para dar tiempo a propagarse, pero generalmente OK de inmediato.
    }

    // 6. Si todas las operaciones terminaron, resetear flags y devolver OK
    if (status_done && finalized_done && pendings_done && command_idle_done) {
        request_sent      = false;
        status_done       = false;
        finalized_done    = false;
        pendings_done     = false;
        command_idle_done = false;
        error_count       = 0;
        fb_state = FB_State::OK;
        if (FB_DEBUG_MODE) Serial.println("FullReset: COMPLETADO exitosamente.");
        return FB_State::OK;
    }

    // 7. Timeout global
    if (millis() - time_sent > FB_PUSH_CLEAR_TIMEOUT_MS) {
        error_count++;
        request_sent      = false;
        status_done       = false;
        finalized_done    = false;
        pendings_done     = false;
        command_idle_done = false;
        if (FB_DEBUG_MODE) Serial.println("FullReset: Timeout alcanzado. Reintentando...");
        if (error_count >= FB_PUSH_CLEAR_MAX_ERRORS) {
            error_count = 0;
            fb_state = FB_State::ERROR;
            if (FB_DEBUG_MODE) Serial.println("FullReset: Demasiados errores acumulados. Abortando.");
            return FB_State::ERROR;
        }
    }

    return FB_State::PENDING;
}


void auth_debug_print(AsyncResult &aResult) {
    if (FB_DEBUG_MODE) {    
        if (aResult.isEvent()) {
            Firebase.printf("Firebase - Event task: %s, msg: %s, code: %d\n", aResult.uid().c_str(), aResult.eventLog().message().c_str(), aResult.eventLog().code());
        }
        if (aResult.isDebug()) {
            Firebase.printf("Firebase - Debug task: %s, msg: %s\n", aResult.uid().c_str(), aResult.debug().c_str());
        }
        if (aResult.isError()) {
            Firebase.printf("Firebase - Error task: %s, msg: %s, code: %d\n", aResult.uid().c_str(), aResult.error().message().c_str(), aResult.error().code());
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
            if (FB_DEBUG_MODE) {
                Serial.printf("Task_PushStatus: Estado %u, pos (%f, %f), RPMs (%d, %d), WP (%f, %f)\n",
                    static_cast<uint8_t>(os.state), pose.x, pose.y, 
                    static_cast<int>(round(pose.w_L * 60.0 / (2 * 3.14))),
                    static_cast<int>(round(pose.w_R * 60.0 / (2 * 3.14))),
                    os.fb_waypoint_data.wp_x, os.fb_waypoint_data.wp_y);
            }
        }
    }
}

// void Task_GetCommands(void *pvParameters) {
//     // Configuración del periodo de muestreo
//     TickType_t xLastWakeTime = xTaskGetTickCount();
//     const TickType_t period = pdMS_TO_TICKS(FB_GET_COMMANDS_PERIOD_MS);
//     // Recuperar variables globales
//     GlobalContext* ctx_ptr = static_cast<GlobalContext*>(pvParameters);
//     volatile OperationData& os = *(ctx_ptr->os_ptr);
//     // Ejecutar tarea periódicamente
//     for (;;) {
//         vTaskDelayUntil(&xLastWakeTime, period);
//         uint8_t action = 0;
//         uint8_t controller_type = 0;
//         if (os.state != OS_State::INIT) 
//         {
//             if (UpdateCommands(action, controller_type, os.fb_state) == FB_State::OK) 
//             {
//                 os.fb_last_command    = Int2Cmd(action);
//                 os.fb_controller_type = Int2CtrlType(controller_type);
//                 if (FB_DEBUG_MODE) {
//                     Serial.printf("Task_GetCommands: action=%u → cmd=%d, ctrl_type=%u → ctrl=%d\n",
//                         action, (int)os.fb_last_command, controller_type, (int)os.fb_controller_type);
//                 }
//             }
//         }
//     }
// }

void Task_GetCommands(void *pvParameters) 
{
    // Recuperar variables globales
    GlobalContext* ctx_ptr = static_cast<GlobalContext*>(pvParameters);
    volatile OperationData& os = *(ctx_ptr->os_ptr);

    // Ejecutar tarea periodicamente
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(FB_GET_COMMANDS_PERIOD_MS);
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        uint8_t action = 0;
        uint8_t controller_type = 0;
        if (os.state != OS_State::INIT) 
        {
            if (UpdateCommands(action, controller_type, os.fb_state) == FB_State::OK) 
            {
                switch (action) {
                    case 0:
                        os.fb_last_command = UserCommand::STOP;
                        if (FB_DEBUG_MODE) Serial.printf("[Task_GetCommands] Asignado STOP (0). Valor actual: %d\n", (int)os.fb_last_command);
                        break;
                    case 1:
                        os.fb_last_command = UserCommand::START;
                        if (FB_DEBUG_MODE) Serial.printf("[Task_GetCommands] Asignado START (1). Valor actual: %d\n", (int)os.fb_last_command);
                        break;
                    case 2:
                        os.fb_last_command = UserCommand::IDLE;
                        if (FB_DEBUG_MODE) Serial.printf("[Task_GetCommands] Asignado IDLE (2). Valor actual: %d\n", (int)os.fb_last_command);
                        break;
                    default: 
                        os.fb_last_command = UserCommand::STOP;
                        os.fb_state = FB_State::ERROR;
                        if (FB_DEBUG_MODE) Serial.printf("[Task_GetCommands] Acción inválida: %u. Asignado STOP (0). Valor actual: %d\n", action, (int)os.fb_last_command);
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
}


void Task_Loop(void *pvParameters) {
    // Configuración del periodo de muestreo
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(FB_LOOP_PERIOD_MS);
    // Recuperar variables globales
    GlobalContext* ctx_ptr = static_cast<GlobalContext*>(pvParameters);
    volatile OperationData& os = *(ctx_ptr->os_ptr);
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, period);
        if(!ready()) {
            os.fb_state = FB_State::CONNECTION_ERROR;
            if (FB_DEBUG_MODE) Serial.println("Task_Loop: Firebase no está listo.");
        }
    }
}

} // namespace FirebaseComm
