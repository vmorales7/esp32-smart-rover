#include "firebase_comm.h"

namespace FirebaseComm {

static MyFirebase fb; // Instancia única global para el wrapper
static bool firebase_ready = false;

// Inicialización
bool init(SSL_CLIENT &client) {
    // Usa las credenciales del archivo secrets.h
    fb.userBegin(client, FB_DATABASE_URL, FB_DATABASE_API_KEY, FB_USER_EMAIL, FB_USER_PASSWORD);
    firebase_ready = fb.ready();
    return firebase_ready;
}

// Loop de mantenimiento
bool ready() {
    return fb.ready();
}

// Set y Get para String
bool setString(const String &path, const String &value) {
    return fb.setString(path, value);
}
bool getString(const String &path, String &value) {
    value = fb.getString(path);
    return !fb.isError();
}

// Set y Get para Int
bool setInt(const String &path, int value) {
    return fb.setInt(path, value);
}
bool getInt(const String &path, int &value) {
    value = fb.getInt(path);
    return !fb.isError();
}

// Set y Get para Float
bool setFloat(const String &path, float value) {
    return fb.setFloat(path, value);
}
bool getFloat(const String &path, float &value) {
    value = fb.getFloat(path);
    return !fb.isError();
}

// Último error
String lastError() {
    return fb.errorString();
}


} // namespace FirebaseComm
