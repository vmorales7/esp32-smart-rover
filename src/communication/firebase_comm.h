#ifndef FIREBASE_COMM_H
#define FIREBASE_COMM_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "secrets.h" // Credenciales de Firebase y WiFi

// Siempre comenzar con las definiciones de preprocesador para FirebaseClient
#define ENABLE_USER_AUTH
#define ENABLE_DATABASE
#include <FirebaseClient.h>
#include "MyFirebase.h"

#define SSL_CLIENT WiFiClientSecure

namespace FirebaseComm {

// Inicialización de Firebase con autenticación por email/pass
bool init(SSL_CLIENT &client);

// Estado de autenticación listo y mantenimiento en loop
bool ready();

// Escritura y lectura de string
bool setString(const String &path, const String &value);
bool getString(const String &path, String &value);

// Escritura y lectura de int
bool setInt(const String &path, int value);
bool getInt(const String &path, int &value);

// Escritura y lectura de float
bool setFloat(const String &path, float value);
bool getFloat(const String &path, float &value);

// Último error
String lastError();

} // namespace FirebaseComm

#endif

