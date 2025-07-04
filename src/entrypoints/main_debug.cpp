#include <Arduino.h>
#include "WiFi.h"

#define WIFI_SSID  "VMA"
#define WIFI_PASS  "Tangananica"

enum class MotorMode : uint8_t {
    IDLE = 0U,    
    MANUAL = 1U,  
    AUTO = 2U,   
    BREAK = 3U   
};
enum class PositionControlMode : uint8_t {
    INACTIVE = 0U,
    MANUAL,
    ALIGN,
    MOVE,
    ROTATE
};
struct SystemStates {
    MotorMode motors;
    uint8_t encoders;
    uint8_t imu;
    uint8_t distance;
    uint8_t pose;
    PositionControlMode position;
};
struct SensorsData {
    float enc_phiL;
    float enc_phiR;
    float enc_wL;
    float enc_wR;
    uint8_t us_left_dist;      
    uint8_t us_mid_dist;   
    uint8_t us_right_dist;   
    bool us_left_obst;     
    bool us_mid_obst;    
    bool us_right_obst;      
    bool us_obstacle;    
    float imu_acc; 
    float imu_w; 
    float imu_theta;
};
struct GlobalContext {
    SystemStates* systems_ptr;
    SensorsData* sensors_ptr;
};

// ====================== VARIABLES GLOBALES ======================
GlobalContext* ctx = nullptr;

// =================== Operación del sistema ====================

void setup() {
    Serial.begin(115200);
    delay(5000);
    Serial.println();

    ctx = new GlobalContext;
    ctx->systems_ptr = new SystemStates;
    ctx->sensors_ptr = new SensorsData;

    Serial.println("Iniciando WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\n¡WiFi conectado!");
}

void loop() {
    // Nada
}
