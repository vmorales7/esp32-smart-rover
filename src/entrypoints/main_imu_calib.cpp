#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#warning "Compilando main_imu_calib.cpp"

#define BNO_SDA 21      // Cambia estos pines según tu hardware
#define BNO_SCL 13

extern Adafruit_BNO055 bno;

// Imprime el estado de calibración
void printCalibrationStatus() {
    uint8_t sys, gyro, accel, mag;
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    Serial.print("Calibración -> SYS:");
    Serial.print(sys);
    Serial.print(" | GYR:");
    Serial.print(gyro);
    Serial.print(" | ACC:");
    Serial.print(accel);
    Serial.print(" | MAG:");
    Serial.print(mag);
    Serial.print("  [3 = OK]");
}

// Imprime la orientación
void printEuler() {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print("  Yaw (Z): ");
    Serial.print(euler.x(), 2);
    Serial.print("°  Pitch (Y): ");
    Serial.print(euler.y(), 2);
    Serial.print("°  Roll (X): ");
    Serial.print(euler.z(), 2);
    Serial.print("°");
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("==== Calibración BNO055 ====");
    Serial.println("Instrucciones:");
    Serial.println("- GYR: manten quieto el sensor unos segundos.");
    Serial.println("- ACC: gíralo en todas las posiciones (6 caras de un cubo).");
    Serial.println("- MAG: haz el clásico '8' en el aire, lento y amplio.");
    Serial.println("Cuando todos lleguen a 3, la calibración está completa.");

    Wire.begin(BNO_SDA, BNO_SCL);

    if (!bno.begin()) {
        Serial.println("No se detectó el BNO055. Revisa las conexiones.");
        while (1) delay(10);
    }
    bno.setExtCrystalUse(true);
}

void loop() {
    printCalibrationStatus();
    printEuler();
    Serial.println();
    delay(500);
}
