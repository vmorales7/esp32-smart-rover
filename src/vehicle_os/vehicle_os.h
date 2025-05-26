#ifndef VEHICLE_OS_H
#define VEHICLE_OS_H

/* ------------------------ Mis librerías ------------------------*/

#include "project_config.h"

// Sensores
#include "sensors_firmware/encoder_reader.h"
#include "sensors_firmware/distance_sensors.h"
// #include "sensors_firmware/imu_reader.h"

// Estimación y control
#include "position_system/pose_estimator.h"
#include "position_system/position_controller.h"
// #include "position_system/evade_controller.h"

// Control de motores
#include "motor_drive/motor_controller.h"

// Comunicación (a futuro)
// #include "communication/firebase_comm.h" // Solo si empiezas a usar Firebase

/* ------------------------ Constantes ------------------------*/

constexpr float NULL_WAYPOINT_XY = 99.9f;


/* ------------------------ Funciones ------------------------*/




#endif // VEHICLE_OS_H