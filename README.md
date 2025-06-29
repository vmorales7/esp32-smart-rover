# Autonomous Vehicle Project - ESP32

This repository contains the source code for an autonomous vehicle controlled by an ESP32, developed as part of the **IEE2913 - Electrical Design** course. The system implements autonomous navigation with position control (X, Y), pose estimation using encoders and an IMU, obstacle avoidance, and a multitasking structure using RTOS (FreeRTOS on ESP32).

---

## 🌐 Project Structure

The code follows a modular architecture that clearly separates functionalities by layer.

```text
📁 src/
├── motor_drive/         # PWM-based driver control (L298N) and wheel speed PI controller
├── sensors_firmware/    # Modules for reading encoders, ultrasonic sensors, and IMU
├── position_system/     # Pose estimators and position controllers
├── communication/       # WiFi, Firebase connection and messaging
├── vehicle_os/          # Vehicle operation logic and obstacle avoidance state machine
├── entrypoints/         # Individual main.cpp files for specific test scenarios
├── main.cpp             # Default main (not used directly)
├── main_selector.h      # Selector to choose the active main.cpp file for compilation
└── secrets.h            # Stores WiFi and API credentials (not uploaded to GitHub)

📁 web_dashboard/
├── index.html           # Web app (plain HTML) for control and data display
└── config.js            # Stores API credentials (not uploaded to GitHub)
```

---

## 📁 Folder Details

### `/motor_drive/`
Contains the `MotorController`, responsible for:
- Executing wheel speed control using a PI algorithm
- Translating angular reference (`w_ref`) to motor `duty cycle`
- Applying active braking or controlled inversion

### `/sensors_firmware/`
Includes modules for:
- **Incremental encoder** reading
- **Ultrasonic sensors (HC-SR04)** for obstacle detection
- **IMU (BNO)** readings

### `/position_system/`
Implements:
- Position and velocity (pose) estimator
- Position controller with two modes:
  - `PID`: Classic PID angle control
  - `BACKS`: Backstepping-type control law

### `/communication/`
Includes WiFi connection setup and Firebase asynchronous communication.

### `/vehicle_os/`
Implements the main system state machine. Includes:
- Operating logic for movement
- Obstacle avoidance logic
- Global configuration and constants for system setup

### `/entrypoints/`
Each `main_*.cpp` file corresponds to a **specific test**, for example:
- `main_pose.cpp`: pose estimation
- `main_wheel_speed.cpp`: wheel speed control test
- `main_distance_sensors.cpp`: obstacle detection logic test

---

## 🔀 Selecting the active main file (via `main_selector.h`)

The `main_selector.h` file allows you to dynamically choose which `main_*.cpp` file in `/entrypoints/` to compile, without modifying `platformio.ini`. Just uncomment the corresponding `#include` line in `main_selector.h` and comment out the rest.
