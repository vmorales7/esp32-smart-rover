#include "position_controller.h"

namespace PositionController {

    void init(
        volatile uint8_t& control_mode,
        volatile float& wL_ref, volatile float& wR_ref
    ) {
        control_mode = SPEED_REF_INACTIVE;
        wL_ref = 0.0f;
        wR_ref = 0.0f;
    }

    void set_control_mode(
        const uint8_t new_mode,
        volatile uint8_t& control_mode
    ) {
        control_mode = new_mode;
    }

    float compute_wheel_speed_ref(const float v_ref, const float w_ref, const uint8_t wheel_id) {
        float rotation_term = w_ref * WHEEL_DISTANCE / 2.0f;
        if (wheel_id == WHEEL_LEFT)
            return (v_ref - rotation_term) / WHEEL_RADIUS;
        else if (wheel_id == WHEEL_RIGHT)
            return (v_ref + rotation_term) / WHEEL_RADIUS;
        else
            return 0.0f; // default / error
    }

    void set_wheel_speed_ref(
        const float wL, const float wR,
        volatile float& wL_ref_global, volatile float& wR_ref_global,
        volatile uint8_t& control_mode
    ) {
        if (control_mode == SPEED_REF_MANUAL) {
            wL_ref_global = constrain(wL, -WM_NOM, WM_NOM);
            wR_ref_global = constrain(wR, -WM_NOM, WM_NOM);

        // El controlador internamente debe hacer el ajuste
        } else if (control_mode == SPEED_REF_AUTO_BASIC || control_mode == SPEED_REF_AUTO_ADVANCED){ 
            wL_ref_global = wL;
            wR_ref_global = wR;
        }
    }

}
