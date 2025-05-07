#include "position_controller.h"

namespace PositionController {

    void init(
        volatile uint8_t* control_mode_ptr,
        volatile float* wL_ref_ptr, volatile float* wR_ref_ptr
    ) {
        *control_mode_ptr = SPEED_REF_INACTIVE;
        *wL_ref_ptr = 0.0f;
        *wR_ref_ptr = 0.0f;
    }

    void set_control_mode(
        uint8_t mode,
        volatile uint8_t* control_mode_ptr
    ) {
        *control_mode_ptr = mode;
    }

    float compute_wheel_speed_ref(float v_ref, float w_ref, uint8_t wheel_id) {
        float rotation_term = w_ref * WHEEL_DISTANCE / 2.0f;
        if (wheel_id == WHEEL_LEFT)
            return (v_ref - rotation_term) / WHEEL_RADIUS;
        else if (wheel_id == WHEEL_RIGHT)
            return (v_ref + rotation_term) / WHEEL_RADIUS;
        else
            return 0.0f; // default / error
    }

    void set_wheel_speed_ref(
        float wL, float wR,
        volatile float* wL_ref_ptr, volatile float* wR_ref_ptr,
        volatile uint8_t* control_mode_ptr
    ) {
        if (*control_mode_ptr == SPEED_REF_MANUAL) {
            *wL_ref_ptr = constrain(wL, -WM_NOM, WM_NOM);
            *wR_ref_ptr = constrain(wR, -WM_NOM, WM_NOM);

        // El controlador internamente debe hacer el ajuste
        } else if (*control_mode_ptr == SPEED_REF_AUTO_BASIC || *control_mode_ptr == SPEED_REF_AUTO_ADVANCED){ 
            *wL_ref_ptr = wL;
            *wR_ref_ptr = wR;
        }
    }

}
