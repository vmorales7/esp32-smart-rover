#include "position_controller.h"

namespace PositionController {

    void init_position_controller(
        volatile float* v_ref_ptr, volatile float* w_ref_ptr,
        volatile float* wL_ref_ptr, volatile float* wR_ref_ptr,
        volatile uint8_t* control_mode_ptr
    ) {
        *v_ref_ptr = 0.0f;
        *w_ref_ptr = 0.0f;
        *wL_ref_ptr = 0.0f;
        *wR_ref_ptr = 0.0f;
        *control_mode_ptr = SPEED_REF_INACTIVE;
    }

    void set_position_control_mode(
        uint8_t mode,
        volatile uint8_t* control_mode_ptr
    ) {
        *control_mode_ptr = mode;
    }

    float compute_wheel_speed_ref(float v_ref, float w_ref, uint8_t wheel_id) {
        float rotation_term = w_ref * WHEEL_DISTANCE / 2.0f;
        if (wheel_id == WHEEL_LEFT)
            return (v_ref - rotation_term) / WHEEL_RADIUS;
        else
            return (v_ref + rotation_term) / WHEEL_RADIUS;
    }

    void set_wheel_speed_ref(
        float value_left, float value_right,
        volatile float* wL_ref_ptr, volatile float* wR_ref_ptr,
        volatile uint8_t* control_mode_ptr
    ) {
        if (*control_mode_ptr != SPEED_REF_INACTIVE) {
            *wL_ref_ptr = value_left;
            *wR_ref_ptr = value_right;
        }
    }

    void set_velocity_ref(
        float v_ref, float w_ref,
        volatile float* v_ref_ptr, volatile float* w_ref_ptr,
        volatile float* wL_ref_ptr, volatile float* wR_ref_ptr,        
        volatile uint8_t* control_mode_ptr
    ) {
        if (*control_mode_ptr != SPEED_REF_INACTIVE) {
            *v_ref_ptr = v_ref;
            *w_ref_ptr = w_ref;
            float wL = compute_wheel_speed_ref(v_ref, w_ref, WHEEL_LEFT);
            float wR = compute_wheel_speed_ref(v_ref, w_ref, WHEEL_RIGHT);
            *wL_ref_ptr = wL;
            *wR_ref_ptr = wR;
        }
    }

}
