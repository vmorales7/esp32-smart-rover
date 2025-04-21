#include "position_controller.h"

namespace PositionController {

    void update_wheel_speed_reference(
        volatile float* v_ref_ptr, volatile float* w_ref_ptr,
        volatile float* wL_ref_ptr, volatile float* wR_ref_ptr
    ) {
        *wL_ref_ptr  = (*v_ref_ptr - *w_ref_ptr * WHEEL_DISTANCE / 2.0f) / WHEEL_RADIUS;
        *wR_ref_ptr = (*v_ref_ptr + *w_ref_ptr * WHEEL_DISTANCE / 2.0f) / WHEEL_RADIUS;
    }
    
}

