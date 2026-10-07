#include "pid.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
    PID_ResetAll();
    for (int i = 0; i < 10000; i++) {
        float value = velocity_PID_value_l_no_ff(1000.0f, 0.0f);
        assert(isfinite(value) && value >= 0.0f && value <= 1000.0f);
    }
    PID_ResetAll();
    assert(velocity_PID_value_l_no_ff(0.0f, 100.0f) == 0.0f);
    assert(velocity_PID_value_l_no_ff(NAN, 1.0f) == 0.0f);
    assert(velocity_PID_value_l_no_ff(20.0f, 0.0f) > 0.0f);
    assert(velocity_PID_value_r_no_ff(INFINITY, 1.0f) == 0.0f);
    assert(velocity_PID_value_r_no_ff(20.0f, 0.0f) > 0.0f);
    PID_ResetAll();
    assert(place_PID_value_limited(450.0f, 450.0f, 5.0f) == 0.0f);
    assert(place_PID_value_limited(NAN, 450.0f, 5.0f) == 0.0f);
    for (int i = 0; i < 100; i++) {
        float value = place_PID_value_limited(800.0f, 450.0f, 5.0f);
        assert(isfinite(value) && fabsf(value) <= 5.0f);
    }
    assert(PID_Mode1PlaceLimited(INFINITY, 450.0f, 5.0f) == 0.0f);
    assert(isfinite(PID_Mode1PlaceLimited(800.0f, 450.0f, 5.0f)));
    puts("PASS: PID saturation, no reverse braking, center deadband and invalid-input recovery");
    return 0;
}
