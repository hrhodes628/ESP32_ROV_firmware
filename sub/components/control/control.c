#include "control.h"
#include "motor.h"
#include "ibus.h"


#define ARMING_DEADBAND 0.05f
#define FAILSAFE_TIMEOUT_MS 200

static float norm_clamp(float v)
{
    if (v > 1.0f) return 1.0f;
    if (v < -1.0f) return -1.0f;
    return v;
}

static float norm_ibus(uint16_t v)
{
    /* IBUS nominal range: 1000–2000, center at 1500 */
    float x = ((float)v - 1500.0f) / 500.0f;

    x= norm_clamp(x);

    return x;
}


void control_update(const control_input_t *in)
{   

    bool armed = arming_is_armed();
    bool want_arm = (arm_switch_high);

    ibus_state_t link_state = ibus_get_state();

    bool failsafe_active = link_state.link_ok || link_state.last_rx_ms > FAILSAFE_TIMEOUT_MS ;

    bool safe_to_arm = inputs_are_neutral(in) && !failsafe_active;


    float left  = in->forward + in->yaw;
    float right = in->forward - in->yaw;

    left  = norm_clamp(left);
    right = norm_clamp(right);

    motor_set_norm(MOTOR_LEFT,  left);
    motor_set_norm(MOTOR_RIGHT, right);
}

static bool inputs_are_neutral(const control_input_t *in){

    return( fabsf(in->forward) < ARMING_DEADBAND &&
            fabsf(in->strafe) < ARMING_DEADBAND &&
            fabsf(in->vertical) < ARMING_DEADBAND &&
            fabsf(in->yaw) < ARMING_DEADBAND);

}
