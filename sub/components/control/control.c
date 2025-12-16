#include "control.h"
#include "control_internal.h"

#include "arming.h"
#include "motor.h"
#include "ibus.h"
#include <math.h>


#define ARMING_DEADBAND 0.05f
#define FAILSAFE_TIMEOUT_MS 200

static float norm_clamp(float v)
{
    if (v > 1.0f) return 1.0f;
    if (v < -1.0f) return -1.0f;
    return v;
}

static bool inputs_are_neutral(const control_input_t *in)
{
    return( fabsf(in->forward) < ARMING_DEADBAND &&
            fabsf(in->strafe) < ARMING_DEADBAND &&
            fabsf(in->vertical) < ARMING_DEADBAND &&
            fabsf(in->yaw) < ARMING_DEADBAND);

}

static control_input_t ctrl_in;

void control_init(void)
{
    ctrl_in.valid = false;
}

void control_step(void)
{
    ibus_norm_frame_t rc;

    /* ---- Read normalised RC ---- */
    if (!ibus_norm_read(&rc)) {
        ctrl_in.valid = false;
    } else {
        ctrl_in.forward  = rc.channels[CH_FORWARD];
        ctrl_in.strafe   = rc.channels[CH_STRAFE];
        ctrl_in.vertical = rc.channels[CH_VERTICAL];
        ctrl_in.yaw      = rc.channels[CH_YAW];
        ctrl_in.arm      = (rc.channels[CH_ARM] > 0.5f);
        ctrl_in.valid    = true;
    }

    /* ---- Failsafe ---- */
    if (!ctrl_in.valid) {
        motor_set_norm(MOTOR_SERVO,  0.0f);
        motor_set_norm(MOTOR_VERT, 0.0f);
        return;
    }

    /* ---- Arming logic ---- */
    bool armed     = arming_is_armed();
    bool want_arm  = ctrl_in.arm;
    bool safe_arm  = inputs_are_neutral(&ctrl_in);

    if (!armed && want_arm && safe_arm) {
        arming_set(true);
    } else if (armed && !want_arm) {
        arming_set(false);
    }

    if (!arming_is_armed()) {
        motor_set_norm(MOTOR_SERVO,  0.0f);
        motor_set_norm(MOTOR_VERT, 0.0f);
        return;
    }

    /* ---- Mixing ---- */
    float vert  = ctrl_in.vertical;
    float servo = ctrl_in.tilt;
    float led = ctrl_in.led_duty;

    vert  = norm_clamp(vert);
    servo = norm_clamp(servo);
    led = norm_clamp(led);

    motor_set_norm(MOTOR_SERVO,  servo);
    motor_set_norm(MOTOR_VERT, vert);
}
