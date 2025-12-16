#pragma once 



typedef struct {
    float forward;
    float strafe;
    float yaw;
    float vertical;
    float led_duty;
    float tilt;
    float arm;
    bool valid;
} control_input_t;

enum {
    CH_STRAFE = 0,
    CH_FORWARD,
    CH_VERTICAL,
    CH_YAW,
    CH_ARM,
    CH_LED,
};