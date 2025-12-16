#pragma once
#include <stdbool.h>

typedef struct {
    float forward;
    float strafe;
    float yaw;
    float vertical;
    float 
} control_input_t;

void control_init(void);
void control_update(const control_input_t *in);
static float norm_ibus(uint16_t v);