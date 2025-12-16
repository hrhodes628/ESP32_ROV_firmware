#pragma once
#include <stdint.h>
typedef enum {
    MOTOR_SERVO,
    MOTOR_VERT,
    LED_PANEL,
} motor_id_t;

void motor_init(void);
void motor_set_us(motor_id_t motor, uint16_t pulse_us);
void motor_set_norm_duty(motor_id_t motor, uint8_t percent);
void motor_set_norm(motor_id_t motor, float value);