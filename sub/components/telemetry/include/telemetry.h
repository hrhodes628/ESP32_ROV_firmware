#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"
#include "driver/gpio.h"

typedef struct {
    uart_port_t uart;
    gpio_num_t  tx_pin;
    int         baudrate;

    uint32_t heartbeat_period_ms;  // defaulted if zero
    uint32_t telemetry_rate_ms;
} telemetry_config_t;

/* ---------- Init ---------- */
void telemetry_init(const telemetry_config_t *cfg);

/* ---------- Sensor updates ---------- */
void telemetry_set_voltage(float v);
void telemetry_set_depth(float m);
void telemetry_set_heading(float deg);
void telemetry_set_camera_pitch(float rad);

/* ---------- Status / errors ---------- */
void telemetry_report_ibus(bool ok, uint32_t dropped);
void telemetry_report_error(uint32_t error_mask);

/* ---------- Optional text ---------- */
void telemetry_send_text(const char *text, uint8_t severity);