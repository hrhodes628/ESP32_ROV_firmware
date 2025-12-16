#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ---------- Init ---------- */
void telemetry_init(void);

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