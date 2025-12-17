#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"

// IBUS is always 14 or 10 channels, but we’ll keep it configurable
#define IBUS_MAX_CHANNELS 14

typedef struct {
    bool link_ok;                  // receiver link valid
    bool frame_ok;
    uint32_t last_rx_ms;           // time since last valid frame
    uint32_t dropped_frames;       // cumulative
} ibus_state_t;

typedef struct {
    uint8_t channel_count;   // e.g. 8
    float channels[IBUS_MAX_CHANNELS];
    bool valid;
} ibus_norm_frame_t;

typedef struct {
    uart_port_t uart;
    gpio_num_t rx_pin;
    gpio_num_t tx_pin;
    uint8_t channel_count;
    int baudrate;
} ibus_config_t;

// bool ibus_read(ibus_frame_t *out);
bool ibus_norm_read(ibus_norm_frame_t *out);
bool ibus_get_state(ibus_state_t *out_state);
void ibus_init(const ibus_config_t *cfg);