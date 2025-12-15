#pragma once

#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/uart.h"

// IBUS is always 14 or 10 channels, but we’ll keep it configurable
#define IBUS_MAX_CHANNELS 14

typedef struct {
    uint8_t channel_count;   // e.g. 8
    uint16_t channels[IBUS_MAX_CHANNELS];
} ibus_frame_t;

typedef struct {
    uart_port_t uart;
    gpio_num_t rx_pin;
    gpio_num_t tx_pin;
    uint8_t channel_count;
} ibus_config_t;