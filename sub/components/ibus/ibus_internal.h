#pragma once

#include "ibus.h"

typedef struct {
    uint8_t channel_count;   // e.g. 8
    uint16_t channels[IBUS_MAX_CHANNELS];
} ibus_frame_t;