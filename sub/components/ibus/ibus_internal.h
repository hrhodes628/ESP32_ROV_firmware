#pragma once
#include "ibus.h"
#define IBUS_OVERHEAD 4
#define IBUS_HEADER_0 0x20
#define IBUS_HEADER_1 0x44   // some receivers use 0x40 or 0x44

#define IBUS_MAX_US 2000
#define IBUS_MIN_US 1000
#define IBUS_MID_US 1500

typedef struct {
    uint8_t channel_count;   // e.g. 8
    uint16_t channels[IBUS_MAX_CHANNELS];
} ibus_frame_t;