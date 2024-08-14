#pragma once

#include <stdint.h>

enum class ubx_ant_status : uint8_t {
    ASTATUS_INIT        = 0x00,
    ASTATUS_DONTKNOW    = 0x01,
    ASTATUS_OK          = 0x02,
    ASTATUS_SHORT       = 0x03,
    ASTATUS_OPEN        = 0x04
};

enum class ubx_jam_state {
    JAM_STATE_UNKNOWN = 0,
    JAM_STATE_NONE,
    JAM_STATE_WARNING,
};

enum class ubx_spoof_state {
    SPOOF_STATE_UNKNOWN = 0,
    SPOOF_STATE_NONE,
    SPOOF_STATE_INDICATED,
    SPOOF_STATE_AFFIRMED,
};

enum class ubx_nav_fix_type {
    FIX_NONE = 0,
    FIX_DEAD_RECKONING = 1,
    FIX_2D = 2,
    FIX_3D = 3,
    FIX_GPS_DEAD_RECKONING = 4,
    FIX_TIME = 5
};

enum class ubx_nav_fix_flags {
    FIX_FLAGS_DGPS = 0b00000010,
    FIX_FLAGS_RTK_FLOAT = 0b01000000,
    FIX_FLAGS_RTK_FIXED = 0b10000000
};