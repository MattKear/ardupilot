#pragma once

#include <AP_Param/AP_Param.h>
#include "AP_Relay_config.h"

class AP_Relay_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_Relay_Params(void);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Relay_Params);

    enum class Default_State : uint8_t {
        Off = 0,
        On = 1,
        NoChange = 2,
    };

    enum class Function : uint8_t {
        none     = 0,
        relay    = 1,
        ignition = 2, // high for on
        starter  = 3, // high for on
        num_functions // must be the last entry
    };

    AP_Enum<Function> function;         // relay function
    AP_Int8 pin;                          // gpio pin number
    AP_Enum<Default_State> default_state; // default state
};
