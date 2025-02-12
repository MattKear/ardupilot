/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "AP_Mission.h"
#include <AC_WPNav/AC_WPNav.h>

extern const AP_HAL::HAL& hal;

class AP_Containment_Manager
{
public:
    AP_Containment_Manager(AC_WPNav *&wpnav);

    // Function for running time scaler calculation and applying to mission time
    void update(const Location current_loc);

    void update_logging(void);

    // Function for resetting the containment manager and the mission time scaler
    void reset(void);

    // Function to update state from mission command
    void update_state(const AP_Mission::Mission_Command& cmd);

private:

    AC_WPNav*& _wpnav;

    const char* get_state_string(void) const;

    enum class State {
        DISABLE = 0,
        MONITORING_ONLY = 1,
        ACTIVE = 2,
        ERROR = 10,
        TIMEOUT = 11
    } _state;

    float _radius;                    // Measured planer radius (m)
    float _max_radius;                // Max radius set by mission do command (m)
    float  _time_scaler;              // Calculated time scale value that is applied to the target kinematics in the mission
    State _desired_sate;              // The state set by the last mission do command
    State _last_state;                // keep track of changing state
    uint32_t _last_logging_update_ms;         // Last time update function was successfully run (ms)
    uint32_t _last_scaler_update_ms;  // Last time update time scaler function was successfully run (ms)
};
