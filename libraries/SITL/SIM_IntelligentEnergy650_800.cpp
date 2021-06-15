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
/*
  Simulator for the IntelligentEnergy 650 W and 800 W FuelCell generator
*/

#include <AP_Math/AP_Math.h>

#include "SIM_IntelligentEnergy650_800.h"
#include "SITL.h"

#include <errno.h>

#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

using namespace SITL;

// table of user settable parameters
const AP_Param::GroupInfo IntelligentEnergy650_800::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: IntelligentEnergy 650 W and 800 W fuel cell sim enable/disable
    // @Description: Allows you to enable (1) or disable (0) the FuelCell simulator
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENABLE", 1, IntelligentEnergy650_800, enabled, 0),

    // @Param: STATE
    // @DisplayName: Explicitly set state
    // @Description: Explicity specify a state for the generator to be in
    // @User: Advanced
    AP_GROUPINFO("STATE", 2, IntelligentEnergy650_800, set_state, 2),

    // @Param: ERROR
    // @DisplayName: Explicitly set error code
    // @Description: Explicity specify an error code to send to the generator
    // @User: Advanced
    AP_GROUPINFO("ERROR", 3, IntelligentEnergy650_800, err_code, 0),

    AP_GROUPEND
};

IntelligentEnergy650_800::IntelligentEnergy650_800() : IntelligentEnergy::IntelligentEnergy()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void IntelligentEnergy650_800::update(const struct sitl_input &input)
{
    if (!enabled.get()) {
        return;
    }
    update_send();
}

void IntelligentEnergy650_800::update_send()
{
    // just send a chunk of data at 2Hz:
    const uint32_t now = AP_HAL::millis();
    if (now - last_sent_ms < 500) {
        return;
    }

    // Simulate constant tank pressure. This isn't true in reality, but is good enough
    const int8_t tank_pct = 80;

    // Update battery
    const float delta = discharge ? -0.2 : 0.2;
    battery_pct += delta;

    // Decide if we need to charge or discharge the battery
    if (battery_pct <= 20.0f) {
        discharge = false;
    } else if (battery_pct >= 100.0f) {
        discharge = true;
    }

    last_sent_ms = now;

    char message[128];
    hal.util->snprintf(message, ARRAY_SIZE(message), "<%i,%i,%u,%u>\n",
             tank_pct,
             battery_pct,
             set_state.get(),
             (uint32_t)err_code);

    if ((unsigned)write_to_autopilot(message, strlen(message)) != strlen(message)) {
        AP_HAL::panic("Failed to write to autopilot: %s", strerror(errno));
    }
}
