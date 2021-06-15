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
  Simulator for the IntelligentEnergy 650W and 800W FuelCell

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --uartF=sim:ie650_800 --speedup=1 --console

param set SERIAL5_PROTOCOL 30  # Generator
param set SERIAL5_BAUD 115200
param set GEN_TYPE 1  # IE650 IE800
param set BATT2_MONITOR 17  # electrical
param set SIM_IE650_ENABLE 1
param fetch

graph BATTERY_STATUS.voltages[0]

reboot

./Tools/autotest/autotest.py --gdb --debug build.ArduCopter fly.ArduCopter.IntelligentEnergy650_800

*/

#pragma once

#include <AP_Param/AP_Param.h>

#include "SITL_Input.h"

#include "SIM_IntelligentEnergy.h"

#include <stdio.h>

namespace SITL {

class IntelligentEnergy650_800 : public IntelligentEnergy {
public:

    IntelligentEnergy650_800();

    // update state
    void update(const struct sitl_input &input) override;

    static const AP_Param::GroupInfo var_info[];

private:

    void update_send();

    AP_Int8 enabled;  // enable sim
    AP_Int8 set_state;
    AP_Int32 err_code;

    float battery_pct = 100.0f;
    bool discharge = true; // used to switch between battery charging and discharging
    uint32_t last_sent_ms;

};

}
