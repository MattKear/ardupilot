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

#include "AP_RPM.h"
#include "RPM_Pin.h"
#include "RPM_SITL.h"
#include "RPM_EFI.h"
#include "RPM_HarmonicNotch.h"
#include "RPM_ESC_Telem.h"

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_RPM::var_info[] = {

    // 0-13 used by old param indexes before being moved to AP_RPM_Params

    // @Group: 1_
    // @Path: AP_RPM_Params.cpp
    AP_SUBGROUPINFO(_params[0], "1_", 14, AP_RPM, AP_RPM_Params),

#if RPM_MAX_INSTANCES > 1
    // @Group: 2_
    // @Path: AP_RPM_Params.cpp
    AP_SUBGROUPINFO(_params[1], "2_", 15, AP_RPM, AP_RPM_Params),
#endif

    AP_GROUPEND
};

AP_RPM::AP_RPM(void)
{
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_RPM must be singleton");
    }
    _singleton = this;
}

/*
  initialise the AP_RPM class.
 */
void AP_RPM::init(void)
{
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }
    for (uint8_t i=0; i<RPM_MAX_INSTANCES; i++) {
        switch (_params[i]._type) {
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
        case RPM_TYPE_PWM:
        case RPM_TYPE_PIN:
            // PWM option same as PIN option, for upgrade
            drivers[i] = new AP_RPM_Pin(*this, i, state[i]);
            break;
#endif
        case RPM_TYPE_ESC_TELEM:
            drivers[i] = new AP_RPM_ESC_Telem(*this, i, state[i]);
            break;
#if HAL_EFI_ENABLED
        case RPM_TYPE_EFI:
            drivers[i] = new AP_RPM_EFI(*this, i, state[i]);
            break;
#endif
        // include harmonic notch last
        // this makes whatever process is driving the dynamic notch appear as an RPM value
        case RPM_TYPE_HNTCH:
            drivers[i] = new AP_RPM_HarmonicNotch(*this, i, state[i]);
            break;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        case RPM_TYPE_SITL:
            drivers[i] = new AP_RPM_SITL(*this, i, state[i]);
            break;
#endif
        }
        if (drivers[i] != nullptr) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
            num_instances = i+1; // num_instances is a high-water-mark
        }
    }
}

/*
  update RPM state for all instances. This should be called by main loop
 */
void AP_RPM::update(void)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != nullptr) {
            if (_params[i]._type == RPM_TYPE_NONE) {
                // allow user to disable an RPM sensor at runtime and force it to re-learn the quality if re-enabled.
                state[i].signal_quality = 0;
                continue;
            }

            drivers[i]->update();
        }
    }
}

/*
  check if an instance is healthy
 */
bool AP_RPM::healthy(uint8_t instance) const
{
    if (instance >= num_instances || _params[instance]._type == RPM_TYPE_NONE) {
        return false;
    }

    // check that data quality is above minimum required
    if (state[instance].signal_quality < _params[instance]._quality_min) {
        return false;
    }

    return true;
}

/*
  check if an instance is activated
 */
bool AP_RPM::enabled(uint8_t instance) const
{
    if (instance >= num_instances) {
        return false;
    }
    // if no sensor type is selected, the sensor is not activated.
    if (_params[instance]._type == RPM_TYPE_NONE) {
        return false;
    }
    return true;
}

/*
  get RPM value, return true on success
 */
bool AP_RPM::get_rpm(uint8_t instance, float &rpm_value) const
{
    if (!healthy(instance)) {
        return false;
    }
    rpm_value = state[instance].rate_rpm;
    return true;
}

// singleton instance
AP_RPM *AP_RPM::_singleton;

namespace AP {

AP_RPM *rpm()
{
    return AP_RPM::get_singleton();
}

}
