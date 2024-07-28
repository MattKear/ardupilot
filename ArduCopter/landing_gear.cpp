#include "Copter.h"

#if AP_LANDINGGEAR_ENABLED

// Run landing gear controller at 10Hz
void Copter::landinggear_update()
{
    // exit immediately if no landing gear output has been enabled
    if (!SRV_Channels::function_assigned(SRV_Channel::k_landing_gear_control)) {
        return;
    }

    // support height based triggering using rangefinder or altitude above ground
    float height;
    if (!rangefinder_state.get_height_above_ground(height)){
        // if something has gone wrong with hagl measurement, it is
        // safer to assume zero height and deploy the landing gear
        height = 0.0;
    }

#if AP_RANGEFINDER_ENABLED
    switch (rangefinder.status_orient(ROTATION_PITCH_270)) {
    case RangeFinder::Status::NotConnected:
    case RangeFinder::Status::NoData:
        // use altitude above home or terrain for non-functioning rangefinder
        break;

    case RangeFinder::Status::OutOfRangeLow:
        // altitude is close to zero (gear should deploy)
        height = 0.0;
        break;

    case RangeFinder::Status::OutOfRangeHigh:
    case RangeFinder::Status::Good:
        // use last good reading
        height = rangefinder_state.alt_cm_filt.get()*0.01;
        break;
    }
#endif  // AP_RANGEFINDER_ENABLED

    landinggear.update(height);
}

#endif // AP_LANDINGGEAR_ENABLED
