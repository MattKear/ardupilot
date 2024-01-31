#include "Copter.h"

// Run standby functions at approximately 100 Hz to limit maximum variable build up
//
// When standby is active:
//      all I terms are continually reset
//      heading error is reset to zero
//      position errors are reset to zero
//      crash_check is disabled
//      thrust_loss_check is disabled
//      parachute_check is disabled
//      hover throttle learn is disabled
//      and landing detection is disabled.
void Copter::standby_update()
{
    if (!standby_active) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        if (copter.g.sysid_this_mav == 1 && AP::sitl()->ride_along_master.get() != 0) {
            AP::sitl()->ride_along_master.set_and_notify(0);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "MAV %d: Changing ride_along_master", g.sysid_this_mav.get());
        }
#endif
        return;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (copter.g.sysid_this_mav == 1 && AP::sitl()->ride_along_master.get() != 1) {
        AP::sitl()->ride_along_master.set_and_notify(1);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "MAV %d: Changing ride_along_master", g.sysid_this_mav.get());
    }
#endif


    attitude_control->reset_rate_controller_I_terms();
    attitude_control->reset_yaw_target_and_rate();
    pos_control->standby_xyz_reset();
}
