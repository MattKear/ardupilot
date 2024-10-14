/* -----------------------------------------------------------------------------------------
    This is currently a SITL only function until the project is complete.
    To trial this in SITL you will need to use Real Flight 8.
    Instructions for how to set this up in SITL can be found here:
    https://discuss.ardupilot.org/t/autonomous-autorotation-gsoc-project-blog/42139
 -----------------------------------------------------------------------------------------*/

#include "Copter.h"
#include <AC_Autorotation/AC_Autorotation.h>
#include "mode.h"

#if MODE_AUTOROTATE_ENABLED

bool ModeAutorotate::init(bool ignore_checks)
{
#if FRAME_CONFIG != HELI_FRAME
    // Only allow trad heli to use autorotation mode
    return false;
#endif

    // Check that mode is enabled, make sure this is the first check as this is the most
    // important thing for users to fix if they are planning to use autorotation mode
    if (!g2.arot.enabled()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "AROT: Mode not enabled");
        return false;
    }

    // Must be armed to use mode, prevent triggering state machine on the ground
    if (!motors->armed() || copter.ap.land_complete || copter.ap.land_complete_maybe) {
        gcs().send_text(MAV_SEVERITY_WARNING, "AROT: Must be armed and flying");
        return false;
    }

    // Initialise controller
    g2.arot.init();

    // Display message 
    gcs().send_text(MAV_SEVERITY_INFO, "Autorotation initiated");

     // Set all initial flags to on
    _flags.entry_init = false;
    _flags.glide_init = false;
    _flags.flare_init = false;
    _flags.touch_down_init = false;
    _flags.landed_init = false;

    // Ensure the surface distance object is enabled.  We tidy this up on the mode exit function
    copter.rangefinder_state.set_enabled_by_ap(true);

    // Setting default starting switches
    phase_switch = Autorotation_Phase::ENTRY;

    // Set entry timer
    _entry_time_start_ms = millis();

    // reset logging timer
    _last_logged = 0;

    return true;
}

void ModeAutorotate::run()
{
    // Current time
    uint32_t now = millis(); //milliseconds

    // Set dt in library
    float const last_loop_time_s = AP::scheduler().get_last_loop_time_s();
    g2.arot.set_dt(last_loop_time_s);

    g2.arot.update_hagl();

    //----------------------------------------------------------------
    //                  State machine logic
    //----------------------------------------------------------------
    // State machine progresses through the autorotation phases as you read down through the if statements.
    // More urgent phases (the ones closer to the ground) take precedence later in the if statements. Init
    // flags are used to prevent flight phase regression

    if (!_flags.glide_init && !g2.arot.below_flare_height() && ((now - _entry_time_start_ms) > g2.arot.entry_time_ms)) {
        // Flight phase can be progressed to steady state glide
        phase_switch = Autorotation_Phase::SS_GLIDE;
    }

    // Check if we are between the flare start height and the touchdown height
    if (!_flags.flare_init && g2.arot.below_flare_height()) {// && !g2.arot.should_begin_touchdown()) {
        phase_switch = Autorotation_Phase::FLARE;
    }

    // TODO: hover entry init

    // Begin touch down if within touch down time
    // if (!_flags.touch_down_init && g2.arot.should_begin_touchdown()) {
    //     phase_switch = Autorotation_Phase::TOUCH_DOWN;
    // }

    // Check if we believe we have landed. We need the landed state to zero all
    // controls and make sure that the copter landing detector will trip
    if (g2.arot.check_landed()) {
        phase_switch = Autorotation_Phase::LANDED;
    }

    // Check if we are bailing out and need to re-set the spool state
    if (motors->autorotation_bailout()) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    // Get norm input from yaw channel
    const float pilot_norm_input = channel_yaw->norm_input_dz();

    //----------------------------------------------------------------
    //                  State machine actions
    //----------------------------------------------------------------
    switch (phase_switch) {

        case Autorotation_Phase::ENTRY:
        {
            // Smoothly transition the collective to enter autorotation regime and begin forward speed control
            if (!_flags.entry_init) {
                g2.arot.init_entry();
                _flags.entry_init = true;
            }
            g2.arot.run_entry(pilot_norm_input);
            break;
        }

        case Autorotation_Phase::SS_GLIDE:
        {
            // Maintain head speed and forward speed as we glide to the ground
            if (!_flags.glide_init) {
                g2.arot.init_glide();
                _flags.glide_init = true;
            }
            g2.arot.run_glide(pilot_norm_input);
            break;
        }

        case Autorotation_Phase::FLARE:
        {
            // Smoothly slow the aircraft to a stop by pitching up, maintaining set point head speed throughout.
            if (!_flags.flare_init) {
                g2.arot.init_flare();
                _flags.flare_init = true;
            }
            g2.arot.run_flare();
            break;
        }

        case Autorotation_Phase::TOUCH_DOWN:
        {
            // Ensure vehicle is level and use energy stored in head to gently touch down on the ground
            if (!_flags.touch_down_init) {
                g2.arot.init_touchdown();
                _flags.touch_down_init = true;
            }
            g2.arot.run_touchdown();
            break;
        }
        case Autorotation_Phase::LANDED:
        {
            // Landed phase functions to be run only once
            if (!_flags.landed_init) {
                gcs().send_text(MAV_SEVERITY_INFO, "AROT: Landed");
                _flags.landed_init = true;
            }
            // don't allow controller to continually ask for more pitch to build speed when we are on the ground, decay to zero smoothly
            g2.arot.run_landed();
            break;
        }
    }

    // Slow rate (25 Hz) logging for the mode
    if (now - _last_logged > 40U) {
        g2.arot.log_write_autorotation();
        _last_logged = now;
    }

} // End function run()

void ModeAutorotate::exit()
{
    // Ensure we return the rangefinder state back to how we found it
    copter.rangefinder_state.set_enabled_by_ap(false);
}

#endif
