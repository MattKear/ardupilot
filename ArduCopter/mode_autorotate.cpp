#include "Copter.h"

#include "mode.h"

#if MODE_AUTOROTATE_ENABLED == ENABLED

/*
 * Init and run calls for autorotate flight mode
 */

//bool Copter::ModeAutorotate::init(bool ignore_checks)
//{

// Calculate recovery scaling values

//return 1

//}

void Copter::ModeAutorotate::run()
{
    // apply simple mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
    } else if (ap.throttle_zero) {
        // Attempting to Land
        motors->set_desired_spool_state(AP_Motors::DESIRED_GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
    }

    if (motors->get_spool_mode() == AP_Motors::SHUT_DOWN) {
        // Motors Stopped
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->reset_rate_controller_I_terms();
    } else if (motors->get_spool_mode() == AP_Motors::GROUND_IDLE) {
        // Landed
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->reset_rate_controller_I_terms();
    } else if (motors->get_spool_mode() == AP_Motors::THROTTLE_UNLIMITED) {
        // clear landing flag above zero throttle
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // output pilot's throttle
    attitude_control->set_throttle_out(get_pilot_desired_throttle(),
                                       true,
                                       g.throttle_filt);

}


// example extra functiony thingy
//void Copter::ModeAutorotate::get_pilot_desired_angle_rates(int16_t roll_in, int16_t pitch_in, int16_t yaw_in, float &roll_out, float &pitch_out, float &yaw_out)
//{
//    float rate_limit;
//    Vector3f rate_ef_level, rate_bf_level, rate_bf_request;

    // apply circular limit to pitch and roll inputs
//    float total_in = norm(pitch_in, roll_in);

//    if (total_in > ROLL_PITCH_YAW_INPUT_MAX) {
//        float ratio = (float)ROLL_PITCH_YAW_INPUT_MAX / total_in;
//        roll_in *= ratio;
//        pitch_in *= ratio;
//       }
//}

#endif
