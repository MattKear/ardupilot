#include "Copter.h"

#include "mode.h"

#if MODE_AUTOROTATE_ENABLED == ENABLED

/*
 * Init and run calls for autorotate flight mode
 */

// initialise zigzag controller
//bool Copter::ModeZigZag::init(bool ignore_checks)
//{
//    // initialize's loiter position and velocity on xy-axes from current pos and velocity
//    loiter_nav->clear_pilot_desired_acceleration();
//    loiter_nav->init_target();

//    // initialise position_z and desired velocity_z
//    if (!pos_control->is_active_z()) {
//        pos_control->set_alt_target_to_current_alt();
//        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
//    }

//    // initialise waypoint state
//    stage = STORING_POINTS;
//    dest_A.zero();
//    dest_B.zero();

//    return true;
//}


bool Copter::ModeAutorotate::init(bool ignore_checks)
{
     //Calculate recovery scaling values

    //Increase Z velocity limit in position controller to allow for high descent rates sometimes required for steady-state autorotation
    pos_control->set_max_speed_z(-2000, 500); //speed in cm/s

    //Record current x,y velocity to maintain initially
    _inital_vel_x = inertial_nav.get_velocity().x;
    _inital_vel_y = inertial_nav.get_velocity().y;


    return true;
}

void Copter::ModeAutorotate::run()
{
    
    // Kill power by setting HeliRSC channel to min value  <--- need to check effect on gas engine aircraft (i.e. dont want to shut down engines)
    SRV_Channels::set_output_scaled(SRV_Channel::k_heli_rsc, SRV_Channel::SRV_CHANNEL_LIMIT_MIN);
    
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
    //attitude_control->set_throttle_out(get_pilot_desired_throttle(),
    //                                   true,
    //                                   g.throttle_filt);
    


    _last_vel_x = inertial_nav.get_velocity().x;
    _last_vel_y = inertial_nav.get_velocity().y;

// do not use z-axis desired velocity feed forward
//    _flags.use_desvel_ff_z = false;

  //Alt in cm 
    if (inertial_nav.get_position().z > 300) {
        
        // set desired xy velocty 
        pos_control->set_desired_velocity_xy(_inital_vel_x, _inital_vel_y);
        
        
        // set target vert descent speed for steadystate autorotation
        pos_control->set_desired_velocity_z(-830);
        
    } else if (inertial_nav.get_position().z <= 300 && inertial_nav.get_position().z > 40) {
        
        
        // set desired xy velocty 
        pos_control->set_desired_velocity_xy(0, 0);
        
        // set target vert descent speed for initiation of flare
        pos_control->set_desired_velocity_z(-350);
    } else {
        
        // set desired xy velocty 
        pos_control->set_desired_velocity_xy(0, 0);
        
        
        // set target vert climb speed to initiate flare
        pos_control->set_desired_velocity_z(20);
    }



    // call xy-axis position controller
    //pos_control->update_xy_controller();

    // call z-axis position controller
    pos_control->update_z_controller();








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
