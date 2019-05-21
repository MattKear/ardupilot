#include "Copter.h"

#include "mode.h"

#include <utility>

#if MODE_AUTOROTATE_ENABLED == ENABLED

/*
 * Init and run calls for autorotate flight mode
 */


bool Copter::ModeAutorotate::init(bool ignore_checks)
{

#if FRAME_CONFIG != HELI_FRAME
    //Only allow trad heli to use autorotation mode
    return false;
#endif


#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    gcs().send_text(MAV_SEVERITY_INFO, "Autorotation initiated");
#endif

     phase_switch = ENTRY;
     
     //set inial switches
    _entry_initial = 1;
    _ss_glide_initial = 1;
    _flare_initial = 1;
    _touch_down_initial = 1;
    _level_initial = 1;
    _break_initial = 1;
    _straight_ahead_initial = 1;


    // initialize vertical speeds and acceleration limits and settings
    pos_control->set_speed_z(-2000, 500);
    pos_control->set_accel_z(200000);
    pos_control->set_true_desired_velocity_ff_z();
    pos_control->force_ff_accel_z();
    
    
    //pos_control->set_max_accel_xy(20000);//<----------------
    //pos_control->set_max_speed_xy(2000);//<----------------

    //Record current x,y velocity to maintain initially
    _inital_vel_x = inertial_nav.get_velocity().x;
    _inital_vel_y = inertial_nav.get_velocity().y;
    
    _inital_pos_x = inertial_nav.get_position().x;
    _inital_pos_y = inertial_nav.get_position().y;

    //initialise position controller
    pos_control->init_vel_controller_xyz();

    message_counter = 0;

    return true;
}

void Copter::ModeAutorotate::run()
{
    //initialise local variables
    
    // current time
    now = millis(); //milliseconds
    
    // Current altitude
    float curr_alt = inertial_nav.get_position().z;
    float curr_x = inertial_nav.get_position().x;
    float curr_y = inertial_nav.get_position().y;
    
    float curr_vel_z = inertial_nav.get_velocity().z;
    
    
    // Cut power by setting HeliRSC channel to min value  <--- need to check effect on gas engine aircraft (i.e. dont want to shut down engines)
    //  this needs to overide the rc interlock channel
    //SRV_Channels::set_output_scaled(SRV_Channel::k_heli_rsc, SRV_Channel::SRV_CHANNEL_LIMIT_MIN);
    
    // convert pilot input to lean angles
    //float target_roll, target_pitch;
    //get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
    //float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // call attitude controller
    //attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    
    
    

// state machine
switch (phase_switch) {

    case ENTRY:

        if (_entry_initial == 1) {

            #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                gcs().send_text(MAV_SEVERITY_INFO, "Entry");
            #endif

            _entry_initial = 0;
        }


        //Calculate entry scaling values
        _desired_v_z = -680; //(cm/s)

        pos_control->set_alt_target_from_climb_rate(_desired_v_z, G_Dt, true);

        pos_control->update_z_controller();

        nav_pos_switch = LEVEL;

        break;

    case SS_GLIDE:


        break;


    case FLARE:


        break;


    case TOUCH_DOWN:


        break;

}



switch (nav_pos_switch) {

    case STRAIGHT_AHEAD:


        break;


    case INTO_WIND:

         break;


    case NEAREST_RALLY:

         break;

    case BREAK:


        break;


    case LEVEL:

        if (_level_initial == 1) {
            #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                gcs().send_text(MAV_SEVERITY_INFO, "Stabilise Control Law for attitude");
            #endif

            _level_initial = 0;
        }


        // convert pilot input to lean angles
        float target_roll, target_pitch;
        get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

        // get pilot's desired yaw rate
        float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

        break;
}









//  These message outputs are purely for debugging purposes.  Will be removed in future.
if (message_counter == 300) {
//    gcs().send_text(MAV_SEVERITY_INFO, "Desired Vel %.2f",pos_control->get_vel_target_z());

//    gcs().send_text(MAV_SEVERITY_INFO, "--- --- --- ---");
    message_counter = 0;
}
message_counter++;



//Write to data flash log
    if (log_counter++ % 20 == 0) {
        DataFlash_Class::instance()->Log_Write("AROT", "TimeUS,CVz,CPz", "Qff",
                                               AP_HAL::micros64(),
                                               (double)curr_vel_z,
                                               (double)curr_alt);
    }

}



#endif