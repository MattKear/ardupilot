#include "Copter.h"
#include <AC_AutorotationCtrl/AC_AutorotationCtrl.h>
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

//Check that interlock is disengaged
if (motors->get_interlock()) {
    gcs().send_text(MAV_SEVERITY_INFO, "Autorotation Mode change fail: Interlock Engaged");
    return false;
}


//  ToDo:  Add check to enforce RPM sensor available to allow mode change.


#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    gcs().send_text(MAV_SEVERITY_INFO, "Autorotation initiated");
#endif

     //Setting default phase switch positions
     phase_switch = ENTRY;
     nav_pos_switch = LEVEL;

     //set all inial flags to on
    _flags.entry_initial = 1;
    _flags.ss_glide_initial = 1;
    _flags.flare_initial = 1;
    _flags.touch_down_initial = 1;
    _flags.level_initial = 1;
    _flags.break_initial = 1;
    _flags.straight_ahead_initial = 1;


            //TEMP  KEEPING FOR REFERANCE ONLY 
    // initialize vertical speeds and acceleration limits and settings
    //pos_control->set_true_desired_velocity_ff_z();
    //pos_control->force_ff_accel_z();
    //pos_control->set_max_accel_xy(20000);//
    //pos_control->set_max_speed_xy(2000);//
    //initialise position controller
    //pos_control->init_vel_controller_xyz();

    //Record current x,y velocity to maintain initially
    _inital_vel_x = inertial_nav.get_velocity().x;
    _inital_vel_y = inertial_nav.get_velocity().y;
    
    _inital_pos_x = inertial_nav.get_position().x;
    _inital_pos_y = inertial_nav.get_position().y;

    message_counter = 0;

    return true;
}

void Copter::ModeAutorotate::run()
{
    
    //check if interlock becomes engaged
    if (motors->get_interlock()) {
        set_mode(copter.prev_control_mode, MODE_REASON_AUTOROTATION_BAILOUT);
    }


    //initialise internal variables
    float des_z;

    // current time
    now = millis(); //milliseconds

    // Current altitude
    float curr_alt = inertial_nav.get_position().z;
    float curr_vel_z = inertial_nav.get_velocity().z;
    
    
    //TODO:  Improve the speed etsimate here.  This is only temporary.
    //calculate speed in lateral plane
    float speed = sqrtf(inertial_nav.get_velocity().x * inertial_nav.get_velocity().x + inertial_nav.get_velocity().y * inertial_nav.get_velocity().y);

    //altitude check to jump to correct flight phase if at low height
    if (curr_alt <= rpm_control->get_td_alt() && speed < 500) {
        //jump to touch down phase
        phase_switch = TOUCH_DOWN;
    }
    
    
    nav_pos_switch = LEVEL;

// state machine
switch (phase_switch) {

    case ENTRY:

        //----------------------------------------------------------------
        // Using autorotation controller to manage entry phase of manouver
        //----------------------------------------------------------------
        if (_flags.entry_initial == 1) {

            #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                gcs().send_text(MAV_SEVERITY_INFO, "Entry");
            #endif

            
            _flags.entry_initial = 0;
            
            //Set tartget head speed in RPM
            rpm_control->use_entry_slew();
        }

        rpm_control->set_attitude_hs_mixing_flag (0);
        
        //float aspeed;
        //float test = attitude_control->airspeed_estimate(&aspeed);
        

        break;

    case SS_GLIDE:


        break;


    case FLARE:


        break;


    case TOUCH_DOWN:
        //----------------------------------------------------------------
        //      Using z position controller to manage final descent
        //----------------------------------------------------------------
        
        if (_flags.touch_down_initial == 1) {

            #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                gcs().send_text(MAV_SEVERITY_INFO, "Touch Down");
            #endif

            //pos_control->init_vel_controller_xyz();
            //pos_control->reset_i();

            //Extend limits of z velocity and z acceleration
            pos_control->set_speed_z(-800, 500);
            pos_control->set_accel_z(200000);

            //Get height that touchdown phase starts is initiated at
            _z_touch_down_start = curr_alt;

            //Set time that touchdown phase is initiated at
            _t_touch_down_initiate = now;

            // get collective aggression from parameter
            _collective_aggression = rpm_control->get_td_agression();

            _flags.touch_down_initial = 0;

        }

        //Calculate desired z
        des_z = _z_touch_down_start * 0.1f * expf(-_collective_aggression * (now - _t_touch_down_initiate)/1000.0f); // + terrain_offset (cm)

        pos_control->set_alt_target(des_z);

        pos_control->update_z_controller();

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

        if (_flags.level_initial == 1) {
            #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                gcs().send_text(MAV_SEVERITY_INFO, "Stabilise Control Law for attitude");
            #endif

            _flags.level_initial = 0;
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



//run RPM/collective controller
rpm_control->update_hs_glide_controller(G_Dt);


        // convert pilot input to lean angles
        float target_roll, target_pitch;
        get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

        // get pilot's desired yaw rate
        float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

        // call attitude controller
        //attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);



//  These message outputs are purely for debugging purposes.  Will be removed in future.
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (message_counter == 300) {
        gcs().send_text(MAV_SEVERITY_INFO, "Current RPM = %.2f",rpm_control->get_rpm());
        gcs().send_text(MAV_SEVERITY_INFO, "Head Speed Error = %.4f",rpm_control->get_rpm_error());
        //gcs().send_text(MAV_SEVERITY_INFO, "P AS %.2f",aspeed);
        //gcs().send_text(MAV_SEVERITY_INFO, "Test if airspeed true %.2f",test);
        //gcs().send_text(MAV_SEVERITY_INFO, "Target Pitch %.5f",target_pitch);
        gcs().send_text(MAV_SEVERITY_INFO, "--- --- --- ---");
        message_counter = 0;
    }
message_counter++;
#endif


//Write to data flash log
    if (log_counter++ % 20 == 0) {
        DataFlash_Class::instance()->Log_Write("AROT", "TimeUS,CVz,CPz,DPz,CRPM", "Qffff",
                                               AP_HAL::micros64(),
                                               (double)curr_vel_z,
                                               (double)curr_alt,
                                               (double)des_z,
                                               (double)rpm_control->get_rpm());
    }

}






#endif