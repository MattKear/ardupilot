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

     //set all inial flags to on
    _flags.entry_initial = 1;
    _flags.ss_glide_initial = 1;
    _flags.flare_initial = 1;
    _flags.touch_down_initial = 1;
    _flags.level_initial = 1;
    _flags.break_initial = 1;
    _flags.straight_ahead_initial = 1;

    //Record current x,y velocity to maintain initially
    _inital_vel_x = inertial_nav.get_velocity().x;
    _inital_vel_y = inertial_nav.get_velocity().y;
    
    _inital_pos_x = inertial_nav.get_position().x;
    _inital_pos_y = inertial_nav.get_position().y;

    //initialise speed/height controller
    helispdhgtctrl->init_controller();

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


    //----------------------------------------------------------------
    //                  State machine logic
    //----------------------------------------------------------------

     //Setting default phase switch positions
     phase_switch = ENTRY;
     nav_pos_switch = STRAIGHT_AHEAD;

    //altitude check to jump to correct flight phase if at low height
    if (curr_alt <= arot_control->get_td_alt()) {// && speed < 500) {
        //jump to touch down phase
        phase_switch = TOUCH_DOWN;
    }
    
    

    //----------------------------------------------------------------
    //                  State machine actions
    //----------------------------------------------------------------
    switch (phase_switch) {

        case ENTRY:
        {
            if (_flags.entry_initial == 1) {

                #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                    gcs().send_text(MAV_SEVERITY_INFO, "Entry Phase");
                #endif

                _flags.entry_initial = 0;

                //TODO:  Set entry flag in head speed controller
                //Set tartget head speed in RPM
                arot_control->use_entry_slew();
            }

            //get target airspeed for best glide efficiency
            float aspeed = arot_control->get_speed_target();

            //Set desired air speed target
            helispdhgtctrl->set_desired_speed(aspeed);

            helispdhgtctrl->set_max_accel(arot_control->get_accel_max());



            //run airspeed/attitude controller
            helispdhgtctrl->update_speed_controller();

            //retrieve pitch target from helispdhgtctrl 
            _pitch_target = helispdhgtctrl-> get_pitch();



            //temp
            float pilot_roll, pilot_pitch;
            get_pilot_desired_lean_angles(pilot_roll, pilot_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

            //get pilot's desired yaw rate
            float pilot_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

            //update controllers
            arot_control->update_hs_glide_controller(G_Dt); //run head speed/ collective controller
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pilot_roll, _pitch_target, pilot_yaw_rate);

            break;

        }
        case SS_GLIDE:


            break;


        case FLARE:


            break;


        case TOUCH_DOWN:

            if (_flags.touch_down_initial == 1) {
                //functions and settings to be done once are done here.

                #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                    gcs().send_text(MAV_SEVERITY_INFO, "Touch Down Initiated");
                #endif

                //set limits of z velocity and z acceleration
                pos_control->set_speed_z(-800, 500);
                pos_control->set_accel_z(200000);

                //Get height that touchdown phase starts is initiated at
                _z_touch_down_start = curr_alt;

                //Set time that touchdown phase is initiated at
                _t_touch_down_initiate = now;

                // get collective aggression from parameter
                _collective_aggression = arot_control->get_td_agression();

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

        //    if (_flags.straight_ahead_initial == 1) {
                //functions and settings to be done once are done here.

        //        #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        //            gcs().send_text(MAV_SEVERITY_INFO, "Straight ahead");
         //       #endif

        //        _flags.straight_ahead_initial = 0;
        //    }

        //    // convert pilot input to lean angles
        //    float target_roll, target_pitch;
        //    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

        //    // get pilot's desired yaw rate
        //    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

        //    // call attitude controller
         //   attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);


            break;


        case INTO_WIND:

            break;


        case NEAREST_RALLY:

            break;

        case BREAK:


            break;


        case LEVEL:

            break;

    }






//  These message outputs are purely for debugging purposes.  Will be removed in future.
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (message_counter == 300) {
        gcs().send_text(MAV_SEVERITY_INFO, "Current RPM = %.2f",arot_control->get_rpm());
        gcs().send_text(MAV_SEVERITY_INFO, "Head Speed Error = %.4f",arot_control->get_rpm_error());
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
                                               (double)arot_control->get_rpm());
    }

}






#endif