#include "Copter.h"
#include <AC_Autorotation/AC_Autorotation.h>
#include "mode.h"

#include <utility>

#if MODE_AUTOROTATE_ENABLED == ENABLED

bool ModeAutorotate::init(bool ignore_checks)
{
#if FRAME_CONFIG != HELI_FRAME
    //Only allow trad heli to use autorotation mode
    return false;
#endif

//Check that mode is enabled
if (!arot->is_enable()) {
    gcs().send_text(MAV_SEVERITY_INFO, "Autorotation Mode Change Fail: Mode Not Enabled");
    return false;
}

//Check that interlock is disengaged
if (motors->get_interlock()) {
    gcs().send_text(MAV_SEVERITY_INFO, "Autorotation Mode Change Fail: Interlock Engaged");
    return false;
}

    //initialise head speed/collective controller
    //This must be done before RPM value is fetched
    arot->init_hs_controller();

    // Retrive rpm and start rpm sensor health checks
    _initial_rpm = arot->get_rpm(true);

    //Display message 
    gcs().send_text(MAV_SEVERITY_INFO, "Autorotation initiated");

    //Retrieve parameter values from autorotation library
    arot->set_param_values(&_param_head_speed_set_point, &_param_accel_max, &_param_target_fwd_speed, &_param_td_alt, &_param_col_entry_cutoff_freq, &_param_col_glide_cutoff_freq, &_param_bail_time);

     //set all inial flags to on
    _flags.entry_initial = 1;
    _flags.ss_glide_initial = 1;
    _flags.flare_initial = 1;
    _flags.touch_down_initial = 1;
    _flags.level_initial = 1;
    _flags.break_initial = 1;
    _flags.straight_ahead_initial = 1;
    _flags.bail_out_initial = 1;
    _msg_flags.bad_rpm = true;

    //Prevent divide by zero error
    if (_param_head_speed_set_point < 500) {
        _param_head_speed_set_point = 500;  //Making sure that hover rpm is not unreasonably low
    }

    //initialise speed/height controller
    helispdhgtctrl->init_controller();

    //setting default starting switches
    phase_switch = ENTRY;

    //set entry timer
    _entry_time_remain = AUTOROTATE_ENTRY_TIME;

    //The decay rate to reduce the head speed from the current to the target
    _hs_decay = ((_initial_rpm/_param_head_speed_set_point) - HEAD_SPEED_TARGET_RATIO) / AUTOROTATE_ENTRY_TIME;//AUTOROTATE_HS_ADJUST_TIME;

    return true;
}



void ModeAutorotate::run()
{

    //Check if interlock becomes engaged
    if (motors->get_interlock() && !copter.ap.land_complete) {
        phase_switch = BAIL_OUT;
    } else if (motors->get_interlock() && copter.ap.land_complete) {
        //Aircraft is landed and no need to bail out
        set_mode(copter.prev_control_mode, MODE_REASON_AUTOROTATION_BAILOUT);
    }

    //Current time
    now = millis(); //milliseconds

    //Initialise internal variables
    float curr_alt = inertial_nav.get_position().z;     // Current altitude
    float curr_vel_z = inertial_nav.get_velocity().z;   // Current vertical descent

    //----------------------------------------------------------------
    //                  State machine logic
    //----------------------------------------------------------------

     //Setting default phase switch positions
     nav_pos_switch = USER_CONTROL_STABILISED;

    //altitude check to jump to correct flight phase if at low height
    if (curr_alt <= _param_td_alt) {
        //jump to touch down phase
        //phase_switch = TOUCH_DOWN;  //currently switched off in this version.
    }

    //Timer from entry phase to progress to glide phase
    if (phase_switch == ENTRY){

        //update entry phase timer
        _entry_time_remain -= G_Dt;

        if (_entry_time_remain < 0.0f) {
            //Flight phase can be progressed to steady state glide
            phase_switch = SS_GLIDE;
        }

    }


    //----------------------------------------------------------------
    //                  State machine actions
    //----------------------------------------------------------------
    switch (phase_switch) {

        case ENTRY:
        {
            //Entry phase functions to be run only once
            if (_flags.entry_initial == 1) {

                #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                    gcs().send_text(MAV_SEVERITY_INFO, "Entry Phase");
                #endif

                //set following trim low pass cut off frequency
                arot->set_col_cutoff_freq(_param_col_entry_cutoff_freq);

                //target head speed is set to rpm at initiation to prevent unwanted changes in attitude
                _target_head_speed = _initial_rpm/_param_head_speed_set_point;

                //Set desired forward speed target
                _fwd_speed_target = _param_target_fwd_speed;
                helispdhgtctrl->set_desired_speed(_fwd_speed_target);

                //set acceleration limit
                helispdhgtctrl->set_max_accel(_param_accel_max);

                //Prevent running the initial entry functions again
                _flags.entry_initial = 0;

            }

            //Slowly change the target head speed until the target 
            //head speed matches the parameter defined value
            if (arot->get_rpm() > HEAD_SPEED_TARGET_RATIO*1.005f  ||  arot->get_rpm() < HEAD_SPEED_TARGET_RATIO*0.995f) {
                _target_head_speed -= _hs_decay*G_Dt;
            } else {
                _target_head_speed = HEAD_SPEED_TARGET_RATIO;
            }

            //set target head speed in head speed controller
            arot->set_target_head_speed(_target_head_speed);

            //run airspeed/attitude controller
            helispdhgtctrl->set_dt(G_Dt);
            helispdhgtctrl->update_speed_controller();

            //retrieve pitch target from helispdhgtctrl 
            _pitch_target = helispdhgtctrl-> get_pitch();

            //update controllers
            _flags.bad_rpm = arot->update_hs_glide_controller(G_Dt); //run head speed/ collective controller

            break;
        }

        case SS_GLIDE:
        {
            //steady state glide functions to be run only once
            if (_flags.ss_glide_initial == 1) {

                #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                    gcs().send_text(MAV_SEVERITY_INFO, "SS Glide Phase");
                #endif

                //set following trim low pass cut off frequency
                arot->set_col_cutoff_freq(_param_col_glide_cutoff_freq);

                //Set desired forward speed target
                _fwd_speed_target = _param_target_fwd_speed;
                helispdhgtctrl->set_desired_speed(_fwd_speed_target);

                //set target head speed in head speed controller
                _target_head_speed = HEAD_SPEED_TARGET_RATIO;  //Ensure target hs is set to glide incase hs hasent reached target for glide
                arot->set_target_head_speed(_target_head_speed);

                //set acceleration limit
                helispdhgtctrl->set_max_accel(_param_accel_max);

                //Prevent running the initial glide functions again
                _flags.ss_glide_initial = 0;
            }

            //run airspeed/attitude controller
            helispdhgtctrl->set_dt(G_Dt);
            helispdhgtctrl->update_speed_controller();

            //retrieve pitch target from helispdhgtctrl 
            _pitch_target = helispdhgtctrl-> get_pitch();

            //update controllers
            _flags.bad_rpm = arot->update_hs_glide_controller(G_Dt); //run head speed/ collective controller
            //attitude controller is updated in navigation switch-case statements

            break;
        }

        case FLARE:
        {
            break;
        }

        case TOUCH_DOWN:
        {
            if (_flags.touch_down_initial == 1) {
                //Functions and settings to be done once are done here.

                #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                    gcs().send_text(MAV_SEVERITY_INFO, "Touch Down Initiated");
                #endif

                //Set limits of z velocity and z acceleration
                pos_control->set_max_speed_z(-800, 500);
                pos_control->set_max_accel_z(200000);

                //Get height that touchdown phase starts is initiated at
                _z_touch_down_start = curr_alt;

                //Set time that touchdown phase is initiated at
                _t_touch_down_initiate = now;

                //Get collective aggression from parameter
                _collective_aggression = 0.15f;

                _flags.touch_down_initial = 0;
            }

            //Calculate desired z
            _des_z = _z_touch_down_start * 0.1f * expf(-_collective_aggression * (now - _t_touch_down_initiate)/1000.0f); // + terrain_offset (cm)

            pos_control->set_alt_target(_des_z);

            pos_control->update_z_controller();

            break;
        }

        case BAIL_OUT:
        {
        if (_flags.bail_out_initial == 1) {
                //functions and settings to be done once are done here.

                #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                    gcs().send_text(MAV_SEVERITY_INFO, "Bailing Out of Autorotation");
                #endif

                //Set bail out timer remaining equal to the paramter value
                if (_param_bail_time < BAILOUT_RAMP_TIME) {
                    bail_time_remain = BAILOUT_RAMP_TIME;
                } else {
                    bail_time_remain = _param_bail_time;
                }

                //set initial target vertical speed
                _desired_v_z = curr_vel_z;

                //Initialise position and desired velocity
                if (!pos_control->is_active_z()) {
                    pos_control->relax_alt_hold_controllers(arot->get_last_collective());
                }

                //Get pilot parameter limits
                float pilot_spd_dn = -get_pilot_speed_dn();
                float pilot_spd_up = g.pilot_speed_up;

                //set speed limit
                pos_control->set_max_speed_z(curr_vel_z, pilot_spd_up);

                float pilot_des_v_z = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
                pilot_des_v_z = constrain_float(pilot_des_v_z, pilot_spd_dn, pilot_spd_up);

                //calculate target climb rate adjustment to transition from bail out decent speed to requested climb rate on stick.
                _target_climb_rate_adjust = (curr_vel_z - pilot_des_v_z)/(_param_bail_time - BAILOUT_RAMP_TIME); //accounting for 0.5s motor spool time

                //calculate pitch target adjustment rate to return to level
                _target_pitch_adjust = _pitch_target/(_param_bail_time - BAILOUT_RAMP_TIME); //accounting for 0.5s motor spool time_param_bail_time;

                //set acceleration limit
                pos_control->set_max_accel_z(abs(_target_climb_rate_adjust));

                motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

                _flags.bail_out_initial = 0;
            }

        if (bail_time_remain >= BAILOUT_RAMP_TIME) {
            //update desired vertical speed and pitch target after the bailout ramp timer has completed
            _desired_v_z -= _target_climb_rate_adjust*G_Dt;
            _pitch_target -= _target_pitch_adjust*G_Dt;
        }
        //set position controller
        pos_control->set_alt_target_from_climb_rate(_desired_v_z, G_Dt, false);

        //update controllers
        pos_control->update_z_controller();

        //update bail out timer
        bail_time_remain -= G_Dt;
        
        if (bail_time_remain <= 0.0f) {
            //bail out timer complete.  Change flight mode.
            set_mode(copter.prev_control_mode, MODE_REASON_AUTOROTATION_BAILOUT);
        }

        break;
        }
    }


    switch (nav_pos_switch) {

        case USER_CONTROL_STABILISED:
        {
            //Operator is in control of roll and yaw.  Controls act as if in stabilise flight mode.  Pitch is controlled 
            //by speed-height controller.
            float pilot_roll, pilot_pitch;
            get_pilot_desired_lean_angles(pilot_roll, pilot_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

            //get pilot's desired yaw rate
            float pilot_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

            //pitch target is calculated in autorotation phase switch above
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pilot_roll, _pitch_target, pilot_yaw_rate);
            //attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pilot_roll, pilot_pitch, pilot_yaw_rate);
            break;
        }

        case STRAIGHT_AHEAD:
        {
            break;
        }

        case INTO_WIND:
        {
            break;
        }

        case NEAREST_RALLY:
        {
            break;
        }
    }

    //Output warning messaged if rpm signal is bad
    if (_flags.bad_rpm) {
        warning_message(1);
    }

} //end function run()


// Determine the glide path angle based on NED frame velocities.  This method determines the instaintanious glide angle,
// not accounting for the wind.  This function is not yet used.  It will be adopted in the future for selection of most efficient
// forward flight speed.
float ModeAutorotate::get_ned_glide_angle(void)
{
    float sink_velocity = inertial_nav.get_velocity().z; //(cm/s)

    //Convert sink velocity to m/s and convert sink velocity to be positive down
    sink_velocity /= -100.0f; //(m/s)

    //remove climb rates from sink velocity
    if (sink_velocity < 0) {
        sink_velocity = 0;
    }

    //Determine ground speed
    Vector2f _groundspeed_vector = ahrs.groundspeed_vector();
    float ground_speed_forward = _groundspeed_vector.x*ahrs.cos_yaw() + _groundspeed_vector.y*ahrs.sin_yaw(); //(m/s)

    //catch divide by zero
    if (ground_speed_forward < 0.01f) {
        ground_speed_forward = 0.01f;
    }

    //Calculate glide slope
    float windless_glide_slope = atanf(sink_velocity/ground_speed_forward);

    return windless_glide_slope;
}


void ModeAutorotate::warning_message(uint8_t message_n)
{
    switch (message_n) {
        case 1:
        {
            if (_msg_flags.bad_rpm) {
                //bad rpm sensor health.
                gcs().send_text(MAV_SEVERITY_INFO, "Warning: Poor RPM Sensor Health");
                gcs().send_text(MAV_SEVERITY_INFO, "Action: Minimum Collective Applied");
                _msg_flags.bad_rpm = false;
            }
            break;
        }
    }
}

#endif