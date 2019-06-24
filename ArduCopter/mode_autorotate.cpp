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


    //Record initial speed to be maintained 
    _inital_airspeed = helispdhgtctrl->calc_speed_forward() * 100.0f;

    //Initialise hs error ring buffer with all ones
    for (int i = 0; i <= 9; i++) {
        _hs_error_history[i] = 1.0f;
    }
    //reset head speed error mean
    _hs_error_mean = 1.0f;

    //initialise head speed/collective controller
    arot_control->init_hs_controller();

    //initialise speed/height controller
    helispdhgtctrl->init_controller();

    message_counter = 0;

    //setting default starting switches
    phase_switch = ENTRY;

    //TODO: introduce the set entry flag back into controller when state machiene logic is ready to decide when to go to entry
    arot_control->set_entry_flag(true);

    return true;
}



void Copter::ModeAutorotate::run()
{

    //check if interlock becomes engaged
    if (motors->get_interlock()) {
        set_mode(copter.prev_control_mode, MODE_REASON_AUTOROTATION_BAILOUT);
    }

    // current time
    now = millis(); //milliseconds

    //initialise internal variables
    float des_z;
    float curr_alt = inertial_nav.get_position().z;     // Current altitude
    float curr_vel_z = inertial_nav.get_velocity().z;


    //----------------------------------------------------------------
    //                  State machine logic
    //----------------------------------------------------------------

     //Setting default phase switch positions
     nav_pos_switch = STRAIGHT_AHEAD;

    //altitude check to jump to correct flight phase if at low height
    if (curr_alt <= arot_control->get_td_alt()) {// && speed < 500) {
        //jump to touch down phase
        //phase_switch = TOUCH_DOWN;
        //arot_control->set_entry_flag(false);
    }

    //If head speed has been guided down to target speed then move onto glide
    if (phase_switch == ENTRY && !arot_control->get_entry_state()){
        //Head speed is stable and flight phase can be progressed to steady state glide
        phase_switch = SS_GLIDE;
        arot_control->set_entry_flag(false);

        //temp for debugging
        _flags.entry_initial = 1;
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

                // Set entry flag in head speed controller
                arot_control->set_entry_flag(true);

                 _flags.entry_initial = 0;

            }

            //Get airspeed target from parameters
            _aspeed = arot_control->get_speed_target();
            
            //Determine headspeed error penelty function 
            //float attitude_penalty = get_head_speed_penalty(arot_control->get_hs_error());

            //Apply airspeed penalty
            //_aspeed *= (1.0f - (attitude_penalty * 0.25f)); //this doesn't decay over time as its constantly refreshed

            //Apply pitch acceleration penalty
            _att_accel_max = arot_control->get_accel_max(); //refresh the acceleration limit to prevent penelty from being perminant
            //_att_accel_max *= (1.0f - attitude_penalty);  //Apply strong penelty scheme to ensure good head speed achieved.

            //Set desired air speed target
            helispdhgtctrl->set_desired_speed(_aspeed);

            //set acceleration limit
            helispdhgtctrl->set_max_accel(_att_accel_max);

            //run airspeed/attitude controller
            helispdhgtctrl->set_dt(G_Dt);
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

        {
            //steady state glide functions to be run only once
            if (_flags.ss_glide_initial == 1) {

                #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                    gcs().send_text(MAV_SEVERITY_INFO, "SS Glide Phase");
                #endif

                _flags.ss_glide_initial = 0;
            }

            //get target airspeed
            _aspeed = arot_control->get_speed_target();
            
            //Refresh acceleration limit.
            _att_accel_max = arot_control->get_accel_max();


            //Determine headspeed error penelty function 
            //float attitude_penalty = get_head_speed_penalty(arot_control->get_hs_error());

            //Apply airspeed penalty
            //_aspeed *= (1.0f - (attitude_penalty *0.25f)); //this doesn't decay over time as its constantly refreshed


            //TODO:  Try applying the airspeed penelty to the target pitch attitude (PIDS) instead of the airspeed target.
            //May get a far more crisp response for head speed recovery.  This could have the adverse effect of causing
            //the controller the controller to fight itself....


            //Apply pitch acceleration penalty
            //This penelty scheme resets every time the mode is initiated.
            //_att_accel_max -= attitude_penalty*_att_accel_max*0.005;  //Acceleration limit will decay oscillations in acceleration;

            //Set desired air speed target
            helispdhgtctrl->set_desired_speed(_aspeed);

            //set acceleration limit
            helispdhgtctrl->set_max_accel(_att_accel_max);

            //run airspeed/attitude controller
            helispdhgtctrl->set_dt(G_Dt);
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
            //attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pilot_roll, pilot_pitch, pilot_yaw_rate);
            
            break;
        }

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
        //gcs().send_text(MAV_SEVERITY_INFO, "Initial Speed = %.3f",_inital_airspeed);
        //gcs().send_text(MAV_SEVERITY_INFO, "--- --- --- ---");
        message_counter = 0;
    }
message_counter++;
#endif


    //Write to data flash log
    if (log_counter++ % 20 == 0) {
        DataFlash_Class::instance()->Log_Write("AROT", "TimeUS,InVz,Glide,HSMean,DPz,CRPM,AcL,ASTg,BufH,sum", "Qfffffffff",
                                                AP_HAL::micros64(),
                                               (double)curr_vel_z,
                                               (double)get_ned_glide_angle(),
                                               (double)_hs_error_mean,
                                               (double)des_z,
                                               (double)arot_control->get_rpm(),
                                               (double)_att_accel_max,
                                               (double)_aspeed,
                                               (double)_buffer_head,
                                               (double)_hs_error_sum);
    }

}


//this function determines the headspeed penelty.  The headspeed penelty is used to prevent the
//the attitude controller from slowing the headspeed too much.  The critical head speed is defined
//to be 0.1 less than the target.  That correspods to 10% of the hover head speed less than the
//target.  An exponential penelty function is used.
float Copter::ModeAutorotate::get_head_speed_penalty(float hs)
{

    const float weighting_exponent = -76.75f;

    const float min_hs_error = -0.1;

    float penelty_weighting = expf(weighting_exponent * (hs - min_hs_error));

    //Prevent penelty from being larger that 1
    if (penelty_weighting > 1.0f) {
        penelty_weighting = 1.0f;
    }

    return penelty_weighting;

}


// Determine the glide path angle based on NED frame velocities.  This method determines the instaintanious glide angle,
// not accounting for the wind.  Wind direction approximation/learning is done in another function, using this function.
float Copter::ModeAutorotate::get_ned_glide_angle(void)
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


// This function looks at the Head Speed (HS) trend over a number of iterations.  Once the mean HS error drops to
// within 5% of the target a true value is returned.  Mean is being measured over approximatley 1 sec, provided 
// mode is being called in the 100 Hz loop.
bool Copter::ModeAutorotate::is_hs_stable(void)
{
    //every 30th time step is accepted into the mean, to give a longer time projection without excesssive memory usage
    if (_time_step > 129) {

        //function stores hs error every 30 iterations.  Stored in a cirular buffer.
        _hs_error_history[_buffer_head] = arot_control->get_hs_error();

        //compute mean
        //_hs_error_sum += _hs_error_history[_buffer_head]; //add new data measurement
        //_hs_error_sum -= _hs_error_history[_buffer_tail]; //remove tail data measurement
        //_hs_error_mean = _hs_error_sum/10.0f;

        //update head buffer position
        if (_buffer_head < 9){
            _buffer_head++;
        } else {
            _buffer_head = 0;
            
            //on reset recalc mean
            _hs_error_mean = 0.0f;
            for (int i = 0; i <= 9; i++) {
                _hs_error_mean += _hs_error_history[i];
            }
            _hs_error_mean /= 10.0f;
        }

        //update tail buffer position
        if (_buffer_tail < 9){
            _buffer_tail++;
        } else {
            _buffer_tail = 0;
        }

        //reset time step
        _time_step = 1;

        //Check if mean is within 5% of target head speed
        if (_hs_error_mean > -0.05f  &&  _hs_error_mean < 0.05f) {
            return true;
        } else {
            return false;
        }


    } else {

        // time step counter is incremented
        _time_step++;

        // mean calculation hasn't changed since last update therefore must still be false
        return false;
    }
}


//TODO: winding direction approximation learning via circuling descent



void Copter::ModeAutorotate::errormessage(int message_number)
{
    
    
    
    
    
    
    
    
    

}





#endif