#include "Copter.h"

#include "mode.h"
#include <AC_PID/AC_P.h>               // P library
#include <AC_PID/AC_PID.h>             // PID library

#if MODE_AUTOROTATE_ENABLED == ENABLED

/*
 * Init and run calls for autorotate flight mode
 */


bool Copter::ModeAutorotate::init(bool ignore_checks)
{

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    gcs().send_text(MAV_SEVERITY_INFO, "Autorotation initiated");
#endif

     phase_switch = RECOVERY;
     
     //set inial switches
    recovery_initial = 1;
    ss_glide_initial = 1;
    flare_initial = 1;
    touch_down_initial = 1;
    
    //reset velocity error and filter
    //_vel_error.z = 0;
    //_vel_error_filter.reset(0);
    //_flags.reset_rate_to_accel_z = false;
     

    //Increase Z velocity limit in position controller to allow for high descent rates sometimes required for steady-state autorotation
    pos_control->set_max_speed_z(-20000, 500); //speed in cm/s

    //Record current x,y velocity to maintain initially
    _inital_vel_x = inertial_nav.get_velocity().x;
    _inital_vel_y = inertial_nav.get_velocity().y;



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
    
    float curr_vel_z = inertial_nav.get_velocity().z;
    
    // clear limit flags?????  --not tried this yet, need to confirm what the limit flags do
    //_limit.pos_up = false;
    //_limit.pos_down = false;
    //_limit.vel_up = false;
    //_limit.vel_down = false;
    
    //checked and dt does vary a little range 0.001 -0.007, mean 0.003
    
    // Cut power by setting HeliRSC channel to min value  <--- need to check effect on gas engine aircraft (i.e. dont want to shut down engines)
    SRV_Channels::set_output_scaled(SRV_Channel::k_heli_rsc, SRV_Channel::SRV_CHANNEL_LIMIT_MIN);
    
    // convert pilot input to lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());


    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // output pilot's throttle
    //attitude_control->set_throttle_out(get_pilot_desired_throttle(),
    //                                   true,
    //                                   g.throttle_filt);

   //gcs().send_text(MAV_SEVERITY_INFO, "Vz %.4f",inertial_nav.get_position().z);



//use z-axis desired velocity feed forward
//    _flags.use_desvel_ff_z = true;

// state machine
switch (phase_switch) {
    
    case RECOVERY:
    
        if (recovery_initial == 1) {
        
            flare_aggression = 0.15; 
            
            z_flare = (-G_Dt*desired_v_z)/(expf(-flare_aggression*G_Dt)) * 1000.0f;  //(mm)  //There is a risk here if dt changes later in the flare, it could lead to over/under shot flare
    
            #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                gcs().send_text(MAV_SEVERITY_INFO, "recovery");
                gcs().send_text(MAV_SEVERITY_INFO, "flare height %.3f",z_flare);
            #endif

            recovery_initial = 0;
        }
    

        //Calculate recovery scaling values
        desired_v_z = -680; //(cm/s)
    
        //Calculate desired z from recovery velocity
        des_z = curr_alt + desired_v_z*G_Dt;
    
        //set inital acceleration
        required_accel_z = (desired_v_z - curr_vel_z)/G_Dt;
          //may need to apply check to prevent +accelerations????
    
        //apply forced recovery collective???
    
    
        // set target vert descent speed for steadystate autorotation<---defunct
        //pos_control->set_desired_velocity_z(desired_v_z);
    
        if ((desired_v_z - curr_vel_z)/desired_v_z < 0.02) {  //if descent velocity is within 2% of desired ss vel then switch
            //Steady-State Glide velocity achieved
            phase_switch = SS_GLIDE;
        
        } else if (curr_alt <= z_flare){
            //low altitude skip to flare
            phase_switch = FLARE;
        }
    

    
    
        break;
    
    
    case SS_GLIDE:
    
        if (ss_glide_initial == 1) {
            #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                gcs().send_text(MAV_SEVERITY_INFO, "steady state glide");
            #endif

            ss_glide_initial = 0;
        }
        
        //Calculate desired z from steady state autorotation velocity
        des_z = curr_alt + desired_v_z*G_Dt;
    
        //set acceleration
        required_accel_z = (desired_v_z - curr_vel_z)/G_Dt;
    
        // set target vert descent speed for steadystate autorotation  <---defunct
        //pos_control->set_desired_velocity_z(desired_v_z);
    
        if (curr_alt <= z_flare){
            //Initiate flare phase
            phase_switch = FLARE;
        }

        break;
    
    
    case FLARE:
        if (flare_initial == 1) {
            //set time that flare was initiated
            t_flare_initiate = now;
        
            //prevent future calls from reseting initiation of flare time
            flare_initial = 0;
        
            //desired_v_z remains unchanged for one more loop to prevent jerks in pos control velocity when exp(0) below.
            //Calculate desired z from seteady state autorotation velocity
            des_z = curr_alt + desired_v_z*G_Dt;
    
            //set acceleration
            required_accel_z = (desired_v_z - curr_vel_z)/G_Dt;
        
            //message to ground control station to inform that flare has engaged
            #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                gcs().send_text(MAV_SEVERITY_INFO, "Flare initiated");
            #endif
        } else {

            //Calculate desired z
            des_z = z_flare * expf(-flare_aggression * (now - t_flare_initiate)/1000.0f); // + terrain_offset
            
            
            //calculate desired z velocity for flare trajectory
            desired_v_z = -flare_aggression * z_flare * expf(-flare_aggression * (now - t_flare_initiate)/1000.0f);  //(cm/s)
            
            //Account for position errors by adjusting velocity
            desired_v_z += (des_z_last - curr_alt)/G_Dt; //(cm/s)
    
            //set acceleration
            required_accel_z = (desired_v_z - curr_vel_z)/G_Dt; //(cm/s/s)
        }
    
        // set target vert speed for flare phase  <-----defunct
        //pos_control->set_desired_velocity_z(desired_v_z);
    
        if (desired_v_z >= -150) {
            desired_v_z = -150;
            phase_switch = TOUCH_DOWN;
        }

        break;


    case TOUCH_DOWN:
        // set target vert speed for landing complete. <-----defunct
        //pos_control->set_desired_velocity_z(desired_v_z);
    
        //Calculate desired z from touch down velocity
        des_z = curr_alt + desired_v_z*G_Dt;
    
        //set acceleration
        required_accel_z = (desired_v_z - curr_vel_z)/G_Dt;
    
        if (touch_down_initial == 1) {
            #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                gcs().send_text(MAV_SEVERITY_INFO, "touch down");
            #endif

            touch_down_initial = 0;
        }

        break;

}


//Calculate the velocity error
v_z_error = desired_v_z - curr_vel_z;


//pos_control->_accel_target.z = _p_vel_z.get_p(v_z_error);
//pos_control->_accel_target.z += required_accel_z;
//pos_control->set_desired_accel_z(required_accel_z);
// --------------------------------------------------------


pos_control->set_accel_z(required_accel_z);

pos_control->acceleration_to_throtte();


// set desired xy velocty 
//pos_control->set_desired_velocity_xy(_inital_vel_x, _inital_vel_y);


// call xy-axis position controller
//pos_control->update_xy_controller();

// call z-axis position controller  <----- now defunct
//pos_control->update_z_controller();

//saving last targeted z position for corrections in the next iteration
des_z_last = des_z;


if (message_counter == 300) {
    gcs().send_text(MAV_SEVERITY_INFO, "Desired Vel %.2f",desired_v_z);
    gcs().send_text(MAV_SEVERITY_INFO, "Actual Vel %.2f",curr_vel_z);
    gcs().send_text(MAV_SEVERITY_INFO, "Actual Height %.2f",curr_alt);
    gcs().send_text(MAV_SEVERITY_INFO, "Flare time %.4f",now - t_flare_initiate);
    gcs().send_text(MAV_SEVERITY_INFO, "--- --- --- ---");
    message_counter = 0;
}
message_counter++;
}

#endif