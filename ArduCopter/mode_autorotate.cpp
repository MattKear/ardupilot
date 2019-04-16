#include "Copter.h"

#include "mode.h"

#if MODE_AUTOROTATE_ENABLED == ENABLED

/*
 * Init and run calls for autorotate flight mode
 */


bool Copter::ModeAutorotate::init(bool ignore_checks)
{

     phase_switch = INITIATE;
     
     //set inial flare setting
     flare_initial = 1;
     

    //Increase Z velocity limit in position controller to allow for high descent rates sometimes required for steady-state autorotation
    pos_control->set_max_speed_z(-2000, 500); //speed in cm/s

    //Record current x,y velocity to maintain initially
    _inital_vel_x = inertial_nav.get_velocity().x;
    _inital_vel_y = inertial_nav.get_velocity().y;


    return true;
}

void Copter::ModeAutorotate::run()
{
    // current time
    now = millis(); //milliseconds
    dt = (last - now)/1000;  //delta time in seconds
    
    // Cut power by setting HeliRSC channel to min value  <--- need to check effect on gas engine aircraft (i.e. dont want to shut down engines)
    SRV_Channels::set_output_scaled(SRV_Channel::k_heli_rsc, SRV_Channel::SRV_CHANNEL_LIMIT_MIN);
    

    // convert pilot input to lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    //if (!motors->armed()) {
        // Motors should be Stopped
    //    motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
    //} else if (ap.throttle_zero) {
    //    // Attempting to Land
    //    motors->set_desired_spool_state(AP_Motors::DESIRED_GROUND_IDLE);
    //} else {
    //    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
    //}

    //if (motors->get_spool_mode() == AP_Motors::SHUT_DOWN) {
        // Motors Stopped
    //    attitude_control->set_yaw_target_to_current_heading();
    //    attitude_control->reset_rate_controller_I_terms();
    //} else if (motors->get_spool_mode() == AP_Motors::GROUND_IDLE) {
        // Landed
     //   attitude_control->set_yaw_target_to_current_heading();
    //    attitude_control->reset_rate_controller_I_terms();
    //} else if (motors->get_spool_mode() == AP_Motors::THROTTLE_UNLIMITED) {
        // clear landing flag above zero throttle
     //   if (!motors->limit.throttle_lower) {
     //       set_land_complete(false);
     //   }
    //}

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // output pilot's throttle
    //attitude_control->set_throttle_out(get_pilot_desired_throttle(),
    //                                   true,
    //                                   g.throttle_filt);




// do not use z-axis desired velocity feed forward
//    _flags.use_desvel_ff_z = false;

// state machine
switch (phase_switch) {
    
    case INITIATE:
     
     flare_aggression = 0.125; //0.12 a little too soft, 0.15 does react quick enough, 0.13 seems close but there is an agressive bounce back on touch down
     
     //Calculate recovery scaling values
     v_z_ss = -830; //(cm/s)
    
    //Scale velocities
    
    
    //apply recovery collective
    
    phase_switch = RECOVERY;
    break;
    
    
    case RECOVERY:
    //calculate flare height  <--- needs at least two loops of this mode to calc dt
    z_flare = (dt*v_z_ss)/(exp(-flare_aggression*dt));  //There is a risk here if dt changes later in the flare, it could lead to over/under shot flare
    
    // set target vert descent speed for steadystate autorotation
    pos_control->set_desired_velocity_z(v_z_ss);
    
    if ((v_z_ss - inertial_nav.get_velocity().z)/v_z_ss < 0.02) {
        //Steady-State Glide velocity achieved
        phase_switch = SS_GLIDE;
        
    } else if (inertial_nav.get_position().z <= z_flare){
        //low altitude skip to flare
        phase_switch = FLARE;
    }
    break;
    
    
    case SS_GLIDE:
    
    // set target vert descent speed for steadystate autorotation
    pos_control->set_desired_velocity_z(v_z_ss);
    
    if (inertial_nav.get_position().z <= z_flare){
        //low altitude skip to flare
        phase_switch = FLARE;
    }
    
    break;
    
    
    case FLARE:
    if (flare_initial == 1) {
        //set time that flare was initiated
        t_flare_initiate = now;
        
        //prevent future calls from reseting initiation of flare time
        flare_initial = 0;
        
        //v_z_ss remains unchanged for one more loop to prevent jerks in pos control velocity when exp(0) below.
    } else {

        //calculate z velocity for flare trajectory
        v_z_ss = -flare_aggression * z_flare * exp(-flare_aggression * (now - t_flare_initiate)/1000) * 100;  //cms
    }
    
    // set target vert speed for flare phase
    pos_control->set_desired_velocity_z(v_z_ss);
    
    if (abs(v_z_ss) <= 100) {
        phase_switch = TOUCH_DOWN;
    }

    break;


    case TOUCH_DOWN:
     // set target vert speed for landing complete.
    pos_control->set_desired_velocity_z(-100);

    break;

}


        // set desired xy velocty 
        //pos_control->set_desired_velocity_xy(_inital_vel_x, _inital_vel_y);


    // call xy-axis position controller
    //pos_control->update_xy_controller();

    // call z-axis position controller
    pos_control->update_z_controller();

    //update last time for next loop
    last = now;


}

#endif