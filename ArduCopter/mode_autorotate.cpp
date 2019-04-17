#include "Copter.h"

#include "mode.h"

#if MODE_AUTOROTATE_ENABLED == ENABLED

/*
 * Init and run calls for autorotate flight mode
 */


bool Copter::ModeAutorotate::init(bool ignore_checks)
{

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    gcs().send_text(MAV_SEVERITY_INFO, "Autorotation initiated");
#endif

     phase_switch = INITIATE;
     
     //set inial switches
    recovery_initial = 1;
    ss_glide_initial = 1;
    flare_initial = 1;
    touch_down_initial = 1;

     

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
    // current time
    now = millis(); //milliseconds
    
    
    //checked and dt does vary a little range 0.001 -0.007, mean 0.003
    
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

   //gcs().send_text(MAV_SEVERITY_INFO, "Vz %.4f",inertial_nav.get_position().z);



// do not use z-axis desired velocity feed forward
//    _flags.use_desvel_ff_z = false;

// state machine
switch (phase_switch) {
    
    case INITIATE:
     
     flare_aggression = 0.15; //
     
     //Calculate recovery scaling values
     v_z_ss = -830; //(cm/s)
    
    //Scale velocities
    
    
    //apply recovery collective
    
    phase_switch = RECOVERY;
    break;
    
    
    case RECOVERY:
    
    if (recovery_initial == 1) {
        
        //calculate flare height  <--- needs at least two loops of this mode to calc dt
        dt = (last - now)/1000;  //delta time in seconds
        z_flare = (dt*v_z_ss)/(exp(-flare_aggression*dt)) * 1000.0f;  //(mm)  //There is a risk here if dt changes later in the flare, it could lead to over/under shot flare
    
        #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            gcs().send_text(MAV_SEVERITY_INFO, "recovery");
            gcs().send_text(MAV_SEVERITY_INFO, "flare height %.3f",z_flare);
        #endif

        recovery_initial = 0;
    }
    
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
    
    if (ss_glide_initial == 1) {
        #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            gcs().send_text(MAV_SEVERITY_INFO, "steady state glide");
        #endif

        ss_glide_initial = 0;
    }
    
    break;
    
    
    case FLARE:
    if (flare_initial == 1) {
        //set time that flare was initiated
        t_flare_initiate = now;
        
        //prevent future calls from reseting initiation of flare time
        flare_initial = 0;
        
        //v_z_ss remains unchanged for one more loop to prevent jerks in pos control velocity when exp(0) below.
        
        //message to ground control station to inform that flare has engaged
        #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            gcs().send_text(MAV_SEVERITY_INFO, "Flare initiated");
        #endif
    } else {

        //calculate z velocity for flare trajectory
        v_z_ss = -flare_aggression * z_flare * exp(-flare_aggression * (now - t_flare_initiate)/1000.0f);  //cms
        

    }
    
    // set target vert speed for flare phase
    pos_control->set_desired_velocity_z(v_z_ss);
    
    if (v_z_ss >= -150) {
        v_z_ss = -150;
        phase_switch = TOUCH_DOWN;
    }

    break;


    case TOUCH_DOWN:
     // set target vert speed for landing complete.
    pos_control->set_desired_velocity_z(v_z_ss);
    
    if (touch_down_initial == 1) {
        #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            gcs().send_text(MAV_SEVERITY_INFO, "touch down");
        #endif

        touch_down_initial = 0;
    }

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


    if (message_counter == 300) {
        gcs().send_text(MAV_SEVERITY_INFO, "Desired Vel %.2f",v_z_ss);
        gcs().send_text(MAV_SEVERITY_INFO, "Actual Vel %.2f",inertial_nav.get_velocity().z);
        gcs().send_text(MAV_SEVERITY_INFO, "Actual Height %.2f",inertial_nav.get_position().z);
        gcs().send_text(MAV_SEVERITY_INFO, "Flare time %.4f",now - t_flare_initiate);
        gcs().send_text(MAV_SEVERITY_INFO, "--- --- --- ---");
        message_counter = 0;
    }
    message_counter++;
}

#endif