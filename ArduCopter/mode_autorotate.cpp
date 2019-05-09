#include "Copter.h"

#include "mode.h"

#include <DataFlash/DataFlash.h>

#if MODE_AUTOROTATE_ENABLED == ENABLED

/*
 * Init and run calls for autorotate flight mode
 */


bool Copter::ModeAutorotate::init(bool ignore_checks)
{

#if FRAME_CONFIG != HELI_FRAME
    //Only allow trad heli to use autorotation mode
    return false
#endif


#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    gcs().send_text(MAV_SEVERITY_INFO, "Autorotation initiated");
#endif

     phase_switch = RECOVERY;
     
     //set inial switches
    recovery_initial = 1;
    ss_glide_initial = 1;
    flare_initial = 1;
    touch_down_initial = 1;
     

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
    
    float curr_vel_z = inertial_nav.get_velocity().z;
    
    
    // Cut power by setting HeliRSC channel to min value  <--- need to check effect on gas engine aircraft (i.e. dont want to shut down engines)
    //  this needs to overide the rc interlock channel
    SRV_Channels::set_output_scaled(SRV_Channel::k_heli_rsc, SRV_Channel::SRV_CHANNEL_LIMIT_MIN);
    
    // convert pilot input to lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    
    
    

// state machine
switch (phase_switch) {
    
    case RECOVERY:
    
        if (recovery_initial == 1) {
        
            flare_aggression = 0.15; 
            
            z_flare = (-G_Dt*desired_v_z)/(expf(-flare_aggression*G_Dt)) * 1000.0f;  //(mm)  //There is a risk here if dt changes later in the flare, it could lead to over/under shot flare
    
            //Check z_flare has not been driven to be very small due to erronious G_Dt values
            if (z_flare < 400) {
                z_flare = 400;
                gcs().send_text(MAV_SEVERITY_INFO, "flare height adjusted");
            }
    
    
            #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                gcs().send_text(MAV_SEVERITY_INFO, "recovery");
                gcs().send_text(MAV_SEVERITY_INFO, "flare height %.3f",z_flare);
            #endif

            recovery_initial = 0;
        }
    

        //Calculate recovery scaling values
        desired_v_z = -680; //(cm/s)
    
        //Calculate desired z from recovery velocity
        des_z = curr_alt/10.0f + desired_v_z*G_Dt;  //(cm)
    
    
        //apply forced recovery collective???
    
    
        if ((desired_v_z - curr_vel_z)/desired_v_z < 0.02) {  //if descent velocity is within 2% of desired ss vel then switch
            //Steady-State Glide velocity achieved
            //phase_switch = SS_GLIDE;
        
        } else if (curr_alt <= z_flare){
            //low altitude skip to flare
            //phase_switch = FLARE;
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
        des_z = curr_alt/10.0f + desired_v_z*G_Dt;  //(cm)
        // set position controller targets//<----------------
        //pos_control->set_alt_target_from_climb_rate_ff(desired_v_z, G_Dt, false);//<----------------
    
    
        if (curr_alt <= z_flare){
            //Initiate flare phase
            //phase_switch = FLARE;
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
    
            //position at flare initiation
            flare_pos_x = inertial_nav.get_position().x;
            flare_pos_y = inertial_nav.get_position().y;
        
            //message to ground control station to inform that flare has engaged
            #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                gcs().send_text(MAV_SEVERITY_INFO, "Flare initiated");
            #endif
        } else {

            //Calculate desired z
            des_z = z_flare * 0.1f * expf(-flare_aggression * (now - t_flare_initiate)/1000.0f); // + terrain_offset (cm)
            
        }
    
        //if (curr_vel_z >= -150) {
         //   phase_switch = TOUCH_DOWN;
        //}
        
        xy_pos_switch = BREAK;

        break;


    case TOUCH_DOWN:
    
        desired_v_z = -150;
    
        //Calculate desired z from touch down velocity
        des_z = curr_alt + desired_v_z*G_Dt;

        if (touch_down_initial == 1) {
            #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                gcs().send_text(MAV_SEVERITY_INFO, "touch down");
            #endif

            touch_down_initial = 0;
        }

        break;

}



switch (xy_pos_switch) {

    case STRAIGHT_AHEAD:
        
        //pos_control->set_max_speed_xy(_inital_vel_x);
        
        //Calculate desired x position from x velocity at autorotation initiation
        des_x = inertial_nav.get_position().x + _inital_vel_x*G_Dt;  //(cm)
        
        //Calculate desired y position from y velocity at autorotation initiation
        des_y = inertial_nav.get_position().y + _inital_vel_y*G_Dt;  //(cm)
        
        break;
        
    case INTO_WIND:
        
        
         break;
         
    case NEAREST_RALLY:
        
        
    case BREAK:
    
        //Maintain level for touch down;
        des_x = inertial_nav.get_position().x;
        des_y = inertial_nav.get_position().y;
        
        break;
        
}

//set z position target
pos_control->set_alt_target(des_z);

//set xy desired positions
//pos_control->set_xy_target(des_x,des_y);




//////////////////////////////////////////////////////////////////////
// Try using pythag to couple xy and z vel components using the measured velocities
// at initiation to determin the vel magnitude, then remove the desired z vel 
// and determine the correct propotion of x and y vel to maintain the current track.
/////////////////////////////////////////////////////




// call xy-axis position controller
//pos_control->update_xy_controller(1.0f);

// call z-axis position controller
pos_control->update_z_controller();



//saving last targeted z position for corrections in the next iteration
des_z_last = des_z;

//  These message outputs are purely for debugging purposes.  Will be removed in future.
if (message_counter == 300) {
    gcs().send_text(MAV_SEVERITY_INFO, "Desired Vel %.2f",pos_control->get_vel_target_z());
    gcs().send_text(MAV_SEVERITY_INFO, "Actual Vel %.2f",curr_vel_z);
    gcs().send_text(MAV_SEVERITY_INFO, "Target Height %.2f",pos_control->get_alt_target());
    gcs().send_text(MAV_SEVERITY_INFO, "Actual Height %.2f",curr_alt);
    gcs().send_text(MAV_SEVERITY_INFO, "Desired Height %.2f",des_z);
//    gcs().send_text(MAV_SEVERITY_INFO, "Target X %.2f",pos_control->get_pos_target().x);
//    gcs().send_text(MAV_SEVERITY_INFO, "Current X %.2f",inertial_nav.get_position().x);
//    gcs().send_text(MAV_SEVERITY_INFO, "Target X Vel %.2f",pos_control->get_desired_velocity().x);
//    gcs().send_text(MAV_SEVERITY_INFO, "Current X Vel %.2f",inertial_nav.get_velocity().x);
//    gcs().send_text(MAV_SEVERITY_INFO, "Initial X Vel %.2f",_inital_vel_x);
    
    
//    gcs().send_text(MAV_SEVERITY_INFO, "Target Y %.2f",pos_control->get_pos_target().y);
//    gcs().send_text(MAV_SEVERITY_INFO, "Current Y %.2f",inertial_nav.get_position().y);
//    gcs().send_text(MAV_SEVERITY_INFO, "Target Y Vel %.2f",pos_control->get_desired_velocity().y);
//    gcs().send_text(MAV_SEVERITY_INFO, "Current Y Vel %.2f",inertial_nav.get_velocity().y);
    gcs().send_text(MAV_SEVERITY_INFO, "--- --- --- ---");
    message_counter = 0;
}
message_counter++;









}






#endif