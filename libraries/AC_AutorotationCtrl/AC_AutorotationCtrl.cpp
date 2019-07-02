#include "AC_AutorotationCtrl.h"


const AP_Param::GroupInfo AC_AutorotationCtrl::var_info[] = {

    // @Param: HS_P
    // @DisplayName: P gain for head spead controller
    // @Description: Increase value to increase sensitivity of head speed controller during autonomous autorotation.
    // @Units: -
    // @Range: 0.3 1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("HS_P", 0, AC_AutorotationCtrl, _param_hs_p, HS_CONTROLLER_HEADSPEED_P),

    // @Param: HS_D
    // @DisplayName: D gain for head spead controller
    // @Description: Increase value to increase damping of head speed controller during autonomous autorotation.
    // @Units: -
    // @Range: 0.001 0.02
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("HS_D", 1, AC_AutorotationCtrl, _param_hs_d, HS_CONTROLLER_HEADSPEED_D),

    // @Param: HS_I
    // @DisplayName: I gain for head spead controller
    // @Description: Increase value to increase damping of head speed controller during autonomous autorotation.
    // @Units: -
    // @Range: 0.001 0.02
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("HS_I", 2, AC_AutorotationCtrl, _param_hs_i, HS_CONTROLLER_HEADSPEED_I),

    // @Param: HS_I_LIM
    // @DisplayName: I gain for head spead controller
    // @Description: Increase value to increase damping of head speed controller during autonomous autorotation.
    // @Units: -
    // @Range: 0.001 0.02
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("HS_I_LIM", 3, AC_AutorotationCtrl, _param_hs_i_lim, HS_CONTROLLER_HEADSPEED_I_LIM),

    // @Param: HS_TARGET
    // @DisplayName: Target head speed for controller to achieve
    // @Description: Specify the target head speed as a percentage of hover headspeed
    // @Units: -
    // @Range: 0.8-1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("HS_TARGET", 4, AC_AutorotationCtrl, _param_target_head_speed, HS_CONTROLLER_HEADSPEED_TARGET),

    // @Param: HS_HOVER
    // @DisplayName: Temporary parameter for head speed in the hover
    // @Description: This will be replaced by hover collective learning in the future
    // @Units: -
    // @Range: -
    // @Increment: -
    // @User: Advanced
    AP_GROUPINFO("HS_HOVER", 5, AC_AutorotationCtrl, _param_head_speed_hover, 1700.0f),

    // @Param: TARG_AS
    // @DisplayName: Target airspeed in cm/s for the autorotation controller to try and achieve/ maintain.
    // @Description: ---
    // @Units: cm/s
    // @Range: -
    // @Increment: -
    // @User: Advanced
    AP_GROUPINFO("TARG_AS", 6, AC_AutorotationCtrl, _param_target_airspeed, 2.0f),

    // @Param: TD_ALT
    // @DisplayName: Altitude at which to begin touch down phase
    // @Description: ---
    // @Units: cm
    // @Range: -
    // @Increment: -
    // @User: Advanced
    AP_GROUPINFO("TD_ALT", 7, AC_AutorotationCtrl, _param_td_alt, 6.0f),

    // @Param: COL_FILT_E
    // @DisplayName: Cut-off frequency for collective feed forward during entry
    // @Description: ---
    // @Units: cm
    // @Range: -
    // @Increment: -
    // @User: Advanced
    AP_GROUPINFO("COL_FILT_E", 8, AC_AutorotationCtrl, _param_col_entry_cutoff_freq, 0.08f),
    
    // @Param: AS_ACC_MAX
    // @DisplayName: Maximum acceleration to apply in airspeed controller
    // @Description: ---
    // @Units: cm
    // @Range: -
    // @Increment: -
    // @User: Advanced
    AP_GROUPINFO("AS_ACC_MAX", 9, AC_AutorotationCtrl, _param_accel_max, 100.0f),

    // @Param: COL_FILT_G
    // @DisplayName: Cut-off frequency for collective feed forward during glide
    // @Description: ---
    // @Units: cm
    // @Range: -
    // @Increment: -
    // @User: Advanced
    AP_GROUPINFO("COL_FILT_G", 10, AC_AutorotationCtrl, _param_col_glide_cutoff_freq, 0.05f),

    AP_GROUPEND
};



//initialisation of head speed controller
void AC_AutorotationCtrl::init_hs_controller()
{
    //reset 'last' terms
    _last_head_speed_norm = 0.0f;
    _last_head_speed_error = 0.0f;

    //set target head speed to be guided on entry
    _flags.target_guide = true;

    //set 3 seconds on entry timer
    _entry_time_remain = 3.0f;

    //set initial collective position to be the collective position on initialisation
    _collective_out = 0.4f;//_motors.get_throttle();

    //Reset feed forward filter
    col_trim_lpf.reset(_collective_out);
    
    // Get singleton for RPM library
    const AP_RPM *rpm = AP_RPM::get_instance();

    //Get current rpm, checking to ensure no nullptr
    if (rpm != nullptr) {
        _current_rpm = rpm->get_rpm(0);
    }

    //The decay rate to reduce the head speed from the current to the target
    _hs_decay = ((_current_rpm/_param_head_speed_hover) - _param_target_head_speed) /2000.0f;

    //Set temporary target to current head speed for entry phase
    _target_head_speed = _current_rpm/_param_head_speed_hover;
}


void AC_AutorotationCtrl::update_hs_glide_controller(float dt)
{
    // Get singleton for RPM library
    const AP_RPM *rpm = AP_RPM::get_instance();

    //Get current rpm, checking to ensure no nullptr
    if (rpm != nullptr) {
        _current_rpm = rpm->get_rpm(0);
    }

    //Normalised head speed
    float head_speed_norm = _current_rpm / _param_head_speed_hover;


    if (_flags.entry_phase) {
         
         //Guide head speed to glide target using a gradual change of target as the speed natuarally decays.
         //This prevents the collective from raising on initiation
        if (head_speed_norm >= _param_target_head_speed && head_speed_norm >= _last_head_speed_norm && _flags.target_guide) {
            //head speed has increased, gently reduce head speed target based on 2 second decay rate
            _target_head_speed -= _hs_decay*dt;

        } else if (head_speed_norm >= _param_target_head_speed && head_speed_norm < _last_head_speed_norm && _flags.target_guide) {
            //head speed has decreased.  Predict target collective position based on sustained deceleration.
            _target_head_speed -= (_last_head_speed_norm - head_speed_norm);

        } else if (_flags.target_guide){
            // the target head speed has dropped below target, maintain target head speed and complete entry.
            _target_head_speed = _param_target_head_speed;
            _flags.target_guide = false;
        }

        //set collective trim low pass filter cut off frequency
        col_trim_lpf.set_cutoff_frequency(_param_col_entry_cutoff_freq);

        //update entry phase timer
        _entry_time_remain -= dt;

        //if entry phase time is up, then set entry flag to false
        if (_entry_time_remain <= 0.0f) {
            _flags.entry_phase = false;
        }


    } else {
        _target_head_speed = _param_target_head_speed;

        //set collective trim low pass filter cut off frequency
        col_trim_lpf.set_cutoff_frequency(_param_col_glide_cutoff_freq);


    }
    _last_head_speed_norm = head_speed_norm;


    //Prevent divide by zero error
    if (_param_head_speed_hover < 500) {
        _param_head_speed_hover = 500;  //Making sure that hover rpm is not unreasonably low
    }

    //Calculate the head speed error
    //Current rpm is normalised by the hover head speed.  Target head speed is defined as a percentage of hover speed
    _head_speed_error = head_speed_norm - _target_head_speed;

    float P_hs = _head_speed_error * _param_hs_p;

    //calculate head speed error differential
    float head_speed_error_differential = (_head_speed_error - _last_head_speed_error) / dt;

    float D_hs = head_speed_error_differential * _param_hs_d;


    //Adjusting collective trim using feed forward
    float FF_hs = col_trim_lpf.apply(_collective_out, dt); //note that the collective out has not yet been updated, so this value is the previous time steps collective position
    

    //Calculate collective position to be set
    _collective_out = P_hs + D_hs + FF_hs;

    // send collective to setting to motors output library
    set_collective(HS_CONTROLLER_COLLECTIVE_CUTOFF_FREQ);

    //save last head speed error term for differential calculation in next time step
    _last_head_speed_error = _head_speed_error;



    //Write to data flash log
    if (_log_counter++ % 20 == 0) {
        DataFlash_Class::instance()->Log_Write("ARO2", "TimeUS,P,D,hserr,hstarg,ColOut,FFCol", "Qffffff",
                                                AP_HAL::micros64(),
                                               (double)P_hs,
                                               (double)D_hs,
                                               (double)_head_speed_error,
                                               (double)_target_head_speed,
                                               (double)_collective_out,
                                               (double)FF_hs);
    }






}


//function to set collective and collective filter in motor library
void AC_AutorotationCtrl::set_collective(float collective_filter_cutoff)
{

    _motors.set_throttle_filter_cutoff(collective_filter_cutoff);
    _motors.set_throttle(_collective_out);

}




