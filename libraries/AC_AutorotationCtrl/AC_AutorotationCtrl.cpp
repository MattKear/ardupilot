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
    
    // @Param: ENT_SLEW
    // @DisplayName: The slew time for entry into headspeed/collective controller
    // @Description: This allows a gentle transition into the contoller.  Specifiy the time to phase in the headspeed/collective controller PID.
    // @Units: s
    // @Range: 1-4
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("ENT_SLEW", 6, AC_AutorotationCtrl, _param_recovery_slew, HS_CONTROLLER_ENTRY_SLEW_TIME),
    
    // @Param: ATT_P
    // @DisplayName: P gain for head spead/airspeed attitude controller
    // @Description: ---
    // @Units: -
    // @Range: -
    // @Increment: -
    // @User: Advanced
    AP_GROUPINFO("ATT_P", 7, AC_AutorotationCtrl, _param_hs_as_att_p, 2.0f),
    
    // @Param: ATT_I
    // @DisplayName: I gain for head spead/airspeed attitude controller
    // @Description: ---
    // @Units: -
    // @Range: -
    // @Increment: -
    // @User: Advanced
    AP_GROUPINFO("ATT_I", 8, AC_AutorotationCtrl, _param_hs_as_att_i, 2.0f),
    
    // @Param: ATT_I_LIM
    // @DisplayName: I Limit for head spead/airspeed attitude controller
    // @Description: ---
    // @Units: -
    // @Range: -
    // @Increment: -
    // @User: Advanced
    AP_GROUPINFO("ATT_I_LIM", 9, AC_AutorotationCtrl, _param_hs_as_att_i_lim, 2.0f),
    
    // @Param: ACC_MAX
    // @DisplayName: D gain for head spead/airspeed attitude controller
    // @Description: ---
    // @Units: -
    // @Range: -
    // @Increment: -
    // @User: Advanced
    AP_GROUPINFO("ACC_MAX", 10, AC_AutorotationCtrl, _param_accel_max, 2.0f),
    
    // @Param: TARG_AS
    // @DisplayName: Target airspeed in cm/s for the autorotation controller to try and achieve/ maintain.
    // @Description: ---
    // @Units: cm/s
    // @Range: -
    // @Increment: -
    // @User: Advanced
    AP_GROUPINFO("TARG_AS", 11, AC_AutorotationCtrl, _param_target_airspeed, 2.0f),
    
    // @Param: TD_ALT
    // @DisplayName: Altitude at which to begin touch down phase
    // @Description: ---
    // @Units: cm
    // @Range: -
    // @Increment: -
    // @User: Advanced
    AP_GROUPINFO("TD_ALT", 12, AC_AutorotationCtrl, _param_td_alt, 2.0f),
    
    // @Param: TD_COL_AGR
    // @DisplayName: Collective agression during touch down phase
    // @Description: ---
    // @Units: cm
    // @Range: -
    // @Increment: -
    // @User: Advanced
    AP_GROUPINFO("TD_COL_AGR", 13, AC_AutorotationCtrl, _param_td_col_agression, 2.0f),
    
    AP_GROUPEND
};


void AC_AutorotationCtrl::update_hs_glide_controller(float dt)
{
    // Get singleton for RPM library
    const AP_RPM *rpm = AP_RPM::get_instance();

    //Get current rpm, checking to ensure no nullptr
    if (rpm != nullptr) {
        _current_rpm = rpm->get_rpm(0);
    }

    //Use slew rate scaling for entry into phase
    if (_flags.use_entry_slew_rate) {
        _entry_slew_rate -= dt;

        if (_entry_slew_rate <= 0) {
            //switch of entry slew
            _flags.use_entry_slew_rate = 0;
        }
    }

    // ---- gain scaler calculation ----
    float entry_gain_scaler = 1;  // By default no entry slew gain scaler to be used

    if (_flags.use_entry_slew_rate && _entry_slew_rate > 0) {
        // Calculate entry slew gain scaler
        entry_gain_scaler = (_param_recovery_slew - _entry_slew_rate) / _param_recovery_slew;
    }


    //Prevent divide by zero error
    if (_param_head_speed_hover < 500) {
        _param_head_speed_hover = 500;  //Making sure that hover rpm is not unreasonably low
    }

    //Calculate the head speed error
    //Current rpm is normalised by the hover head speed.  Target head speed is defined as a percentage of hover speed
    _head_speed_error = (_current_rpm / _param_head_speed_hover) - _param_target_head_speed;

    float P_hs = _head_speed_error * _param_hs_p * entry_gain_scaler;

    //No I term to be used in entry to controller
    float I_hs = 0;

    if (!_flags.use_entry_slew_rate) {
        //calculate integral of error
        _error_integral += _head_speed_error;

        //apply I gain
         I_hs = _error_integral * _param_hs_i;
    }

    //check that I is within limits
    if (I_hs < -_param_hs_i_lim) {
        I_hs = -_param_hs_i_lim;
    } else if (I_hs > _param_hs_i_lim) {
        I_hs = _param_hs_i_lim;
    }


    //calculate head speed error differential
    float head_speed_error_differential = (_head_speed_error - _last_head_speed_error) / dt;

    float D_hs = head_speed_error_differential * _param_hs_d * entry_gain_scaler;

    //Calculate collective position to be set
    _collective_out = (P_hs + I_hs + D_hs) + _motors.get_throttle_hover();

    // send collective to setting to motors output library
    set_collective(HS_CONTROLLER_COLLECTIVE_CUTOFF_FREQ);

    //save last head speed error term for differential calculation in next time step
    _last_head_speed_error = _head_speed_error;

}




void AC_AutorotationCtrl::set_collective(float collective_filter_cutoff)
{
// TODO: setup slew rate of collective application

    _motors.set_throttle_filter_cutoff(collective_filter_cutoff);
    _motors.set_throttle(_collective_out);

}






//void AC_AutorotationCtrl::update_att_glide_controller(float dt)
//{

  //  float aspeed;
  //  if (_ahrs.airspeed_estimate(&aspeed)){
  //  _airspeed_error = aspeed - _param_target_airspeed;
  //  } else {
  //  _airspeed_error = sqrtf(_inav.get_velocity().x * _inav.get_velocity().x  +  _inav.get_velocity().y * _inav.get_velocity().y  +  _inav.get_velocity().z * _inav.get_velocity().z);
  //  }

  //  float P_as = _airspeed_error * _param_hs_as_att_p;

  //  _data = P_as;

  //  float target_roll = 0;
  //  float target_pitch = P_as;
  //  float target_yaw_rate = 0;

  //  _attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);


//}





