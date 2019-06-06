#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AC_PID/AC_P.h>               // P library
#include <AC_PID/AC_PID.h>             // PID library
#include <AC_PID/AC_PI_2D.h>           // PI library (2-axis)
#include <AC_PID/AC_PID_2D.h>          // PID library (2-axis)
#include <AP_RPM/AP_RPM.h>
#include <AP_Motors/AP_Motors.h>          // motors library
#include <AC_AttitudeControl/AC_AttitudeControl.h> // Attitude control library
#include <AP_InertialNav/AP_InertialNav.h>     // Inertial Navigation library


// Head Speed (HS) controller default definitions
#define HS_CONTROLLER_COLLECTIVE_CUTOFF_FREQ          2.0f    // low-pass filter on accel error (unit: hz)
#define COLLECTIVE_SLEW_RATE_ENTRY                    2.0f    // The time in seconds that it will take to initially set the collective during entry phase to match reversing of inflow direction (unit: s)
#define HS_CONTROLLER_HEADSPEED_P                     0.6f    // Defualt P gain for head speed controller (unit: -)
#define HS_CONTROLLER_HEADSPEED_I                     0.0015f // Defualt D gain for head speed controller (unit: -)
#define HS_CONTROLLER_HEADSPEED_D                     0.1f    // Defualt D gain for head speed controller (unit: -)
#define HS_CONTROLLER_HEADSPEED_I_LIM                 0.2f    // Defualt D gain for head speed controller (unit: -)
#define HS_CONTROLLER_HEADSPEED_TARGET                0.9f    // Defualt D gain for head speed controller (unit: -)
#define HS_CONTROLLER_ENTRY_SLEW_TIME                 2.0f    // Defualt D gain for head speed controller (unit: -)






class AC_AutorotationCtrl
{
public:

    /// Constructor
AC_AutorotationCtrl(const AP_AHRS_View& ahrs,
                     const AP_InertialNav& inav,
                     AP_Motors& motors,
                     AC_AttitudeControl& attitude_control) :
    _ahrs(ahrs),
    _inav(inav),
    _motors(motors),
    _attitude_control(attitude_control)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }



    // Functions
    void update_hs_glide_controller(float dt);  //Update head speed controller
    void update_att_glide_controller(float dt);  //Update attitude controller
    void set_collective(float _collective_filter_cutoff);


    float get_data() { return _data; }
    float get_rpm() { return _current_rpm; }
    float get_rpm_error() { return _head_speed_error; }

    //To be called once when initiating the entry.  Calling multiple times will keep resetting the the slew rate timer.
    void use_entry_slew() { _flags.use_entry_slew_rate = 1;  _entry_slew_rate = _param_recovery_slew; }
    void set_attitude_hs_mixing_flag (bool flag_switch) { _flags.use_attitude_hs_mixing = flag_switch; }
    void reset_I_terms() {_error_integral = 0.0f;}

    float get_p() { return _param_hs_p; }

    float get_d() { return _param_hs_d; }

    float get_td_alt() { return _param_td_alt; }

    float get_td_agression() { return _param_td_col_agression; }

    float get_speed_target(void) { return _param_target_airspeed; }

    float get_accel_max(void) { return _param_accel_max; }


    // User settable parameters
    static const struct AP_Param::GroupInfo var_info[];


protected:

    //----------------------------
    //---- internal variables ----
    //----------------------------
    //Head Speed / Collective Controller
    float _current_rpm;
    float _collective_out;
    float _entry_slew_rate;  //(s) Number of seconds to apply collective setting over
    float _head_speed_error;  // (rad/s) error between target head speed and current head speed
    float _error_integral;
    float _last_head_speed_error;
    
    //Head Speed / Attitude Controller
    float _airspeed_error;
    
    float _data;
    
    
    //internal flags
    struct rpm_controller_flags {
            bool use_attitude_hs_mixing     : 1;    // 1 if attitude/collective mixing should be used to control head speed
            bool use_entry_slew_rate        : 1;    // 1 if the phase of flight requires a gradual slew from one collective position to another
    } _flags;
    
    // Parameter values
    AP_Float _param_hs_p;               //p value for rpm collective controller
    AP_Float _param_hs_d;               //d value for rpm collective controller
    AP_Float _param_hs_i;               //i value for rpm collective controller
    AP_Float _param_hs_i_lim;           //i limit value for rpm collective controller
    AP_Float _param_target_head_speed;  //(rpm) normalised head speed to be maintined in the autorotation (normalised by hover head speed)
    AP_Float _param_head_speed_hover;   //(rpm) head speed during normal hover
    AP_Float _param_recovery_slew;      //(s) the time period over which the PD errors are phased in and the I terms are switched off
    AP_Float _param_hs_as_att_p;
    AP_Float _param_hs_as_att_i;
    AP_Float _param_hs_as_att_i_lim;
    AP_Float _param_accel_max;
    AP_Float _param_target_airspeed;
    AP_Float _param_td_alt;
    AP_Float _param_td_col_agression;
    
    
    
    
    // References to other libraries
    const AP_AHRS_View&         _ahrs;
    const AP_InertialNav&       _inav;
    AP_Motors&                  _motors;
    AC_AttitudeControl&         _attitude_control;
    
    
    
    
};