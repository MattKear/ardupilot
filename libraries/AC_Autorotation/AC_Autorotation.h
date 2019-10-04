#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_Motors/AP_Motors.h>
#include <AP_Motors/AP_MotorsHeli_RSC.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h>
#include <AP_InertialNav/AP_InertialNav.h>
#include <Filter/Filter.h>
#include <Filter/LowPassFilter.h>
#include <AC_PID/AC_P.h>

//Autorotation controller defaults
#define AROT_BAIL_OUT_TIME                            2.0f     // Default time for bail out controller to run (unit: s)
#define HEAD_SPEED_TARGET_RATIO                       1.0f     // Normalised target main rotor head speed (unit: -)

// Head Speed (HS) controller specific default definitions
#define HS_CONTROLLER_COLLECTIVE_CUTOFF_FREQ          2.0f     // low-pass filter on accel error (unit: hz)
#define HS_CONTROLLER_HEADSPEED_P                     0.7f     // Default P gain for head speed controller (unit: -)
#define HS_CONTROLLER_ENTRY_COL_FLITER                0.35f    // Default low pass filter frequency during the entry phase (unit: Hz)
#define HS_CONTROLLER_GLIDE_COL_FLITER                0.05f    // Default low pass filter frequency during the glide phase (unit: Hz)

// Speed Height controller specific default definitions for autorotation use
#define SPD_HGT_CONTROLLER_GND_SPEED_TARGET           1100.0f  // Default target ground speed for speed height controller (unit: cm/s)
#define SPD_HGT_CONTROLLER_MAX_ACCEL                  100.0f   // Default acceleration limit for speed height controller (unit: cm/s/s)

class AC_Autorotation
{
public:

    // Constructor
 AC_Autorotation(AP_Motors& motors);
//AC_Autorotation(AP_Motors& motors,

//    _motors(motors),
//    _attitude_control(attitude_control),
//    _p_hs(HS_CONTROLLER_HEADSPEED_P)
//    {
//        AP_Param::setup_object_defaults(this, var_info);
//    }

    //--------Functions--------
    void init(void);
    void init_hs_controller(void);  //Initialise head speed controller
    bool update_hs_glide_controller(float dt);  //Update head speed controller
    float get_rpm(void) { return _current_rpm; }  //Function just returns the rpm as last read in this library
    float get_rpm(bool update_counter);  //Function fetches fresh rpm update and continues sensor health monitoring
    void set_target_head_speed(float ths) { _target_head_speed = ths; }  //Sets the normalised target head speed
    void set_col_cutoff_freq(float freq) { _col_cutoff_freq = freq; }  //Sets the collective low pass filter cut off frequency
    void set_param_values(int16_t* set_point_hs, int16_t* accel, int16_t* targ_s, float* td_alt, float* ent_freq, float* glide_freq, float* bail_time);  //Enables the parameter values to be retrieved by the autorotation flight mode
    float get_last_collective() { return _collective_out; }
    bool is_enable(void) { return _param_enable; }

    // User Settable Parameters
    static const struct AP_Param::GroupInfo var_info[];


protected:

    //--------Internal Variables--------
    float _current_rpm;
    float _collective_out = 0.0f;
    float _head_speed_error;         //Error between target head speed and current head speed.  Normalised by head speed set point RPM.
    uint16_t _log_counter = 0;
    float _col_cutoff_freq;          //Lowpass filter cutoff frequency (Hz) for collective
    uint8_t _unhealthy_rpm_counter;  //Counter used to track RPM sensor unhealthy signal
    uint8_t _healthy_rpm_counter;    //Counter used to track RPM sensor healthy signal
    float _target_head_speed;        //Normalised target head speed.  Normalised by head speed set point RPM.
    float _p_term_hs;                //Proportional contribution to collective setting
    float _ff_term_hs;               //Following trim feed forward contribution to collective setting

    //--------Parameter Values--------
    AC_P _p_hs;
    AP_Int8  _param_enable;
    AP_Float _param_hs_p;
    AP_Int16 _param_head_speed_set_point;
    AP_Int16 _param_accel_max;
    AP_Int16 _param_target_speed;
    AP_Float _param_col_entry_cutoff_freq;
    AP_Float _param_col_glide_cutoff_freq;
    AP_Float _param_bail_time;
    AP_Int8  _param_rpm_instance;

    AP_Float _param_td_alt;  //currently no parameter.  Will be used in future work.

    //--------Internal Flags--------
    struct controller_flags {
            bool bad_rpm             : 1;
            bool bad_rpm_warning     : 1;
    } _flags;

    //--------Internal Functions--------
    void update_logger(void);

    // low pass filter for collective trim
    LowPassFilterFloat col_trim_lpf;

    //--------References to Other Libraries--------
    AP_Motors&                  _motors;
};