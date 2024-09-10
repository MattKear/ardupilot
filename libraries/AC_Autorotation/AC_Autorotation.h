#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_Motors.h>
#include <AP_Motors/AP_MotorsHeli_RSC.h>
#include <Filter/Filter.h>
#include <Filter/LowPassFilter.h>
#include <AC_PID/AC_P.h>
#include <AP_InertialNav/AP_InertialNav.h>

class AC_Autorotation
{
public:

    //Constructor
    AC_Autorotation(AP_InertialNav& inav, AP_AHRS& ahrs, AP_MotorsHeli*& motors);

    void init(void);

    bool enabled(void) const { return _param_enable.get() > 0; }

    // Init and run entry phase controller
    void init_entry(void);
    void run_entry(float& pitch_target);

    // Init and run the glide phase controller
    void init_glide(void);
    void run_glide(float& pitch_target);

    // Update controller used to drive head speed with collective
    void update_headspeed_controller(void);

    // Update controller used to control speed via vehicle pitch
    void update_forward_speed_controller(float& pitch_target);  // Update forward speed controller

    // Arming checks for autorotation, mostly checking for miss-configurations
    bool arming_checks(size_t buflen, char *buffer) const;

    // Logging of autorotation specific variables
    void log_write_autorotation(void) const;
    void log_reason(uint8_t reason) { _landed_reason = reason; }

    // Helper functions
    float accel_max(void) { return MAX(_param_accel_max.get(), 0.1); }
    void set_dt(float delta_sec) { _dt = delta_sec; }

    // Helper to get measured head speed that has been normalised by head speed set point
    float get_norm_head_speed(void);

    // Calculates the forward ground speed in the horizontal plane
    float get_speed_forward(void);

    // User Settable Parameters
    static const struct AP_Param::GroupInfo var_info[];

    static const uint32_t entry_time_ms = 2000; // (ms) Number of milliseconds that the entry phase operates for

private:

    float _hs_decay;                 // The head speed target acceleration during the entry phase
    float _dt;                       // Time step.

    uint8_t _landed_reason;          // Bitmask of the reasons we think we have landed. Stored in lib for logging.

    //--------- Not Checked vars
    float _collective_out;
    float _head_speed_error;         // Error between target head speed and current head speed. Normalised by head speed set point RPM.

    float _target_head_speed;        // Normalised target head speed.  Normalised by head speed set point RPM.
    float _p_term_hs;                // Proportional contribution to collective setting.
    float _ff_term_hs;               // Following trim feed forward contribution to collective setting.

    float _vel_target;               // Forward velocity target.
    float _speed_forward_last;       // The forward speed calculated in the previous cycle.

    bool _flag_limit_accel;          // Maximum acceleration limit reached flag.
    float _accel_out_last;           // Acceleration value used to calculate pitch target in previous cycle.
    float _cmd_vel;                  // Command velocity, used to get PID values for acceleration calculation.
    float _accel_target;             // Acceleration target, calculated from PID.

    float _vel_p;                    // Forward velocity P term.
    float _vel_ff;                   // Forward velocity Feed Forward term.
    float _accel_out;                // Acceleration value used to calculate pitch target.

    LowPassFilterFloat _accel_target_filter; // acceleration target filter

    //--------Parameter Values--------
    AP_Int8  _param_enable;
    AC_P _p_hs;
    AC_P _p_fw_vel;
    AP_Float _param_head_speed_set_point;
    AP_Float _param_target_speed;
    AP_Float _param_col_entry_cutoff_freq;
    AP_Float _param_col_glide_cutoff_freq;
    AP_Float _param_accel_max;
    AP_Int8  _param_rpm_instance;
    AP_Float _param_fwd_k_ff;

    // low pass filter for collective trim
    LowPassFilterFloat col_trim_lpf;

    //--------References to Other Libraries--------
    AP_InertialNav&    _inav;
    AP_AHRS&           _ahrs;
    AP_MotorsHeli*&    _motors_heli;
};
