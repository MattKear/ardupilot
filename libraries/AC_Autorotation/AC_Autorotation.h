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
#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>


class AC_Autorotation
{
public:

    //Constructor
    AC_Autorotation(AP_InertialNav& inav);

    //--------Functions--------
    void init_hs_controller(void);  // Initialise autorotation controller
    void init_fwd_spd_controller(void);  // Initialise forward speed controller
    bool update_hs_glide_controller(void);  // Update head speed controller
    float get_rpm(void) const { return _current_rpm; }  // Function just returns the rpm as last read in this library
    float get_rpm(bool update_counter);  // Function fetches fresh rpm update and continues sensor health monitoring
    void set_target_head_speed(float ths) { _target_head_speed = ths; }  // Sets the normalised target head speed
    void set_col_cutoff_freq(float freq) { _col_cutoff_freq = freq; }  // Sets the collective low pass filter cut off frequency
    int16_t get_hs_set_point(void) { return _param_head_speed_set_point; }
    float get_col_entry_freq(void) { return _param_col_entry_cutoff_freq; }
    float get_col_glide_freq(void) { return _param_col_glide_cutoff_freq; }
    float get_col_flare_freq(void) { return _param_col_flare_cutoff_freq; }
    float get_bail_time(void) { return _param_bail_time; }
    float get_last_collective() const { return _collective_out; }
    float get_flare_time_period() const { return _param_flare_time_period; }
    int16_t get_td_alt_targ() const { return _param_td_alt_targ; }
    int16_t get_td_vel_targ() const { return _param_vel_z_td; }
    bool is_enable(void) { return _param_enable; }
    void log_write_autorotation(void);
    void update_forward_speed_controller(void);  // Update foward speed controller
    void set_desired_fwd_speed(void) { _vel_target = _param_target_speed; } // Overloaded: Set desired speed for forward controller to parameter value
    void set_desired_fwd_speed(float speed) { _vel_target = speed; } // Overloaded: Set desired speed to argument value
    int32_t get_pitch(void) const { return _pitch_target; }  // Get pitch target
    float calc_speed_forward(void);  // Calculates the forward speed in the horizontal plane
    void set_dt(float delta_sec);
    bool should_flare(void);  // Function to determine whether or not the flare phase should be initiated
    float update_flare_controller(void);
    void set_flare_time(float ft) { _flare_time = ft/1000.0f; }  // Set flare time and convert from millis to seconds
    void set_flare_initial_cond(void);

    // User Settable Parameters
    static const struct AP_Param::GroupInfo var_info[];

private:

    //--------Internal Variables--------
    float _current_rpm;
    float _collective_out;
    float _head_speed_error;         // Error between target head speed and current head speed.  Normalised by head speed set point RPM.
    uint16_t _log_counter;
    float _col_cutoff_freq;          // Lowpass filter cutoff frequency (Hz) for collective.
    uint8_t _unhealthy_rpm_counter;  // Counter used to track RPM sensor unhealthy signal.
    uint8_t _healthy_rpm_counter;    // Counter used to track RPM sensor healthy signal.
    float _target_head_speed;        // Normalised target head speed.  Normalised by head speed set point RPM.
    float _p_term_hs;                // Proportional contribution to collective setting.
    float _ff_term_hs;               // Following trim feed forward contribution to collective setting.

    float _vel_target;               // Forward velocity target.
    float _pitch_target;             // Pitch angle target.
    float _vel_error;                // Velocity error.
    float _accel_max;                // Maximum acceleration limit.
    int16_t _speed_forward_last;       // The forward speed calculated in the previous cycle.
    bool _flag_limit_accel;          // Maximum acceleration limit reached flag.
    float _accel_out_last;           // Acceleration value used to calculate pitch target in previous cycle.
    float _cmd_vel;                  // Command velocity, used to get PID values for acceleration calculation.
    float _accel_target;             // Acceleration target, calculated from PID.
    float _delta_speed_fwd;          // Change in forward speed between computation cycles.
    float _dt;                       // Time step.
    int16_t _speed_forward;            // Measured forward speed.
    float _vel_p;                    // Forward velocity P term.
    float _vel_ff;                   // Forward velocity Feed Forward term.
    float _accel_out;                // Acceleration value used to calculate pitch target.
    float _flare_time;               // Flare time, used for computing target trajectories.
    float _flare_accel_z_peak;
    float _flare_accel_fwd_peak;
    float _flare_accel_peak;         // Calculated peak acceleration for target trajectory.
    int16_t _flare_pitch_ang_max;      // Maximum pitch angle expected in flare phase.
    int16_t _last_vel_z;
    int16_t _last_vel_fwd;
    int16_t _vel_z_initial;
    int16_t _vel_fwd_initial;
    int32_t _alt_z_initial;
    float _adjusted_z_accel_target;
    float _adjusted_fwd_accel_target;
    int16_t _z_vel_target;
    int16_t _fwd_vel_target;
    int32_t _alt_target;
    int16_t _angle_max;
    float _flare_fwd_accel_target;
    float _flare_z_accel_targ;

    float _p_term_pitch;
    int16_t _pitch_out;
    float _flare_time_period;
    float _flare_correction_ratio;

    //temp variables
    int8_t logger_count;

    LowPassFilterFloat _accel_target_filter; // acceleration target filter

    //--------Parameter Values--------
    AP_Int8  _param_enable;
    AC_P _p_hs;
    AC_P _p_fw_vel;
    AP_Int16 _param_head_speed_set_point;
    AP_Int16 _param_target_speed;
    AP_Float _param_col_entry_cutoff_freq;
    AP_Float _param_col_glide_cutoff_freq;
    AP_Int16 _param_accel_max;
    AP_Float _param_bail_time;
    AP_Int8  _param_rpm_instance;
    AP_Float _param_fwd_k_ff;
    AP_Int16 _param_vel_z_td;
    AP_Float _param_flare_time_period;
    AP_Float _param_flare_accel_z_max;
    AP_Int16 _param_td_alt_targ;
    AP_Int8 _param_log_bitmask;
    AP_Float _param_flare_correction_ratio;
    AP_Float _param_col_flare_cutoff_freq;
    AP_Float _param_flare_p;
    AP_Int16 _param_angle_max;

    //--------Internal Flags--------
    struct controller_flags {
            bool bad_rpm             : 1;
            bool bad_rpm_warning     : 1;
    } _flags;

    //--------Internal Functions--------
    void set_collective(void);
    int32_t calc_position_target(float accel_peak, int16_t vel_initial, int32_t pos_initial);
    int16_t calc_velocity_target(float accel_peak, int16_t vel_initial);  // Overloaded function: Determine the velocity target without altitude correction
    int16_t calc_velocity_target(float accel_peak, int16_t vel_initial, int32_t pos_target, int32_t pos_measured);  // Overloaded function: Determine the velocity target with altitude correction
    float calc_acceleration_target(float &accel_target, float accel_peak, int16_t vel_target, int16_t vel_measured);

    // low pass filter for collective trim
    LowPassFilterFloat col_trim_lpf;

    //--------References to Other Libraries--------
    AP_InertialNav&    _inav;

};
