#include "AC_Autorotation.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_AHRS/AP_AHRS.h>

//Autorotation controller defaults
#define AROT_BAIL_OUT_TIME                            2.0f     // Default time for bail out controller to run (unit: s)
#define AROT_FLARE_MIN_Z_ACCEL_PEAK                   1.2f     // Minimum permissible peak acceleration factor for the flare phase (unit: -)
#define AROT_FLARE_TIME_PERIOD_MIN                    0.5f


// Head Speed (HS) controller specific default definitions
#define HS_CONTROLLER_COLLECTIVE_CUTOFF_FREQ          2.0f     // low-pass filter on accel error (unit: hz)
#define HS_CONTROLLER_HEADSPEED_P                     0.7f     // Default P gain for head speed controller (unit: -)
#define HS_CONTROLLER_ENTRY_COL_FILTER                0.35f    // Default low pass filter frequency during the entry phase (unit: Hz)
#define HS_CONTROLLER_GLIDE_COL_FILTER                0.05f    // Default low pass filter frequency during the glide phase (unit: Hz)

// Speed Height controller specific default definitions for autorotation use
#define FWD_SPD_CONTROLLER_GND_SPEED_TARGET           1100     // Default target ground speed for speed height controller (unit: cm/s)
#define FWD_SPD_CONTROLLER_MAX_ACCEL                  100      // Default acceleration limit for speed height controller (unit: cm/s/s)
#define AP_FW_VEL_P                       1.0f
#define AP_FW_VEL_I                       0.5f
#define AP_FW_VEL_D                       0.0f
#define AP_FW_VEL_IMAX                    1000.0f
#define AP_FW_VEL_FF                      0.15f
#define AP_FW_VEL_FILT_T_HZ               10.0f
#define AP_FW_VEL_FILT_E_HZ               10.0f
#define AP_FW_VEL_FILT_D_HZ               1.0f
#define AP_FW_VEL_DT                      0.0025f


const AP_Param::GroupInfo AC_Autorotation::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable settings for RSC Setpoint
    // @Description: Allows you to enable (1) or disable (0) the autonomous autorotation capability.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, AC_Autorotation, _param_enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: HS_P
    // @DisplayName: P gain for head spead controller
    // @Description: Increase value to increase sensitivity of head speed controller during autonomous autorotation.
    // @Range: 0.3 1
    // @Increment: 0.01
    // @User: Advanced
    AP_SUBGROUPINFO(_p_hs, "HS_", 2, AC_Autorotation, AC_P),

    // @Param: HS_SET_PT
    // @DisplayName: Target Head Speed
    // @Description: The target head speed in RPM during autorotation.  Start by setting to desired hover speed and tune from there as necessary.
    // @Units: RPM
    // @Range: 1000 2800
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("HS_SET_PT", 3, AC_Autorotation, _param_head_speed_set_point, 1500),

    // @Param: TARG_SP
    // @DisplayName: Target Glide Ground Speed
    // @Description: Target ground speed in cm/s for the autorotation controller to try and achieve/ maintain during the glide phase.
    // @Units: cm/s
    // @Range: 800 2000
    // @Increment: 50
    // @User: Advanced
    AP_GROUPINFO("TARG_SP", 4, AC_Autorotation, _param_target_speed, FWD_SPD_CONTROLLER_GND_SPEED_TARGET),

    // @Param: COL_FILT_E
    // @DisplayName: Entry Phase Collective Filter
    // @Description: Cut-off frequency for collective low pass filter.  For the entry phase.  Acts as a following trim.  Must be higher than AROT_COL_FILT_G.
    // @Units: Hz
    // @Range: 0.2 0.5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("COL_FILT_E", 5, AC_Autorotation, _param_col_entry_cutoff_freq, HS_CONTROLLER_ENTRY_COL_FILTER),

    // @Param: COL_FILT_G
    // @DisplayName: Glide Phase Collective Filter
    // @Description: Cut-off frequency for collective low pass filter.  For the glide phase.  Acts as a following trim.  Must be lower than AROT_COL_FILT_E.
    // @Units: Hz
    // @Range: 0.03 0.15
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("COL_FILT_G", 6, AC_Autorotation, _param_col_glide_cutoff_freq, HS_CONTROLLER_GLIDE_COL_FILTER),

    // @Param: AS_ACC_MAX
    // @DisplayName: Forward Acceleration Limit
    // @Description: Maximum forward acceleration to apply in speed controller.
    // @Units: cm/s/s
    // @Range: 50 200
    // @Increment: 10
    // @User: Advanced
    AP_GROUPINFO("AS_ACC_MAX", 7, AC_Autorotation, _param_accel_max, FWD_SPD_CONTROLLER_MAX_ACCEL),

    // @Param: BAIL_TIME
    // @DisplayName: Bail Out Timer
    // @Description: Time in seconds from bail out initiated to the exit of autorotation flight mode.
    // @Units: s
    // @Range: 0.5 4
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("BAIL_TIME", 8, AC_Autorotation, _param_bail_time, AROT_BAIL_OUT_TIME),

    // @Param: HS_SENSOR
    // @DisplayName: Main Rotor RPM Sensor 
    // @Description: Allocate the RPM sensor instance to use for measuring head speed.  RPM1 = 0.  RPM2 = 1.
    // @Units: s
    // @Range: 0.5 3
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("HS_SENSOR", 9, AC_Autorotation, _param_rpm_instance, 0),

    // @Param: FW_V_P
    // @DisplayName: Velocity (horizontal) P gain
    // @Description: Velocity (horizontal) P gain.  Determines the propotion of the target acceleration based on the velocity error.
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: FW_V_I
    // @DisplayName: Velocity (horizontal) I gain
    // @Description: Velocity (horizontal) I gain.  Corrects long-term difference in desired velocity to a target acceleration
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: FW_V_D
    // @DisplayName: Velocity (horizontal) D gain
    // @Description: Not used. Leave as zero.
    // @User: Advanced

    // @Param: FW_V_FF
    // @DisplayName: Velocity (horizontal) feed forward
    // @Description: Velocity (horizontal) input filter.  Corrects the target acceleration proportionally to the desired velocity.
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced

    // @Param: FW_V_IMAX
    // @DisplayName: Velocity (horizontal) integrator maximum
    // @Description: Velocity (horizontal) integrator maximum.  Constrains the I term contribution to acceleration target.
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cm/s/s
    // @User: Advanced

    // @Param: FW_V_FLTT
    // @DisplayName: Velocity (horizontal) target filter
    // @Description: Velocity (horizontal) target filter.  Limits the maximum rate that the velocity target can be changed.
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: FW_V_FLTE
    // @DisplayName: Velocity (horizontal) error filter
    // @Description: Velocity (horizontal) error filter.  Limits the maximum rate that the velocity error can be changed.
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: FW_V_FLTD
    // @DisplayName: Velocity (horizontal) D filter
    // @Description: No D term in this controller.  Not used.  Leave as 0.
    // @Units: Hz
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_fw_vel, "FW_V_", 10, AC_Autorotation, AC_PID),

    // @Param: TD_VEL_Z
    // @DisplayName: Desired velocity to initiate the touch down phase
    // @Description: 
    // @Units: cm/s
    // @Range: 30 200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("TD_VEL_Z", 18, AC_Autorotation, _param_vel_z_td, 50),

    // @Param: F_PERIOD
    // @DisplayName: Time period to execute the flare
    // @Description: The target time period in which the controller will attempt to complete the flare phase
    // @Units: s
    // @Range: 0.5 2.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("F_PERIOD", 19, AC_Autorotation, _param_flare_time_period, 0.9),

    // @Param: F_ACC_ZMAX
    // @DisplayName: Maximum allowable vertical acceleration during flare
    // @Description: Multiplier of acceleration due to gravity 'g'.  Cannot be smaller that 1.2.
    // @Range: 1.2 2.5
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("F_ACC_ZMAX", 20, AC_Autorotation, _param_flare_accel_z_max, 1.5),

    // @Param: TD_ALT_TARG
    // @DisplayName: Target altitude to initiate touch down phase
    // @Description: 
    // @Units: cm
    // @Range: 30 150
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("TD_ALT_TARG", 21, AC_Autorotation, _param_td_alt_targ, 50),

    // @Param: LOG
    // @DisplayName: Logging bitmask
    // @Description: 1: Glide phase tuning, 2: Flare phase tuning
    // @Range: 0 2
    // @User: Advanced
    AP_GROUPINFO("LOG", 22, AC_Autorotation, _param_log_bitmask, 0),

    // @Param: F_T_RATIO
    // @DisplayName: Time period to execute the flare
    // @Description: The ratio of the time phase that the controller will use to correct miss alignments to the target trajectories
    // @Range: 0.05 0.5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("F_T_RATIO", 23, AC_Autorotation, _param_flare_correction_ratio, 0.1),

    // @Param: COL_FILT_F
    // @DisplayName: Flare Phase Collective Filter
    // @Description: Cut-off frequency for collective low pass filter.  For the flare phase.  Acts as a following trim.
    // @Units: Hz
    // @Range: 0.2 1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("COL_FILT_F", 24, AC_Autorotation, _param_col_flare_cutoff_freq, 0.8),

    // @Param: COL_F_P
    // @DisplayName: Collective P term for flare controller
    // @Description: 
    // @Range:
    // @Increment:
    // @User: Advanced
    AP_GROUPINFO("COL_F_P", 25, AC_Autorotation, _param_flare_p, HS_CONTROLLER_ENTRY_COL_FILTER),

    // @Param: ANGLE_MAX
    // @DisplayName: Pitch Angle Limit
    // @Description: The maximum pitch angle (positive or negative) to be applied throughout the autorotation manouver.  If left at zero the 
    // @Units: cdeg
    // @Range: 1000 8000
    // @Increment: 100
    // @User: Advanced
    AP_GROUPINFO("ANGLE_MAX", 26, AC_Autorotation, _param_angle_max, 0),

    AP_GROUPEND
};

// Constructor
AC_Autorotation::AC_Autorotation(AP_InertialNav& inav) :
    _inav(inav),
    _p_hs(HS_CONTROLLER_HEADSPEED_P),
    _pid_fw_vel(AP_FW_VEL_P, AP_FW_VEL_I, AP_FW_VEL_D, AP_FW_VEL_FF, AP_FW_VEL_IMAX, AP_FW_VEL_FILT_T_HZ, AP_FW_VEL_FILT_E_HZ, AP_FW_VEL_FILT_D_HZ, AP_FW_VEL_DT)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

// Initialisation of autorotation controller
void AC_Autorotation::init_arot_controller(int16_t angle_max)
{
    // Set initial collective position to be the collective position on initialisation
    _collective_out = 0.4f;

    // Reset feed forward filter
    col_trim_lpf.reset(_collective_out);

    // Reset flags
    _flags.bad_rpm = false;

    // Reset RPM health monitoring
    _unhealthy_rpm_counter = 0;
    _healthy_rpm_counter = 0;

    // Protect against divide by zero
    _flare_correction_ratio = MAX(0.05,_param_flare_correction_ratio);
    _flare_time_period = MAX(AROT_FLARE_TIME_PERIOD_MIN,_param_flare_time_period);

    // Get angle max if param set to 0
    if (_param_angle_max == 0) {
        _angle_max = angle_max; // (cdeg)
    } else {
        _angle_max = _param_angle_max; // (cdeg)
    }
}


bool AC_Autorotation::update_hs_glide_controller(void)
{
    // Reset rpm health flag
    _flags.bad_rpm = false;
    _flags.bad_rpm_warning = false;

    // Get current rpm and update healthly signal counters
    _current_rpm = get_rpm(true);

    if (_unhealthy_rpm_counter <=30) {
        // Normalised head speed
        float head_speed_norm = _current_rpm / _param_head_speed_set_point;

        // Set collective trim low pass filter cut off frequency
        col_trim_lpf.set_cutoff_frequency(_col_cutoff_freq);

        // Calculate the head speed error.  Current rpm is normalised by the set point head speed.  
        // Target head speed is defined as a percentage of the set point.
        _head_speed_error = head_speed_norm - _target_head_speed;

        _p_term_hs = _p_hs.get_p(_head_speed_error);

        // Adjusting collective trim using feed forward (not yet been updated, so this value is the previous time steps collective position)
        _ff_term_hs = col_trim_lpf.apply(_collective_out, _dt);

        // Calculate collective position to be set
        _collective_out = _p_term_hs + _ff_term_hs;

    } else {
        // RPM sensor is bad set collective to minimum
        _collective_out = -1.0f;

        _flags.bad_rpm_warning = true;
    }

    // Send collective to setting to motors output library
    set_collective();

    return _flags.bad_rpm_warning;
}


// Function to set collective and collective filter in motor library
void AC_Autorotation::set_collective(void)
{
    AP_Motors *motors = AP::motors();
    if (motors) {
        motors->set_throttle_filter_cutoff(HS_CONTROLLER_COLLECTIVE_CUTOFF_FREQ);
        motors->set_throttle(_collective_out);
    }
}


//function that sets parameter values in flight mode
void AC_Autorotation::get_param_values(int16_t &set_point_hs, int16_t &accel, int16_t &targ_s, float &ent_freq, float &glide_freq, float &flare_freq, float &bail_time, float &flare_time, int32_t &td_alt)
{
    set_point_hs   = _param_head_speed_set_point;
    accel          = _param_accel_max;
    targ_s         = _param_target_speed;
    ent_freq       = _param_col_entry_cutoff_freq;
    glide_freq     = _param_col_glide_cutoff_freq;
    flare_freq     = _param_col_flare_cutoff_freq;
    bail_time      = _param_bail_time;
    flare_time     = _param_flare_time_period;
    td_alt         = _param_td_alt_targ;
}


//function that gets rpm and does rpm signal checking to ensure signal is reliable
//before using it in the controller
float AC_Autorotation::get_rpm(bool update_counter)
{
    // Get singleton for RPM library
    const AP_RPM *rpm = AP_RPM::get_singleton();

    float current_rpm = 0.0f;

    //Get current rpm, checking to ensure no nullptr
    if (rpm != nullptr) {
        //Check requested rpm instance to ensure either 0 or 1.  Always defaults to 0.
        if ((_param_rpm_instance > 1) || (_param_rpm_instance < 0)) {
            _param_rpm_instance = 0;
        }

        //Get RPM value
        uint8_t instance = _param_rpm_instance;
        current_rpm = rpm->get_rpm(instance);

        //Check RPM sesnor is returning a healthy status
        if (current_rpm <= -1) {
            //unhealthy, rpm unreliable
            _flags.bad_rpm = true;
        }

    } else {
        _flags.bad_rpm = true;
    }

    if (_flags.bad_rpm) {
        //count unhealthy rpm updates and reset healthy rpm counter
        _unhealthy_rpm_counter++;
        _healthy_rpm_counter = 0;

    } else if (!_flags.bad_rpm && _unhealthy_rpm_counter > 0) {
        //poor rpm reading may have cleared.  Wait 10 update cycles to clear.
        _healthy_rpm_counter++;

        if (_healthy_rpm_counter >= 10) {
            //poor rpm health has cleared, reset counters
            _unhealthy_rpm_counter = 0;
            _healthy_rpm_counter = 0;
        }
    }
    return current_rpm;
}


void AC_Autorotation::log_write_autorotation(void)
{
    // Write logs useful for tuning glide phase
    if (1<<0 & _param_log_bitmask) {
        //Write to data flash log
        AP::logger().Write("AR1G",
                       "TimeUS,P,hserr,ColOut,FFCol,CRPM,SpdF,CmdV,p,i,ff,AccO,AccT,PitT",
                         "Qfffffffffffff",
                        AP_HAL::micros64(),
                        (double)_p_term_hs,
                        (double)_head_speed_error,
                        (double)_collective_out,
                        (double)_ff_term_hs,
                        (double)_current_rpm,
                        (double)_speed_forward,
                        (double)_cmd_vel,
                        (double)_vel_p,
                        (double)_vel_i,
                        (double)_vel_ff,
                        (double)_accel_out,
                        (double)_accel_target,
                        (double)_pitch_target);
    }

    if(1<<1 & _param_log_bitmask){
        //Write to data flash log
        AP::logger().Write("AR2F",
                       "TimeUS,ZAT,AZAT,ZVT,AltT,FAT,AFAT,FP,PitOut,AcMxC,AngMax",
                         "Qffffffffff",
                        AP_HAL::micros64(),
                        (double)_flare_z_accel_targ,
                        (double)_adjusted_z_accel_target,
                        (double)_z_vel_target,
                        (double)_alt_target,
                        (double)_flare_fwd_accel_target,
                        (double)_adjusted_fwd_accel_target,
                        (double)_p_term_pitch,
                        (double)_pitch_out,
                        (double)_flare_accel_peak,
                        (double)_flare_pitch_ang_max);
    }
}


// Initialise forward speed controller
void AC_Autorotation::init_fwd_spd_controller(void)
{
    // Reset I term and acceleration target
    _pid_fw_vel.reset_I();
    _accel_target = 0.0f;
    
    // Ensure parameter acceleration doesn't exceed hard-coded limit
    _accel_max = MAX(_param_accel_max, 40.0f);

    // Reset cmd vel and last accel to sensible values
    _cmd_vel = 100.0f * calc_speed_forward(); //(cm/s)
    _accel_out_last = _pid_fw_vel.get_ff(_cmd_vel);
}


// set_dt - sets time delta in seconds for all controllers
void AC_Autorotation::set_dt(float delta_sec)
{
    _dt = delta_sec;

    // update forward speed PID controller dt
    _pid_fw_vel.set_dt(_dt);
}


// update speed controller
void AC_Autorotation::update_forward_speed_controller(void)
{
    float vel_p, vel_i, vel_ff;

    // Specify forward velocity component and determine delta velocity with respect to time
    _speed_forward = calc_speed_forward(); //(m/s)

    _delta_speed_fwd = _speed_forward - _speed_forward_last; //(m/s)
    _speed_forward_last = _speed_forward; //(m/s)

    // Limitng the target velocity based on the max acceleration limit
    if (_cmd_vel < _vel_target) {
        _cmd_vel += _accel_max * _dt;
        if (_cmd_vel > _vel_target) {
            _cmd_vel = _vel_target;
        }
    } else {
        _cmd_vel -= _accel_max * _dt;
        if (_cmd_vel < _vel_target) {
            _cmd_vel = _vel_target;
        }
    }

    // call pid controller
    _pid_fw_vel.update_all(_cmd_vel, _speed_forward*100.0f, true);

    // get p
    vel_p = _pid_fw_vel.get_p();

    // get i
    vel_i = _pid_fw_vel.get_i();

    // get ff
    vel_ff = _pid_fw_vel.get_ff(_cmd_vel);

    //calculate acceleration target based on PI controller
    _accel_target = vel_ff + vel_p + vel_i;

    // filter correction acceleration
    _accel_target_filter.set_cutoff_frequency(10.0f);
    _accel_target_filter.apply(_accel_target, _dt);

    //Limits the maximum change in pitch attitude based on acceleration
    if (_accel_target > _accel_out_last + _accel_max) {
        _accel_target = _accel_out_last + _accel_max;
    } else if (_accel_target < _accel_out_last - _accel_max) {
        _accel_target = _accel_out_last - _accel_max;
    }

    //Limiting acceleration based on velocity gained during the previous time step 
    if (fabsf(_delta_speed_fwd * 100.0f) > _accel_max * _dt) {
        _flag_limit_accel = true;
    } else {
        _flag_limit_accel = false;
    }

    float accel_out = 0.0f;
    if ((_flag_limit_accel && fabsf(_accel_target) < fabsf(_accel_out_last)) || !_flag_limit_accel) {
        accel_out = _accel_target;
    } else {
        accel_out = _accel_out_last;
    }
    _accel_out_last = accel_out;

    // update angle targets that will be passed to stabilize controller
    _pitch_target = atanf(-accel_out/(GRAVITY_MSS * 100.0f))*(18000.0f/M_PI);

}


// Determine the forward ground speed component from measured components
float AC_Autorotation::calc_speed_forward(void)
{
    auto &ahrs = AP::ahrs();
    Vector2f groundspeed_vector = ahrs.groundspeed_vector();
    float speed_forward = groundspeed_vector.x*ahrs.cos_yaw() + groundspeed_vector.y*ahrs.sin_yaw(); //(m/s)
    return speed_forward;
}


// Determine whether or not the flare phase should be initiated
bool AC_Autorotation::should_flare(void)
{
    // Determine peak acceleration if the flare was initiated in this state (cm/s/s)
    _flare_accel_z_peak = 2.0f * (-_param_vel_z_td - _inav.get_velocity().z) / _flare_time_period;
    _flare_accel_fwd_peak = 2.0f * (0.0f - calc_speed_forward() * 100.0f) / _flare_time_period;  // Assumed touch down forward speed of 0 m/s

    // Resolve the magnitude of the total peak acceleration
    _flare_accel_peak = sqrtf(_flare_accel_z_peak * _flare_accel_z_peak + _flare_accel_fwd_peak * _flare_accel_fwd_peak);

    // Compare the calculated peak acceleration to the allowable limits
    if ((_flare_accel_peak < (AROT_FLARE_MIN_Z_ACCEL_PEAK-1) * GRAVITY_MSS * 100.0f)  || (_flare_accel_peak > (_param_flare_accel_z_max-1) * GRAVITY_MSS * 100.0f)){
        gcs().send_text(MAV_SEVERITY_INFO, "Magnitude Fail");
        return false;
    }

    // Compute the maximum pitch angle
    _flare_pitch_ang_max = atanf(_flare_accel_fwd_peak/_flare_accel_z_peak)*(18000.0f/M_PI);  //(cdeg)

    // Compare the calculated max angle limit to the parameter defined limit
    if (fabsf(_flare_pitch_ang_max) > fabsf(_angle_max)) {
        gcs().send_text(MAV_SEVERITY_INFO, "Angle Fail");
        return false;
    }

    // Determine the altitude that the flare would complete
    uint32_t td_alt_predicted = 0.237334852f * _flare_accel_z_peak * _flare_time_period * _flare_time_period  +  _inav.get_velocity().z * _flare_time_period  +  _inav.get_position().z;

    // Compare the prediced altitude to the acceptable range
    if ((td_alt_predicted < _param_td_alt_targ * 0.5f)  ||  (td_alt_predicted > _param_td_alt_targ * 1.5f)){
        return false;
    }

    return true;
}


//set initial conditions for flare targets
void AC_Autorotation::set_flare_initial_cond(void)
{
    _vel_z_initial = _inav.get_velocity().z;
    _vel_fwd_initial = calc_speed_forward() * 100;
    _last_vel_z = _vel_z_initial;
    _last_vel_fwd = _vel_fwd_initial;
    _alt_z_initial = _inav.get_position().z;
    _pitch_out = _pitch_target;  //TODO: move flare controller to just continue using pitch target. Dont create pitch_out variable.
}


/* Use altitude, velocity, and acceleration to come up with a collective position.  Acceleration
can be deamed to have a relationship to collective position.  Use the new collective position to 
feed into head speed controller.  The head speed controller can be used to reverse calculate what 
the target head speed should be.  Compare the head speed target calced from acceleration to the ideal
head speed trajectory to determine an error between desired for kinematics and desired for head speed 
trajectory.  Use energy to weight the coparative difference using a ratio and that will give the new 
target.  That target can then be fed through the actual head speed controller to generate the final output.
This should creat a system whereby the realtive components can be compared using physical relasionships
and won't require PID tuning beyond the original head speed controller.

alt error ==> converted and added to velocity target ==> velocity error ==> converted and added to acceleration 
target ==> acceleration error converted to collective position (linear prediction based on hover col position 
and current col position?)  ==>  Collective position fed through reverse HS controller  ==>  target hs basied on 
accel  ==>  compare that to target head speed from ideal trajectory  ==>  Create energy weighted average target HS
==>  use target HS in HS to collective output controller. */

float AC_Autorotation::update_flare_controller(void)
{
    // Calculate the target altitude trajectory
    _alt_target = calc_position_target(_flare_accel_z_peak, _vel_z_initial, _alt_z_initial);

    // Calculate the target velocity trajectories
    _z_vel_target = calc_velocity_target(_flare_accel_z_peak, _vel_z_initial, _alt_target, _inav.get_position().z);
    _fwd_vel_target = 0;//calc_velocity_target(_flare_accel_fwd_peak, _vel_fwd_initial);

    // Calculate the target acceleration trajectories
    _adjusted_z_accel_target = calc_acceleration_target(_flare_z_accel_targ, _flare_accel_z_peak, _z_vel_target, _inav.get_velocity().z);
    _adjusted_fwd_accel_target = calc_acceleration_target(_flare_fwd_accel_target, _flare_accel_fwd_peak, _fwd_vel_target, calc_speed_forward() * 100);

    // Approximate current acceleration
    float z_accel_measured = (_inav.get_velocity().z - _last_vel_z)/_dt;
    float fwd_accel_measured = (calc_speed_forward() * 100 - _last_vel_fwd)/_dt;

    // Store velocity
    _last_vel_z = _inav.get_velocity().z;
    _last_vel_fwd = calc_speed_forward() * 100;

    // Calculate target acceleration magnitude
    float flare_accel_mag_target = sqrtf(_adjusted_z_accel_target * _adjusted_z_accel_target + _adjusted_fwd_accel_target * _adjusted_fwd_accel_target);

    // Calculate the measured acceleration magnitude
    float flare_accel_mag_measured = sqrtf(z_accel_measured * z_accel_measured + fwd_accel_measured * fwd_accel_measured);

    // Calculate the p term, based on magnitude error
    _p_term_pitch = (flare_accel_mag_target - flare_accel_mag_measured) * _param_flare_p;

    // Compute the pitch angle feed forward
    //int16_t pitch_ang_ff = atanf(_fwd_accel_target/_adjusted_z_accel_target)*(18000.0f/M_PI);

    // Calculate collective position to be set
    _pitch_out = _p_term_pitch;

    _pitch_out = constrain_int16(_pitch_out,-_angle_max,_angle_max);

    //gcs().send_text(MAV_SEVERITY_INFO, "_p_term_pitch = %.3f",_p_term_pitch);
    //gcs().send_text(MAV_SEVERITY_INFO, "pitch_ang_ff = %.3i",pitch_ang_ff);
    //gcs().send_text(MAV_SEVERITY_INFO, "_pitch_out = %.3i",_pitch_out);

    return _pitch_out;
}


int32_t AC_Autorotation::calc_position_target(float accel_peak, int16_t vel_initial, int32_t pos_initial)
{
    int32_t pos_target = accel_peak / 4.0f * (_flare_time * _flare_time - _flare_time_period * _flare_time_period / (M_PI * M_2PI) * sinf((_flare_time * M_2PI)/_flare_time_period)  -  _flare_time_period * _flare_time_period / M_2PI)  +  vel_initial * _flare_time  +  pos_initial;
    return pos_target;
}


// Overloaded function: Determine the velocity target without altitude correction
int16_t AC_Autorotation::calc_velocity_target(float accel_peak, int16_t vel_initial)
{
    // Calculate the target velocity trajectory
    int16_t vel_target = accel_peak / 2.0f * (_flare_time - _flare_time_period * sinf(_flare_time * M_2PI / _flare_time_period) / M_2PI)  +  vel_initial;
    return vel_target;
}


// Overloaded function: Determine the velocity target with altitude correction
int16_t AC_Autorotation::calc_velocity_target(float accel_peak, int16_t vel_initial, int32_t pos_target, int32_t pos_measured)
{
    // Calculate the target velocity trajectory
    int16_t vel_target = accel_peak / 2.0f * (_flare_time - _flare_time_period * sinf(_flare_time * M_2PI / _flare_time_period) / M_2PI)  +  vel_initial;

    // Calculate velocity correction based on altitude error
    int16_t vel_correction = (pos_target - pos_measured) / (_flare_correction_ratio * _flare_time_period);

    // Adjust velocity target
    int16_t adjusted_vel_target = vel_target + vel_correction;
    return adjusted_vel_target;
}


// Determine the acceleration target and correct target to compensate for velocity error
float AC_Autorotation::calc_acceleration_target(float &accel_target, float accel_peak, int16_t vel_target, int16_t vel_measured)
{
    // Calculate desired acceleration
    accel_target = accel_peak * (1 - cosf((_flare_time * M_2PI)/_flare_time_period)) / 2.0f;

    // Calculate acceleration correction based on velocity error
    float accel_correction = (vel_target - vel_measured) / (_flare_correction_ratio * _flare_time_period);

    // Adjust acceleration target
    float adjusted_accel_target = accel_target + accel_correction;
    return adjusted_accel_target;
}

