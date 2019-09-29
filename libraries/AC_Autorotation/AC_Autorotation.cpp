#include "AC_Autorotation.h"

const AP_Param::GroupInfo AC_Autorotation::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable settings for RSC Setpoint
    // @Description: Allows you to enable (1) or disable (0) the autonomous autorotation capability.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 0, AC_Autorotation, _param_enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: HS_P
    // @DisplayName: P gain for head spead controller
    // @Description: Increase value to increase sensitivity of head speed controller during autonomous autorotation.
    // @Range: 0.3 1
    // @Increment: 0.01
    // @User: Advanced
    AP_SUBGROUPINFO(_p_hs, "HS_", 1, AC_Autorotation, AC_P),

    // @Param: HS_SET_PT
    // @DisplayName: Target Head Speed
    // @Description: The target head speed in RPM during autorotation.  Start by setting to desired hover speed and tune from there as neccassary.
    // @Units: RPM
    // @Range: 1000 2800
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("HS_SET_PT", 2, AC_Autorotation, _param_head_speed_set_point, 1500.0f),

    // @Param: TARG_SP
    // @DisplayName: Target Glide Ground Speed
    // @Description: Target ground speed in cm/s for the autorotation controller to try and achieve/ maintain during the glide phase.
    // @Units: cm/s
    // @Range: 800 2000
    // @Increment: 50
    // @User: Advanced
    AP_GROUPINFO("TARG_SP", 3, AC_Autorotation, _param_target_speed, SPD_HGT_CONTROLLER_GND_SPEED_TARGET),

    // @Param: COL_FILT_E
    // @DisplayName: Entry Phase Collective Filter
    // @Description: Cut-off frequency for collective low pass filter.  For the entry phase.  Acts as a following trim.  Must be higher than COL_FILT_G.
    // @Units: Hz
    // @Range: 0.2 0.5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("COL_FILT_E", 4, AC_Autorotation, _param_col_entry_cutoff_freq, HS_CONTROLLER_ENTRY_COL_FLITER),

    // @Param: COL_FILT_G
    // @DisplayName: Glide Phase Collective Filter
    // @Description: Cut-off frequency for collective low pass filter.  For the glide phase.  Acts as a following trim.  Must be lower than COL_FILT_E.
    // @Units: Hz
    // @Range: 0.03 0.15
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("COL_FILT_G", 5, AC_Autorotation, _param_col_glide_cutoff_freq, HS_CONTROLLER_GLIDE_COL_FLITER),

    // @Param: AS_ACC_MAX
    // @DisplayName: Forward Acceleration Limit
    // @Description: Maximum forward acceleration to apply in speed controller.
    // @Units: cm/s/s
    // @Range: 50 200
    // @Increment: 10
    // @User: Advanced
    AP_GROUPINFO("AS_ACC_MAX", 6, AC_Autorotation, _param_accel_max, SPD_HGT_CONTROLLER_MAX_ACCEL),

    // @Param: BAIL_TIME
    // @DisplayName: Bail Out Timer
    // @Description: Time in seconds from bail out initiated to the exit of autorotation flight mode.
    // @Units: s
    // @Range: 0.5 4
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("BAIL_TIME", 7, AC_Autorotation, _param_bail_time, AROT_BAIL_OUT_TIME),

    // @Param: HS_SENSOR
    // @DisplayName: Main Rotor RPM Sensor 
    // @Description: Allocate the RPM sensor instance to use for measuring head speed.  RPM1 = 0.  RPM2 = 1.
    // @Units: s
    // @Range: 0.5 3
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("HS_SENSOR", 8, AC_Autorotation, _param_rpm_instance, 0),

    AP_GROUPEND
};


//initialisation of head speed controller
void AC_Autorotation::init_hs_controller()
{
    //set initial collective position to be the collective position on initialisation
    _collective_out = 0.4f;//_motors.get_throttle();

    //Reset feed forward filter
    col_trim_lpf.reset(_collective_out);

    //reset flags
    _flags.bad_rpm = false;

    //reset RPM health monitoring
    _unhealthy_rpm_counter = 0;
    _healthy_rpm_counter = 0;
}


bool AC_Autorotation::update_hs_glide_controller(float dt)
{
    //reset rpm health flag
    _flags.bad_rpm = false;
    _flags.bad_rpm_warning = false;

    //get current rpm and update healthly signal counters
    _current_rpm = get_rpm(true);

    if (_unhealthy_rpm_counter <=30) {
        //Normalised head speed
        float head_speed_norm = _current_rpm / _param_head_speed_set_point;

        //set collective trim low pass filter cut off frequency
        col_trim_lpf.set_cutoff_frequency(_col_cutoff_freq);

        //Calculate the head speed error
        //Current rpm is normalised by the set point head speed.  Target head speed is defined as a percentage of the set point.
        _head_speed_error = head_speed_norm - _target_head_speed;

        _p_term_hs = _p_hs.get_p(_head_speed_error);

        //Adjusting collective trim using feed forward
        _ff_term_hs = col_trim_lpf.apply(_collective_out, dt); //note that the collective out has not yet been updated, so this value is the previous time steps collective position

        //Calculate collective position to be set
        _collective_out = _p_term_hs + _ff_term_hs;

    } else {
        //RPM sensor is bad set collective to minimum
        _collective_out = -1.0f;

        _flags.bad_rpm_warning = true;
    }

    //update data flash logs
    update_logger();

    // send collective to setting to motors output library
    set_collective(HS_CONTROLLER_COLLECTIVE_CUTOFF_FREQ);

    return _flags.bad_rpm_warning;
}


//function to set collective and collective filter in motor library
void AC_Autorotation::set_collective(float collective_filter_cutoff)
{
    _motors.set_throttle_filter_cutoff(collective_filter_cutoff);
    _motors.set_throttle(_collective_out);
}


//function that sets parameter values in flight mode
void AC_Autorotation::set_param_values(int16_t* set_point_hs, int16_t* accel, int16_t* targ_s, float* td_alt, float* ent_freq, float* glide_freq, float* bail_time)
{
    *set_point_hs   = _param_head_speed_set_point;
    *accel          = _param_accel_max;
    *targ_s         = _param_target_speed;
    *td_alt         = _param_td_alt;
    *ent_freq       = _param_col_entry_cutoff_freq;
    *glide_freq     = _param_col_glide_cutoff_freq;
    *bail_time      = _param_bail_time;
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
        if (_param_rpm_instance > 1) {
            _param_rpm_instance = 0;
        } else if (_param_rpm_instance < 0) {
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


void AC_Autorotation::update_logger(void)
{
    //Write to data flash log
    if (_log_counter++ % 20 == 0) {
        AP::logger().Write("AROT",
                           "TimeUS,P,hserr,ColOut,FFCol,CRPM",
                           "Qfffff",
                           AP_HAL::micros64(),
                           (double)_p_term_hs,
                           (double)_head_speed_error,
                           (double)_collective_out,
                           (double)_ff_term_hs,
                           (double)_current_rpm);
    }
}