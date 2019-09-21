#include "AP_SpdHgtControl_Heli.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_SpdHgtControl_Heli::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: DO NOT SET MANUALLY
    // @Description: This parameter is used by the set by the autorotation controller and is based on AROT_ENABLE.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_SpdHgtControl_Heli, _param_enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: VEL_P
    // @DisplayName: Velocity (horizontal) P gain
    // @Description: Velocity (horizontal) P gain.  Determines the propotion of the target acceleration based on the velocity error.
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: VEL_I
    // @DisplayName: Velocity (horizontal) I gain
    // @Description: Velocity (horizontal) I gain.  Corrects long-term difference in desired velocity to a target acceleration
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: VEL_D
    // @DisplayName: Velocity (horizontal) D gain
    // @Description: Not used. Leave as zero.
    // @User: Advanced

    // @Param: VEL_FF
    // @DisplayName: Velocity (horizontal) feed forward
    // @Description: Velocity (horizontal) input filter.  Corrects the target acceleration proportionally to the desired velocity.
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced

    // @Param: VEL_IMAX
    // @DisplayName: Velocity (horizontal) integrator maximum
    // @Description: Velocity (horizontal) integrator maximum.  Constrains the I term contribution to acceleration target.
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cm/s/s
    // @User: Advanced

    // @Param: VEL_FLTT
    // @DisplayName: Velocity (horizontal) target filter
    // @Description: Velocity (horizontal) target filter.  Limits the maximum rate that the velocity target can be changed.
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: VEL_FLTE
    // @DisplayName: Velocity (horizontal) error filter
    // @Description: Velocity (horizontal) error filter.  Limits the maximum rate that the velocity error can be changed.
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: VEL_FLTD
    // @DisplayName: Velocity (horizontal) D filter
    // @Description: No D term in this controller.  Not used.  Leave as 0.
    // @Units: Hz
    // @User: Advanced
    AP_SUBGROUPINFO(_pid_vel, "VEL_", 1, AP_SpdHgtControl_Heli, AC_PID),
    
    AP_GROUPEND
};


// reset speed controller
void AP_SpdHgtControl_Heli::init_controller(void)
{
    _pid_vel.reset_I();
    accel_target = 0.0f;
    _cmd_vel = 100.0f * calc_speed_forward(); //(cm/s)
    _accel_out_last = _pid_vel.get_ff(_cmd_vel);
}


// set_dt - sets time delta in seconds for all controllers (i.e. 100hz = 0.01, 400hz = 0.0025)
void AP_SpdHgtControl_Heli::set_dt(float delta_sec)
{
    _dt = delta_sec;

    // update PID controller dt
    _pid_vel.set_dt(_dt);
}


// update speed controller
void AP_SpdHgtControl_Heli::update_speed_controller(void)
{
    float vel_p, vel_i, vel_ff;

    //Specify forward velocity component and determine delta velocity with respect to time
    _speed_forward = calc_speed_forward(); //(m/s)

    delta_speed_fwd = _speed_forward - _speed_forward_last; //(m/s)
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
    _pid_vel.update_all(_cmd_vel, _speed_forward*100.0f, true);

    // get p
    vel_p = _pid_vel.get_p();

    // get i
    vel_i = _pid_vel.get_i();

    // get ff
    vel_ff = _pid_vel.get_ff(_cmd_vel);

    //calculate acceleration target based on PI controller
    accel_target = vel_ff + vel_p + vel_i;

    // filter correction acceleration
    _accel_target_filter.set_cutoff_frequency(10.0f);
    _accel_target_filter.apply(accel_target, _dt);

    //Limits the maximum change in pitch attitude based on acceleration
    if (accel_target > _accel_out_last + _accel_max) {
        accel_target = _accel_out_last + _accel_max;
    } else if (accel_target < _accel_out_last - _accel_max) {
        accel_target = _accel_out_last - _accel_max;
    }

    //Limiting acceleration based on velocity gained during the previous time step 
    if (fabsf(delta_speed_fwd * 100.0f) > _accel_max * _dt) {
        _flag_limit_accel = true;
    } else {
        _flag_limit_accel = false;
    }

    float accel_out = 0.0f;
    if ((_flag_limit_accel && fabsf(accel_target) < fabsf(_accel_out_last)) || !_flag_limit_accel) {
        accel_out = accel_target;
    } else {
        accel_out = _accel_out_last;
    }
    _accel_out_last = accel_out;

    // update angle targets that will be passed to stabilize controller
    _pitch_target = atanf(-accel_out/(GRAVITY_MSS * 100.0f))*(18000.0f/M_PI);

    //Write to data flash log
    if (log_counter++ % 20 == 0) {
        AP::logger().Write("SPHT",
                           "TimeUS,SpdF,CmdV,p,i,ff,AccO,AccT,PitT",
                           "Qffffffff",
                           AP_HAL::micros64(),
                           (double)_speed_forward,
                           (double)_cmd_vel,
                           (double)vel_p,
                           (double)vel_i,
                           (double)vel_ff,
                           (double)accel_out,
                           (double)accel_target,
                           (double)_pitch_target);
    }

}


// Determine the forward ground speed component from measured components
float AP_SpdHgtControl_Heli::calc_speed_forward(void)
{
    Vector2f groundspeed_vector = _ahrs.groundspeed_vector();
    float speed_forward = groundspeed_vector.x*_ahrs.cos_yaw() + groundspeed_vector.y*_ahrs.sin_yaw(); //(m/s)

    return speed_forward;
}
