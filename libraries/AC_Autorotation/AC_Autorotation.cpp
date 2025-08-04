#include "AC_Autorotation.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_RangeFinder/AP_RangeFinder.h>

extern const AP_HAL::HAL& hal;

// Autorotation controller defaults
#define HEAD_SPEED_TARGET_RATIO  1.0    // Normalised target main rotor head speed
#define AP_ALPHA_TPP             20.0  // (deg) Maximum angle of the Tip Path Plane

const AP_Param::GroupInfo AC_Autorotation::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable settings for RSC Setpoint
    // @Description: Allows you to enable (1) or disable (0) the autonomous autorotation capability.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, AC_Autorotation, _param_enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: HS_P
    // @DisplayName: Head speed controller proportional gain
    // @Description: Proportional gain for head speed controller.
    // @Range: 0.3 1 TODO
    // @Increment: 0.01
    // @User: Standard

    // @Param: HS_I
    // @DisplayName: Head speed controller integrator gain
    // @Description: Integrator gain for head speed controller.
    // @Range: 0.3 1 TODO
    // @Increment: 0.01
    // @User: Standard

    // @Param: HS_IMAX
    // @DisplayName: Head speed controller max integrator magnitude
    // @Description: Integrator magnitude constraint for head speed controller.
    // @Range: 0.3 1 TODO
    // @Increment: 0.01
    // @User: Standard
    AP_SUBGROUPINFO(_hs_ctrl, "HS_", 2, AC_Autorotation, AC_PI),

    // @Param: HS_SET_PT
    // @DisplayName: Target Head Speed
    // @Description: The target head speed in RPM during autorotation. Start by setting to desired hover speed and tune from there as necessary.
    // @Units: RPM
    // @Range: 1000 2800
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("HS_SET_PT", 3, AC_Autorotation, _param_head_speed_set_point, 1500),

    // @Param: FWD_SP_TARG
    // @DisplayName: Target Glide Body Frame Forward Speed
    // @Description: Target ground speed in cm/s for the autorotation controller to try and achieve/ maintain during the glide phase.
    // @Units: m/s
    // @Range: 8 20
    // @Increment: 0.5
    // @User: Standard
    AP_GROUPINFO("FWD_SP_TARG", 4, AC_Autorotation, _param_target_speed, 11),

    // @Param: XY_ACC_MAX
    // @DisplayName: Body Frame XY Acceleration Limit
    // @Description: Maximum body frame acceleration allowed in the in speed controller. This limit defines a circular constraint in accel. Minimum used is 0.5 m/s/s.
    // @Units: m/s/s
    // @Range: 0.5 8.0
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("XY_ACC_MAX", 7, AC_Autorotation, _param_accel_max, 5.0),

    // @Param: HS_SENSOR
    // @DisplayName: Main Rotor RPM Sensor
    // @Description: Allocate the RPM sensor instance to use for measuring head speed.
    // @Values: 0:RPM1,1:RPM2,2:RPM3,3:RPM4
    // @User: Standard
    AP_GROUPINFO("HS_SENSOR", 8, AC_Autorotation, _param_rpm_instance, 0),

    // @Param: ROT_SOL
    // @DisplayName: rotor solidity
    // @Description: helicopter specific main rotor solidity
    // @Range: 0.01 0.1
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("ROT_SOL", 10, AC_Autorotation, _param_solidity, 0.05),

    // @Param: ROT_DIAM
    // @DisplayName: rotor diameter
    // @Description: helicopter specific main rotor diameter
    // @Units: m
    // @Range: 0.001 0.01
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("ROT_DIAM", 11, AC_Autorotation, _param_diameter, 1.25),

    // @Param: TD_TIME
    // @DisplayName: Touchdown Time
    // @Description: Desired time for the touchdown phase to last. Using the measured vertical velocity, this parameter is used to calculate the height that the vehicle will transition from the flare to the touchdown phase. Minimum value used is 0.3 s.
    // @Units: s
    // @Range: 2.0 8.0
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("TD_TIME", 12, AC_Autorotation, _param_touchdown_time, 3.0),

    // @Param: FLR_MIN_HGT
    // @DisplayName: Minimum Flare Height
    // @Description: A safety cutoff feature to ensure that the calculated flare height cannot go below this value. This is the absolute minimum height that the flare will initiate. Ensure that this is appropriate for your vehicle.
    // @Units: m
    // @Range: 10 30
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("FLR_MIN_HGT", 13, AC_Autorotation, _flare_hgt.min_height, 6),

    // @Param: TD_MIN_HGT
    // @DisplayName: Minimum Touchdown Height
    // @Description: A safety cutoff feature to ensure that the calculated touchdown height cannot go below this value. This is the absolute minimum height that the touchdown will initiate. Touchdown height must be less than flare height. Ensure that this is appropriate for your vehicle.
    // @Units: m
    // @Range: 0.5 10
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TD_MIN_HGT", 14, AC_Autorotation, _touchdown_hgt.min_height, 0.5),

    // @Param: FLR_MAX_HGT
    // @DisplayName: Maximum Flare Height
    // @Description: A safety cutoff feature to ensure that the calculated flare height cannot go above this value. This is the absolute maximum height above ground that the flare will initiate. Ensure that this is appropriate for your vehicle.
    // @Units: m
    // @Range: 10 30
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("FLR_MAX_HGT", 17, AC_Autorotation, _flare_hgt.max_height, 30),

    // @Param: NAV_MODE
    // @DisplayName: Autorotation Navigation Mode
    // @Description: Select the navigation mode to use when in the autonomous autorotation.
    // @Values: 0:Pilot lateral accel control,1:Turn into wind,2:Maintain velocity vector with cross track.
    // @User: Standard
    AP_GROUPINFO("NAV_MODE", 18, AC_Autorotation, _param_nav_mode, 0),

    // @Param: TD_JERK_MAX
    // @DisplayName: Touchdown Max Jerk
    // @Description: The peak jerk to be applied in the scurve trajectory calculated for the touchdown phase.
    // @Units: m/s/s/s
    // @Range: 10 100
    // @Increment: 1.0
    // @User: Standard
    AP_GROUPINFO("TD_JERK_MAX", 19, AC_Autorotation, _param_td_jerk_max, 15),

    // @Param: HEIGHT_FILT
    // @DisplayName: Surface distance filter frequency
    // @Description: Surface distance filter frequency
    // @Unit: Hz
    // @Range: 1 20
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("HEIGHT_FILT", 20, AC_Autorotation, _height_filt_hz, 20),

    // @Param: COL_TRIM
    // @DisplayName: Collective Trim Angle to Achieve Target Head Speed
    // @Description: The collective trim angle that will achieve the target head speed. This value is used in the entry phases and in the event that the RPM sensor fails.
    // @Unit: Deg
    // @Range: -12 -4
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("COL_TRIM", 21, AC_Autorotation, col_angle_trim, -6),

    // @Param: TD_ACC_MAX
    // @DisplayName: Touchdown Acceleration Limit
    // @Description: The maximum positive acceleration that the z-position controller can request in the touchdown phase
    // @Unit: m/s/s
    // @Range: 1 20
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("TD_ACC_MAX", 22, AC_Autorotation, _td_accel_max, 5),

    // @Param: ENT_COL_RAT
    // @DisplayName: Hover Entry phase collective rate
    // @Description: The rate at which the collective will move to zero thrust position in the hover entry phase
    // @Unit: deg/s
    // @Range: 1 5
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("ENT_COL_RAT", 23, AC_Autorotation, _entry_col_rate_deg, 3.0),

    // @Param: AS_DUAL
    // @DisplayName: Enable Asynchronous Dual Rotor Autorotation
    // @Description: This enables additional autorotation functionality to support the autorotation dual helis that do not have a gearbox/drive mechanism between the two rotor heads.  This is not needed if the two rotors of your dual heli are mechanically linked together.
    // @Values: 0:Disable,1:Enable
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("AS_DUAL", 25, AC_Autorotation, _dual_enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: HS_SENSOR2
    // @DisplayName: Second Rotor RPM Sensor 
    // @Description: Allocate the RPM sensor instance to use for measuring head speed of the 2nd swashplate on the dual heli.  On a tandem this will be the rear head.
    // @Values: 0:RPM1,1:RPM2,2:RPM3,3:RPM4
    // @User: Standard
    AP_GROUPINFO("HS_SENSOR2", 26, AC_Autorotation, _param_rpm2_instance, 0),

    // @Param: HS_PTCH_LIM
    // @DisplayName: Headspeed Error to Constrain Pitch Demands
    // @Description: Defines the minimum ratio of target head speed, beyond which the pitch controller will be constrained to prevent the pitch controller from slowing one head any further. If either head gets to a measured speed of less than AROT_HS_PTCH_LIM x AROT_HS_SET_PT then the speed controller will constrain the pitch demand until both head speeds return to speeds above the ratio defined by AROT_HS_PTCH_LIM.
    // @Range: 0.7 1
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("HS_PTCH_LIM", 27, AC_Autorotation, _safe_head_speed_ratio, 0.8),

    AP_GROUPEND
};

// Constructor
AC_Autorotation::AC_Autorotation(AP_MotorsHeli*& motors, AC_AttitudeControl*& att_crtl, AC_PosControl*& pos_ctrl, AP_Int16& land_speed_cm) :
    _motors_heli(motors),
    _attitude_control(att_crtl),
    _pos_control(pos_ctrl),
    _land_speed_cm(land_speed_cm)
    {
#if AP_RANGEFINDER_ENABLED
        // ensure the AP_SurfaceDistance object is enabled
        _ground_surface.enabled = true;
#endif
        AP_Param::setup_object_defaults(this, var_info);
    }

void AC_Autorotation::init(void)
{
    const AP_AHRS &ahrs = AP::ahrs();

    // Calculate the direction vector for use in CROSS_TRACK navigation control
    Vector2f ground_speed_NE = ahrs.groundspeed_vector();

    // TODO add a constrain so that the track vector must be within +/- 90 of heading (heli going backwards aint pretty)

    if (ground_speed_NE.length() > MIN_MANOEUVERING_SPEED) {
        // We are moving with sufficient speed that the vehicle is moving "with purpose" and the
        // ground velocity vector can be used to define the track vector
        _track_vector = ground_speed_NE.normalized();

    } else {
        // We are better off using the vehicle's current heading to define the track vector.
        Vector2f unit_vec = {1.0, 0.0};
        _track_vector = ahrs.body_to_earth2D(unit_vec);
    }

    // Initialisation of head speed controller
    // Set initial collective position to be the current collective position for smooth init
    const float collective_out = _motors_heli->get_throttle();
    // Set head speed controller integrator
    // Note that the headspeed controller is inverted (positive error needs to lead to reducing collective)
    // so we set the I term with negative collective and invert the controller output again before sending it to motors
    _hs_ctrl.set_integrator(-collective_out);

    // Reset the lagged velocity filter. We use this so that we can estimate if we are in steady conditions in the flare height calc.
    _lagged_vel_z.reset(get_ef_velocity_up());

    // Set limits and initialise NE pos controller
    _pos_control->set_max_speed_accel_NE_cm(_param_target_speed.get()*100.0, _param_accel_max.get()*100.0);
    _pos_control->set_correction_speed_accel_NE_cm(_param_target_speed.get()*100.0, _param_accel_max.get()*100.0);
    _pos_control->set_pos_error_max_NE_cm(2000);
    _pos_control->init_NE_controller();

    // Reset the landed reason
    _landed_reason.min_speed = false;
    _landed_reason.land_col = false;
    _landed_reason.is_still = false;

    // Reset the guarded height measurements to ensure that they init to the min value
    _flare_hgt.reset();
    _touchdown_hgt.reset();

    // set the guarded heights
    _touchdown_hgt.max_height = _flare_hgt.min_height;

    // Reset flags for limiting pitch controller based on headspeed
    head_speed_pitch_limit = false;

    // Update the ground surface filtered alt cutoff_frequency
    _ground_surface.alt_cm_filt.set_cutoff_frequency(_height_filt_hz);

    // Calc initial estimate of what height we think we should flare at
    initial_flare_hgt_estimate();
}

// Functions and config that are only to be done once at the beginning of the entry
void AC_Autorotation::init_entry(void)
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AROT: Entry Phase");

    // Target head speed is set to rpm at initiation to prevent steps in controller
    if (!get_mean_norm_headspeed(_target_head_speed)) {
        // Cannot get a valid RPM sensor reading so we default to not slewing the head speed target
        _target_head_speed = HEAD_SPEED_TARGET_RATIO;
    }

    // The rate to move the head speed from the current measurement to the target
    _hs_accel = (HEAD_SPEED_TARGET_RATIO - _target_head_speed) / (float(entry_time_ms)*1e-3);

    // Set speed target to maintain the current speed whilst we enter the autorotation
    _desired_vel = _param_target_speed.get();

    // When entering the autorotation we can have some roll target applied depending on what condition that
    // the position controller initialised on. If we are in the TURN_INTO_WIND or CROSS_TRACK navigation mode
    // we want to prevent changes in heading initially as the roll target may turn the aircraft in the wrong direction
    _heading_hold = Nav_Mode(_param_nav_mode.get()) == Nav_Mode::TURN_INTO_WIND ||
                    Nav_Mode(_param_nav_mode.get()) == Nav_Mode::CROSS_TRACK;
}

// The entry controller just a special case of the glide controller with head speed target slewing
void AC_Autorotation::run_entry(float pilot_accel_norm)
{
    // Slew the head speed target from the initial condition to the target head speed ratio for the glide
    const float max_change = _hs_accel * _dt;
    _target_head_speed = constrain_float(HEAD_SPEED_TARGET_RATIO, _target_head_speed - max_change, _target_head_speed + max_change);

    run_glide(pilot_accel_norm);
}

// Functions and config that are only to be done once at the beginning of the glide
void AC_Autorotation::init_glide(void)
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AROT: Glide Phase");

    // Ensure target head speed is set to setpoint, in case it didn't reach the target during entry
    _target_head_speed = HEAD_SPEED_TARGET_RATIO;

    // Ensure desired forward speed target is set to param value
    _desired_vel = _param_target_speed.get();

    // unlock any heading hold if we had one
    _heading_hold = false;
}

// Maintain head speed and forward speed as we glide to the ground
void AC_Autorotation::run_glide(float des_lat_accel_norm)
{
    check_headspeed_limits();

    update_headspeed_controller();

    update_navigation_controller(des_lat_accel_norm);

    // Keep flare altitude estimate up to date so state machine can decide when to flare
    update_flare_hgt();
}

// Functions and config that are only to be done once at the beginning of the flare
void AC_Autorotation::init_flare(void)
{
    gcs().send_text(MAV_SEVERITY_INFO, "AROT: Flare Phase");

    // Ensure target head speed, we may have skipped the glide phase if we did not have time to complete the
    // entry phase before hitting the flare height
    _target_head_speed = HEAD_SPEED_TARGET_RATIO;

    // We cannot limit the speed target in the flare, the manoeuver will aid in generating head speed anyway
    head_speed_pitch_limit = false;

    if(_pos_control->is_active_NE()) {
        // Init to position control targets if it has been running
        Vector3f vel_target_NEU = _pos_control->get_vel_target_NEU_ms();
        _flare_entry_fwd_speed = AP::ahrs().earth_to_body(vel_target_NEU).x;
        gcs().send_text(MAV_SEVERITY_INFO, "Flare Target Path");
    } else {
        // Incase we have jumped to the flare phase because we are low
        _flare_entry_fwd_speed = get_bf_speed_forward();
    }


    // Lock heading for the flare if the navigation mode uses roll components in the calculations
    _heading_hold = Nav_Mode(_param_nav_mode.get()) == Nav_Mode::TURN_INTO_WIND ||
                    Nav_Mode(_param_nav_mode.get()) == Nav_Mode::CROSS_TRACK;
}

void AC_Autorotation::run_flare(float des_lat_accel_norm)
{
    check_headspeed_limits();

    // Update head speed/ collective controller
    update_headspeed_controller();

    // During the flare we want to linearly slow the aircraft to a stop as we
    // reach the touchdown alt for the start of the touchdown phase
    _desired_vel = linear_interpolate(0.0f, _flare_entry_fwd_speed, _hagl, _touchdown_hgt.get(), _flare_hgt.get());

    // Run forward speed controller
    update_navigation_controller(des_lat_accel_norm);
}

// Functions and config that are only to be done once at the beginning of the hover auto entry
void AC_Autorotation::init_hover_entry()
{
    gcs().send_text(MAV_SEVERITY_INFO, "Hover Entry Phase");

    // set pitch target to current state for smooth transfer
    _desired_vel = get_bf_speed_forward();

    // Ensure no heading hold
    _heading_hold = false;
}


// Controller phase where the aircraft is too low and too slow to perform a full autorotation
// instead, we will try to minimize rotor drag until we can jump to the touchdown phase
void AC_Autorotation::run_hover_entry(float des_lat_accel_norm)
{
    // Move collective to min drag position at parameter specified rate
    float col_angle_deg_out = _motors_heli->get_current_col_angle_deg() - _entry_col_rate_deg.get() * _dt;
    // constrain collective to zero thrust angle (min rotor drag)
    col_angle_deg_out = MAX(col_angle_deg_out, _motors_heli->get_zero_thrust_angle_deg());

    // convert desired collective angle to collective output (0 to 1)
    float collective_out = _motors_heli->get_coll_from_ang_deg(col_angle_deg_out);

    // Do not move collective if we are below the min touch down height
    if (_hagl <= _touchdown_hgt.min_height.get()) {
        collective_out = _motors_heli->get_throttle();
    }

    // Send collective setting to motors output library
    _attitude_control->set_throttle_out(collective_out, false, COLLECTIVE_CUTOFF_FREQ_HZ);

    // Smoothly decay desired forward speed to zero over the entry time
    _desired_vel *= 1 - (_dt / (float(entry_time_ms) * 1e-3));

    update_navigation_controller(des_lat_accel_norm);
}


void AC_Autorotation::init_touchdown(void)
{
    gcs().send_text(MAV_SEVERITY_INFO, "AROT: Touchdown Phase");

    // Ensure we are not limiting the speed target in this phase, incase we jumped to it from a low height entry
    head_speed_pitch_limit = false;

    // Set vertical speed and acceleration limits
    // TODO: make some of these constraints parameter values
    _pos_control->set_max_speed_accel_U_cm(-30.0, 5.0, get_td_accel_max());
    _pos_control->set_correction_speed_accel_U_cmss(-30.0, 5.0, get_td_accel_max());

    // Initialise the vertical position controller
    _pos_control->init_U_controller();

    // Use the same measurements as the position controller as the initial conditions for the generated trajectory
    _td_init_time = AP_HAL::millis();
    _td_init_az = _pos_control->get_measured_accel_U_mss();
    _td_init_vz = _pos_control->get_vel_estimate_NEU_ms().z;
    _td_init_pos = _pos_control->get_pos_desired_NEU_m().z;

    // unlock any heading hold if we had one
    _heading_hold = false;
}


void AC_Autorotation::run_touchdown(float des_lat_accel_norm)
{

    const float td_time = float(AP_HAL::millis() - _td_init_time) * 1e-3;

    // Calc the desired javp targets for our calculated trajectory
    float j, a, v, p;
    update_trajectory(td_time, _td_init_az, _td_init_vz, _td_init_pos, _tj1, _tj2, _tj3, j, a, v, p);

    // Check that we are not saturated on collective output
    const bool collective_limit = _motors_heli->limit.throttle_lower || _motors_heli->limit.throttle_upper;

    // Set velocity target in Z position controller
    _pos_control->input_pos_vel_accel_U_m(p, v, a, collective_limit);

    // Run the vertical position controller and set output collective
    _pos_control->update_U_controller();

    // Update XY targets and controller
    // We don't know exactly at what point we transitioned into the touchdown phase, so we need to 
    // keep driving the desired XY speed to zero. This will help with getting the vehicle level for touchdown
    _desired_vel *= 1 - (_dt / get_touchdown_time());

    update_navigation_controller(des_lat_accel_norm);

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: ARTD
    // @Vehicles: Copter
    // @Description: Helicopter autorotation touch down trajectory information
    // @Field: TimeUS: Time since system startup
    // @Field: J: Touchdown trajectory jerk, positive up, m/s/s/s
    // @Field: A: Touchdown trajectory acceleration, positive up, m/s/s
    // @Field: V: Touchdown trajectory velocity, positive up, m/s/
    // @Field: P: Touchdown trajectory position, positive up, m

    // Write to data flash log
    AP::logger().WriteStreaming("ARTD",
                       "TimeUS,J,A,V,P",
                        "Qffff",
                        AP_HAL::micros64(),
                        j,
                        a,
                        v,
                        p);
#endif
}


void AC_Autorotation::update_headspeed_controller(void)
{
    // Get current rpm
    float head_speed_norm = 0;
    const bool measurement_valid = get_mean_norm_headspeed(head_speed_norm);

    float collective_out;
    if (measurement_valid) {
        // Update PI controller
        const bool collective_limit = _motors_heli->limit.throttle_lower || _motors_heli->limit.throttle_upper;
        collective_out = _hs_ctrl.update(head_speed_norm, _target_head_speed, _dt, collective_limit);

        // Invert the output of collective as positive error needs to lead to decreasing collective
        collective_out *= -1.0;

    } else {
        // RPM sensor is bad, set collective to parameter provided trim
         collective_out = _motors_heli->get_coll_from_ang_deg(col_angle_trim.get());
    }

    _attitude_control->set_throttle_out(collective_out, false, COLLECTIVE_CUTOFF_FREQ_HZ);

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: ARHS
    // @Vehicles: Copter
    // @Description: helicopter AutoRotation Head Speed (ARHS) controller information
    // @Field: TimeUS: Time since system startup
    // @Field: Tar: Normalised target head speed
    // @Field: Act: Normalised measured head speed
    // @Field: Err: Head speed controller error
    // @Field: P: P-term for head speed controller response
    // @Field: FF: FF-term for head speed controller response
    // @Field: Out: Output from the head speed controller
    // @Field: H: Measurement health, 1 = healthy, 0 = unhealthy

    // Write to data flash log
    AP::logger().WriteStreaming("ARHS",
                                "TimeUS,Tar,Act,Err,P,I,Out,H",
                                "s-------",
                                "F0000000",
                                "QffffffB",
                                AP_HAL::micros64(),
                                _target_head_speed,
                                head_speed_norm,
                                _hs_ctrl.get_error(),
                                _hs_ctrl.get_P(),
                                _hs_ctrl.get_I(),
                                _motors_heli->get_throttle(),
                                uint8_t(measurement_valid));
#endif
}


// Get the normalised mean headspeed of both heads if dual is enabled
// Non-averaged value is returned if single heli
bool AC_Autorotation::get_mean_norm_headspeed(float& norm_rpm) const
{
    // Speed of first head
    bool headspeed_valid = get_norm_head_speed(norm_rpm, _param_rpm_instance.get());

    if (_dual_enable.get() == 0) {
        // only need the measurement for the first head so return early
        return headspeed_valid;
    }

    // Speed of second head
    float norm_rpm2 = 0.0;
    headspeed_valid &= get_norm_head_speed(norm_rpm2, _param_rpm2_instance.get());

    // Compute the average
    norm_rpm = (norm_rpm + norm_rpm2) * 0.5;
    return headspeed_valid;
}


// Get measured head speed and normalise by head speed set point. Returns false if a valid rpm measurement cannot be obtained
bool AC_Autorotation::get_norm_head_speed(float& norm_rpm, uint8_t instance) const
{
    // Assuming zero rpm is safer as it will drive collective in the direction of increasing head speed
    float current_rpm = 0.0;

#if AP_RPM_ENABLED
    // Get singleton for RPM library
    const AP_RPM *rpm = AP_RPM::get_singleton();

    // Checking to ensure no nullptr, we do have a pre-arm check for this so it will be very bad if RPM has gone away
    if (rpm == nullptr) {
        return false;
    }

    // Check RPM sensor is returning a healthy status
    if (!rpm->get_rpm(instance, current_rpm)) {
        return false;
    }
#endif

    // Protect against div by zeros later in the code
    const float head_speed_set_point = get_headspeed_setpoint_rpm();

    // Normalize the RPM by the setpoint
    norm_rpm = current_rpm/head_speed_set_point;
    return true;
}

void AC_Autorotation::calc_yaw_rate_from_roll_target(float& yaw_rate_rad, float& lat_accel) {
    // We will be rolling into the wind to resist the error introduced by the wind pushing us
    // Get the current roll angle target from the attitude controller
    float target_roll_deg = _attitude_control->get_att_target_euler_cd().x * 0.01;

    const float roll_deadzone_deg = 2.0;
    if (fabsf(target_roll_deg) < roll_deadzone_deg) {
        target_roll_deg = 0.0;

    } else if (is_positive(target_roll_deg)) {
        // smooth out steps in target accels when coming out of the dead zone
        target_roll_deg -= roll_deadzone_deg;
    } else {
        // smooth out steps in target accels when coming out of the dead zone
        target_roll_deg += roll_deadzone_deg;
    }

    // protect against math error in the angle_deg_to_accel_mss calc
    target_roll_deg = constrain_float(target_roll_deg, -85, 85);

    // Convert it to a lateral acceleration
    float accel = angle_deg_to_accel_mss(target_roll_deg);

    // Maintain the same accel limit that we impose on the pilot inputs so as to prioritize managing forward accels/speeds
    accel = MIN(accel, get_accel_max() * 0.5);

    // Calculate the yaw rate from the lateral acceleration
    if (fabsf(_desired_vel) > MIN_MANOEUVERING_SPEED) {
        // Calc yaw rate from desired body-frame accels
        // this seems suspiciously simple, but it is correct
        // accel = r * w^2, r = radius and w = angular rate
        // radius can be calculated as the distance traveled in the time it takes to do 360 deg
        // One rotation takes: (2*pi)/w seconds
        // Distance traveled in that time: (s*2*pi)/w
        // radius for that distance: ((s*2*pi)/w) / (2*pi)
        // r = s / w
        // accel = (s / w) * w^2
        // accel = s * w
        // w = accel / s
        yaw_rate_rad = accel / _desired_vel;

        // Only apply the acceleration if we are going to apply the yaw rate
        lat_accel = accel;
    }
}


// When using dual heli, we want to ensure that the head speed is not too slow before
// allowing pitch control. Pitch limit flag is set if either headspeed is above the target.
void AC_Autorotation::check_headspeed_limits(void)
{
    if (_dual_enable.get() == 0) {
        // Do not need this check for single helis
        head_speed_pitch_limit = false;
        return;
    }

    // Check head 1
    float norm_rpm;
    if (!get_norm_head_speed(norm_rpm, _param_rpm_instance.get()) || (norm_rpm < _safe_head_speed_ratio.get())) {
        head_speed_pitch_limit = true;
        return;
    }

    // Check head 2
    float norm_rpm2;
    if (!get_norm_head_speed(norm_rpm2, _param_rpm2_instance.get()) || (norm_rpm2 < _safe_head_speed_ratio.get())) {
        head_speed_pitch_limit = true;
        return;
    }

    // Add some headroom between recovery and headspeed ratio by only enabling pitch when both heads are at or above target speed
    if ((norm_rpm >= HEAD_SPEED_TARGET_RATIO) && (norm_rpm2 >= HEAD_SPEED_TARGET_RATIO)) {
        head_speed_pitch_limit = false;
    }
}


// Update speed controller
void AC_Autorotation::update_navigation_controller(float pilot_norm_accel)
{
    const AP_AHRS &ahrs = AP::ahrs();

    float yaw_rate_rads = 0.0;
    Vector3f desired_velocity_bf;
    Vector3f desired_accel_bf;
    Vector3f desired_velocity_NED;
    Vector3f desired_accel_NED;

    // Set body frame velocity targets
    desired_velocity_bf.x = _desired_vel;
    desired_velocity_bf.y = 0.0; // Start with the assumption that we want zero side slip
    desired_velocity_bf.z = MAX(get_bf_speed_down(), 0.0); // Always match the current vz. We add this in because the vz is significant proportion of the speed that needs to be accounted for when we rotate from body frame to earth frame.

    switch (Nav_Mode(_param_nav_mode.get())) {
        case Nav_Mode::TURN_INTO_WIND: {
            // Mode seeks to maintain body frame velocity target whilst turning into wind.

            // The position controller will be resisting the wind by rolling into the wind.
            // Calc the needed lateral accel and yaw rate to make a co-ordinated turn into roll.
            calc_yaw_rate_from_roll_target(yaw_rate_rads, desired_accel_bf.y);

            // Convert body frame targets into earth frame
            desired_velocity_NED = ahrs.body_to_earth(desired_velocity_bf);
            desired_accel_NED = ahrs.body_to_earth(desired_accel_bf);

            break;
        }
        case Nav_Mode::CROSS_TRACK: {
            // Mode seeks to maintain either the velocity vector or the heading (slow speed case)
            // recorded on mode init, then yawing into wind to cross-track, reducing drag and roll angle.

            // The position controller will be resisting the wind by rolling into the wind.
            // Calc the needed yaw rate needed to turn into wind and to cross track.
            float unused_lat_accel = 0.0;
            calc_yaw_rate_from_roll_target(yaw_rate_rads, unused_lat_accel);

            // Convert body frame targets into earth frame
            const Vector3f desired_velocity_ef_NED = ahrs.body_to_earth(desired_velocity_bf);

            // Calculate the resulting speed that we want in the NE plane
            const float desired_NE_speed = desired_velocity_ef_NED.xy().length();

            // We already have the earth-frame unit vector that we want to maintain the velocity vector along.
            // Now we can use the track vector to apportion the vector direction
            desired_velocity_NED = Vector3f{_track_vector, 0.0} * desired_NE_speed;

            break;
        }
        case Nav_Mode::PILOT_LAT_ACCEL:
        default: {
            // Mode lets pilot request a lateral acceleration. The roll and yaw rates
            // are then calculated to perform a coordinated turn.

            // Pilot can request as much as 1/2 of the max accel laterally to perform a turn.
            // We only allow up to half as we need to prioritize building/maintaining airspeed.
            desired_accel_bf.y = pilot_norm_accel * get_accel_max() * 0.5;

            // In the case where we have low ground speed (e.g. touchdown phase) we still want to let the
            // pilot yaw. We use the min manoeuvering speed as the default "time constant" so that the yaw
            // feels consistant below the min ground speed and to avoid a div by zero.
            float bf_fwd_speed = MAX(fabsf(desired_velocity_bf.x), MIN_MANOEUVERING_SPEED);

            // Calc yaw rate from desired body-frame accels
            // this seems suspiciously simple, but it is correct
            // accel = r * w^2, r = radius and w = angular rate
            // radius can be calculated as the distance traveled in the time it takes to do 360 deg
            // One rotation takes: (2*pi)/w seconds
            // Distance traveled in that time: (s*2*pi)/w
            // radius for that distance: ((s*2*pi)/w) / (2*pi)
            // r = s / w
            // accel = (s / w) * w^2
            // accel = s * w
            // w = accel / s
            yaw_rate_rads = desired_accel_bf.y / bf_fwd_speed;

            // Only perform coordinated turns when above the min manoeuvering speed
            if (fabsf(desired_velocity_bf.x) < MIN_MANOEUVERING_SPEED) {
                desired_accel_bf.y = 0.0;
            }

            // Convert body frame targets into earth frame
            desired_velocity_NED = ahrs.body_to_earth(desired_velocity_bf);
            desired_accel_NED = ahrs.body_to_earth(desired_accel_bf);
        }
    }

    // zero yaw rate if we are in a heading hold
    if (_heading_hold) {
        yaw_rate_rads = 0.0;
    }

    // We only use 2D NE position controller so discard z target and convert to cm
    Vector2f desired_velocity_NE_cm = desired_velocity_NED.xy() * 100.0;
    Vector2f desired_accel_NE_cm = desired_accel_NED.xy() * 100.0;

    // Check with motors that we have not saturated
    bool motors_limit = false;
    motors_limit |= _motors_heli->limit.pitch;
    if (_dual_enable.get() > 0) {
        // Dual heli uses combined controls, so we need to check for limits in motors
        // as collective control can be used for pitch
        motors_limit |= _motors_heli->limit.throttle_upper;
        motors_limit |= _motors_heli->limit.throttle_lower;
    }

    // Update the position controller
    _pos_control->input_vel_accel_NE_cm(desired_velocity_NE_cm, desired_accel_NE_cm, true);

    if (motors_limit || head_speed_pitch_limit) {
        _pos_control->set_externally_limited_NE();
    }

    _pos_control->update_NE_controller();

    // Output to the attitude controller
    _attitude_control->input_thrust_vector_rate_heading_rads(_pos_control->get_thrust_vector(), yaw_rate_rads);

    // Calculate the unit velocity vector for wp bearing telemetry data
    if (desired_velocity_NE_cm.length() > MIN_MANOEUVERING_SPEED * 100.0) {
        _bearing_vector = desired_velocity_NE_cm;
        _bearing_vector.normalize();
    } else {
        _bearing_vector = {0.0, 0.0};
    }

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: ARSC
    // @Vehicles: Copter
    // @Description: Helicopter AutoRotation Speed Controller (ARSC) information 
    // @Field: TimeUS: Time since system startup
    // @Field: VX: Desired velocity X in body frame
    // @Field: VY: Desired velocity Y in body frame
    // @Field: AX: Desired Acceleration X in body frame
    // @Field: AY: Desired Acceleration Y in body frame

    AP::logger().WriteStreaming("ARSC",
                                "TimeUS,VX,VY,VZ,AX,AY,TX,TY",
                                "snnnoo--",
                                "F0000000",
                                "Qfffffff",
                                AP_HAL::micros64(),
                                desired_velocity_bf.x,
                                desired_velocity_bf.y,
                                desired_velocity_bf.z,
                                desired_accel_bf.x,
                                desired_accel_bf.y,
                                _track_vector.x,
                                _track_vector.y);

#endif
}

// Calculate an initial estimate of when the aircraft needs to flare
// This function calculates and stores a few constants that are used again in subsequent flare height update calculations
void AC_Autorotation::initial_flare_hgt_estimate(void)
{
    // Get blade pitch angle, accounting for non-zero zero thrust blade pitch for the asymmetric blade case
    float blade_pitch_hover_rad = radians(_motors_heli->get_hover_coll_ang());

    // Ensure safe math operations below by constraining blade_pitch_hover_rad to be > 0.
    // Assuming 0.1 deg will never be enough to blade pitch angle to maintain hover.
    blade_pitch_hover_rad = MAX(blade_pitch_hover_rad, radians(0.1));

    static const float CL_ALPHA = M_2PI;
    const float b = get_solidity() * CL_ALPHA;
    float disc_area = M_PI * 0.25 * sq(_param_diameter.get()); // (m^2)
    if (_dual_enable > 0) {
        disc_area *= 2.0;
    }

    // Calculating the equivalent inflow ratio (average across the whole blade)
    float lambda_eq = -b / 16.0 + safe_sqrt(sq(b) / 256.0 + b * blade_pitch_hover_rad / 12.0);

    // Tip speed = head speed (rpm) / 60 * 2pi * rotor diameter/2. Eq below is simplified.
    float tip_speed_auto = get_headspeed_setpoint_rpm() * M_PI * _param_diameter.get() / 60.0;

    // Calc the coefficient of thrust in the hover
    float c_t_hover = 0.5 * b * (blade_pitch_hover_rad / 3.0 - lambda_eq / 2.0);
    c_t_hover = MAX(c_t_hover, 0.00001); //TODO improve this constrain
    _hover_thrust = c_t_hover * SSL_AIR_DENSITY * disc_area * sq(tip_speed_auto);

    // Estimate rate of descent
    static const float ASSUMED_CD0 = 0.011;
    const float sink_rate = ((0.25 * get_solidity() * ASSUMED_CD0 / c_t_hover) + (sq(c_t_hover) / (get_solidity() * ASSUMED_CD0))) * tip_speed_auto;

    // Calc flare altitude
    // TODO: We need to come up with some way to modify the forward speed target here as the target speed is
    // body frame forward speed and the flare height calculation needs earth frame forward speed.  The trouble is
    // that this is a projection into the future so we cannot provide a measured value at this stage
    float des_spd_fwd = _param_target_speed.get();
    calc_flare_hgt(des_spd_fwd, -1.0 * sink_rate);

    // Always save the initial flare and touchdown height estimates as they do not use measured conditions like the
    // continuous update method hence the model calculations above already assume steady state conditions.
    _touchdown_hgt.set(_calculated_touchdown_hgt);
    _flare_hgt.set(_calculated_flare_hgt);

    gcs().send_text(MAV_SEVERITY_INFO, "Ct/sigma=%.4f W=%.2f kg flare_alt=%.2f", c_t_hover/get_solidity(), _hover_thrust/GRAVITY_MSS, _flare_hgt.get());
    gcs().send_text(MAV_SEVERITY_INFO, "sink rate=%.3f", sink_rate);
    gcs().send_text(MAV_SEVERITY_INFO, "inflow spd=%.3f", lambda_eq*tip_speed_auto);
}

void AC_Autorotation::calc_flare_hgt(const float fwd_speed, float climb_rate)
{
    // we must be descending for this maths to sensible
    if (!is_negative(climb_rate)) {
        return;
    }

    // Keep the slow filter of vel z up to date. We always update this with a fresh measurement as the
    // climb_rate argument maybe from an approximated climb rate not a measured one
    _lagged_vel_z.apply(get_ef_velocity_up(), _dt);

    // Estimate total rotor drag force coefficient in the descent
    // This is not the fully non-dimensionalized drag force to avoid having to constantly
    // dividing and multiply vehicle constants like rotor diameter
    float CR = _hover_thrust / sq(climb_rate);
    CR = constrain_float(CR, 0.01, 1.7); // TODO: confirm typical range of drag coefficients expected

    // Compute speed module and glide path angle during descent
    const float speed_module = MAX(norm(climb_rate, fwd_speed), 0.1); // (m/s)
    const float glide_angle = M_PI / 2 - safe_asin(fwd_speed / speed_module); // (rad)

    // Estimate inflow velocity at beginning of flare
    float entry_inflow = - speed_module * sinf(glide_angle + radians(AP_ALPHA_TPP));

    const float k_1 = safe_sqrt(_hover_thrust / CR); // TODO: discuss: in the initial estimate k1 will always equal climb rate, see definition of CR above
    const float ASYMPT_THRESHOLD = 0.05;
    const float final_sink_rate = climb_rate + ASYMPT_THRESHOLD; //relative to rotor ref system

    // Protect against div by 0 case
    if (is_zero(final_sink_rate + k_1)) {
        climb_rate -= ASYMPT_THRESHOLD;
    }
    if (is_zero(entry_inflow + k_1)) {
        entry_inflow -= ASYMPT_THRESHOLD;
    }

    // Estimate flare duration
    const float m = _hover_thrust / GRAVITY_MSS;
    const float k_3 = safe_sqrt((CR * GRAVITY_MSS) / m);
    const float k_2 = 1 / (2 * k_3) * logf(fabsf((entry_inflow - k_1)/(entry_inflow + k_1)));
    const float a = logf(fabsf((final_sink_rate - k_1)/(final_sink_rate + k_1)));
    const float b = logf(fabsf((entry_inflow - k_1)/(entry_inflow + k_1)));
    const float delta_t_flare = (1 / (2 * k_3)) * (a - b);

    // Estimate flare delta altitude
    const float k_4 = (2 * k_2 * k_3) + (2 * k_3 * delta_t_flare);
    const float flare_distance = ((k_1 / k_3) * (k_4 - logf(fabsf(1-expf(k_4))) - (2 * k_2 * k_3 - logf(fabsf(1 - expf(2 * k_2 * k_3)))))) - k_1 * delta_t_flare;
    const float delta_h = -flare_distance * cosf(radians(AP_ALPHA_TPP));

    // Estimate altitude to begin touchdown phase. This value is preserved for logging without constraining it to the min/max allowable
    _calculated_touchdown_hgt = -1.0 * climb_rate * get_touchdown_time();

    // Ensure that we keep the calculated touchdown height within the allowable min/max values
    // before it is used in the _calculated_flare_hgt. Not doing this can lead to a zero-ground speed 
    // condition before the state machine is permited to progress onto the touchdown phase.
    const float calc_td_hgt = constrain_float(_calculated_touchdown_hgt, _touchdown_hgt.min_height.get(), _touchdown_hgt.max_height.get());

    // Total delta altitude to ground
    _calculated_flare_hgt = calc_td_hgt + delta_h;

    // Save these calculated values for use, if the conditions were appropriate for us to consider the value reliable
    const float speed_error = fabsf(_desired_vel - get_bf_speed_forward());
    if (speed_error < 0.2 * _desired_vel &&  // Check that our forward speed is withing 20% of target
        fabsf(_lagged_vel_z.get() - get_ef_velocity_up()) < 1.0) // Sink rate can be considered approx steady
    {
        _touchdown_hgt.set(calc_td_hgt);
        _flare_hgt.set(_calculated_flare_hgt);
    }
}

void AC_Autorotation::update_flare_hgt(void)
{
    const float vel_z = get_ef_velocity_up();
    const float fwd_speed = get_ef_speed_forward(); // (m/s)

    // update the flare height calc
    calc_flare_hgt(fwd_speed, vel_z);

    // TODO figure out cause of spikes in flare height estimate
}

bool AC_Autorotation::below_flare_height(void) const
{
    // we cannot transition to the flare phase if we do not know what height we are at
    if (!_hagl_valid) {
        return false;
    }
    return _hagl < _flare_hgt.get();
}

void AC_Autorotation::update_trajectory(float time_now, float A0, float V0, float P0, float tj1, float tj2, float tj3, float& Jt, float& At, float& Vt, float& Pt) const
{
    const float T1 = 2 * tj1;
    const float T2 = tj2;
    const float T3 = 2 * tj3;
    const float Jm = get_td_jerk_max();

    // Section 1
    const float t1 = MIN(time_now, T1);
    float J1, A1, V1, P1;
    _scurve.calc_javp_for_segment_incr_jerk(t1, tj1, Jm, A0, V0, P0, J1, A1, V1, P1);

    if (time_now <= T1) {
        // We are still in phase 1
        Jt = J1; At = A1; Vt = V1; Pt = P1;
        return;
    }

    // Section 2
    const float t2 = MIN(time_now - T1, T2);
    float J2, A2, V2, P2;
    _scurve.calc_javp_for_segment_const_jerk(t2, J1, A1, V1, P1, J2, A2, V2, P2);

    if ( time_now <= T1 + T2) {
        // We are still in phase 2
        Jt = J2; At = A2; Vt = V2; Pt = P2;
        return;
    }

    // Section 3
    const float t3 = MIN(time_now - T1 - T2, T3);
    float J3, A3, V3, P3;
    _scurve.calc_javp_for_segment_incr_jerk(t3, tj3, -Jm, A2, V2, P2, J3, A3, V3, P3);

    if ( time_now <= T1 + T2 + T3) {
        // we are still in phase 3
        Jt = J3; At = A3; Vt = V3; Pt = P3;
        return;
    }

    // Section 4
    // Constant velocity descent (after the "touchdown manoeuver")
    // All being well we arrived here at a nice smooth trajectory so jerk and accel is zero. However, there is a corner case in the
    // very low hover autorotation where we have to force the jerk and accel targets to zero so that the vehicle simply does its
    // best to maintain the descent rate and bring the accel under control.
    const float t4 = time_now - T1 - T2 - T3;
    _scurve.calc_javp_for_segment_const_jerk(t4, 0.0, 0.0, V3, P3, Jt, At, Vt, Pt);
}

// Solving for roots of the continuity equations in the non-accel limited case
bool AC_Autorotation::calc_cosine_trajectory_times(float a0, float v0, float a3, float v3, float jm, float& tj1, float& tj3) const
{
    float discriminant = 0.5 * ((a0 * a0) + (a3 * a3) + jm * (v3 - v0));
    if (discriminant < 0) {
        // Invalid solution if the discriminant is negative. we will just wait until
        // the boundary conditions have changed to a state that we can solve for.
        tj1 = 0;
        tj3 = 0;
        return false;
    }

    // Compute the trajectory time periods
    tj1 = (-a0 + safe_sqrt(discriminant)) / jm;
    tj3 = (-a3 + safe_sqrt(discriminant)) / jm;

    return is_positive(tj1) && is_positive(tj3);
}

// Assuming a three-section s-curve trajectory we keep the jerk max constant and calculate
// the two cosine periods tj1 & tj3 that satisfy our entry (a0, v0) and exit (a3, v3) conditions
// tj2 becomes non-zero if we need to accel limit the trajectory
// See ./Derivations.md for more details
bool AC_Autorotation::calc_scurve_trajectory_times(float a0, float v0, float p0, float& tj1, float& tj2, float& tj3) const
{
    bool solution_valid = false;
    tj1 = 0.0;
    tj2 = 0.0;
    tj3 = 0.0;
    const float v3 = get_land_speed();
    // Due to the assumed trajectory shape that has been constructed, we wait for the entry velocity to be <= to the exit condition
    // This reduces the number of scenarios that we need to handle and simplifies the code structure, focusing on the most probable cases.
    // For this application, we can simply wait for the descent rate to increase which is an assured thing in an autorotation.
    if (v0 > v3) {
        return solution_valid;
    }

    // start by calculating the unconstrained acceleration trajectory to get the times needed for this case
    const float a3 = 0.0;
    const float jm = get_td_jerk_max();
    solution_valid = calc_cosine_trajectory_times(a0, v0, a3, v3, jm, tj1, tj3);

    // See if we need to apply acceleration limiting
    const float am = get_td_accel_max();
    const float a_peak = a0 + jm * tj1;
    if ((a_peak < am) && solution_valid) {
        // we do not need to apply any accel limits, we can begin touch down
        return solution_valid;
    }

    // If we got this far we either need to apply accel limits or the previously invalid solution my have a solution with this approach
    // Calculate the time periods required to meet the acceleration boundary conditions
    tj1 = (am - a0) / jm;
    tj3 = (am - a3) / jm;

    // Calculate the time required to meet the velocity condition
    tj2 = (v3 - v0 - (2 * a0 * tj1) - (jm * tj1 * tj1) - (2 * a3 * tj3) - (jm * tj3 * tj3)) / am;

    // Check that the exit conditions match our desired conditions
    const float end_time = (tj1 + tj3) * 2 + tj2;
    float je, ae, ve, pe;
    update_trajectory(end_time, a0, v0, p0, tj1, tj2, tj3, je, ae, ve, pe);

    const float TOL = 1e-5;
    solution_valid = ((fabsf(ve - v3) < TOL) && (fabsf(ae - a3) < TOL));

    return solution_valid;
}


// Determine if we are above the touchdown height using the descent rate and param values
bool AC_Autorotation::should_begin_touchdown(void)
{
    enum class Reason : int8_t {
        ACCEL_LIMITED_INSUFFICIENT_S4_TIME = -6,
        INVALID_ACCEL_LIMIT = -5,
        VALID_BELOW_ACCEL_LIMIT_NO_INTERSECTION = -4,
        SLOWER_THAN_LANDING_SPEED  = -3,
        ABOVE_TD_MAX_HEIGHT  = -2,
        HAGL_INVALID = -1,
        NO_SOLUTION = 0,
        BELOW_TD_MIN_HEIGHT = 1,
        VALID_BELOW_ACCEL_LIMIT_TO_BUFFER_HGT = 2,
        VALID_BELOW_ACCEL_LIMIT_S4_DESCENT = 3,
        VALID_ACCEL_LIMTED_TO_BUFFER_HGT = 4,
        VALID_ACCEL_LIMITED_S4_DESCENT = 5,
    };

    // we cannot transition to the touchdown phase if we do not know what height we are at
    if (!_hagl_valid) {
        _td_start_reason = int8_t(Reason::HAGL_INVALID);
        return false;
    }

    // Check max height condition
    if (_hagl > _touchdown_hgt.max_height.get()) {
        // We are higher than our guarded height, no need to continue calculation
        _td_start_reason = int8_t(Reason::ABOVE_TD_MAX_HEIGHT);
        return false;
    }

    // rest all trajectory times to zero
    _tj1 = 0.0;
    _tj2 = 0.0;
    _tj3 = 0.0;

    // Check min height, target speed condition, this case is designed for the very low hover autorotation case
    const float v0 = _pos_control->get_vel_estimate_NEU_ms().z;
    if ((_hagl < _touchdown_hgt.min_height.get()) && (v0 <= get_land_speed())) {
        // Jump to constant descent case
        _td_start_reason = int8_t(Reason::BELOW_TD_MIN_HEIGHT);
        return true;
    }

    const float a0 = _pos_control->get_measured_accel_U_mss();
    const float v3 = get_land_speed();
    // Due to the assumed trajectory shape that has been constructed, we wait for the entry velocity to be <= to the exit condition
    // This reduces the number of scenarios that we need to handle and simplifies the code structure, focusing on the most probable cases.
    // For this application, we can simply wait for the descent rate to increase which is an assured thing in an autorotation.
    if (v0 > v3) {
        _td_start_reason = int8_t(Reason::SLOWER_THAN_LANDING_SPEED);
        return false;
    }

    // start by calculating the unconstrained acceleration trajectory to get the times needed for this case
    float a3 = 0.0;
    const float jm = get_td_jerk_max();
    bool solution_valid = calc_cosine_trajectory_times(a0, v0, a3, v3, jm, _tj1, _tj3);

    // Check that the exit conditions match our desired conditions
    float manoeuvre_time = (_tj1 + _tj3) * 2 + _tj2;
    float je, ae, ve, pe;
    update_trajectory(manoeuvre_time, a0, v0, _hagl, _tj1, _tj2, _tj3, je, ae, ve, pe);

    // See if we need to apply acceleration limiting
    const float am = get_td_accel_max();
    const float a_peak = a0 + jm * _tj1;
    if ((a_peak < am) && solution_valid) {
        if ((pe <= BUFFER_HEIGHT)) {
            // we do not need to apply any accel limits, we can begin touch down
            _td_start_reason = int8_t(Reason::VALID_BELOW_ACCEL_LIMIT_TO_BUFFER_HGT);
            return true;
        }

        // See if we have sufficient time to intersect the floor with the constant descent rate section
        const float tj4 = get_touchdown_time() - manoeuvre_time;
        const float p4 = pe + tj4 * v3;
        if (is_positive(tj4) && (p4 <= 0.0)) {
            // we have enough time (therefore enough rotor head energy) to safely intersect the ground descending at constant speed
            _td_start_reason = int8_t(Reason::VALID_BELOW_ACCEL_LIMIT_S4_DESCENT);
            return true;
        }

        // If we got this far then we have a valid solution for accel and vel boundary conditions but we will not meet out exit position criteria
        _td_start_reason = int8_t(Reason::VALID_BELOW_ACCEL_LIMIT_NO_INTERSECTION);
        return false;
    }

    // If we got this far we either need to apply accel limits or the previously invalid solution my have a solution with this approach (e.g. tj1 = 0 case)
    // Calculate the time periods required to meet the acceleration boundary conditions
    _tj1 = (am - a0) / jm;
    _tj3 = (am - a3) / jm;

    // Calculate the time required to meet the velocity condition
    _tj2 = (v3 - v0 - (2 * a0 * _tj1) - (jm * _tj1 * _tj1) - (2 * a3 * _tj3) - (jm * _tj3 * _tj3)) / am;

    // Check that the exit conditions match our desired conditions
    manoeuvre_time = (_tj1 + _tj3) * 2 + _tj2;
    update_trajectory(manoeuvre_time, a0, v0, _hagl, _tj1, _tj2, _tj3, je, ae, ve, pe);

    const float TOL = 1e-5;
    solution_valid = ((fabsf(ve - v3) < TOL) && (fabsf(ae - a3) < TOL));

    if (!solution_valid) {
        _td_start_reason = int8_t(Reason::INVALID_ACCEL_LIMIT);
        return false;
    }

    // look forward to see if we intersect with the ground before the manoeuvre is complete
    if (pe <= BUFFER_HEIGHT) {
        // we need to initiate the manoeuvre now
        _td_start_reason = int8_t(Reason::VALID_ACCEL_LIMTED_TO_BUFFER_HGT);
        return true;
    }

    // Check what time we will have remaining from the touchdown time parameter value
    const float tj4 = get_touchdown_time() - manoeuvre_time;

    if (tj4 < 0) {
        // invalid time. we already know that we won't intersect the exit position by the end of the manoeuvre
        // and we don't have any additional time specifed by touchdown time, so don't start the manouver,
        _td_start_reason = int8_t(Reason::ACCEL_LIMITED_INSUFFICIENT_S4_TIME);
        return false;
    }

    // See if we intersect the ground by the end of the constant velocity phase.
    const float p4 = pe + tj4 * v3;
    if (p4 <= 0.0) {
        // we need to initiate the manoeuvre now
        _td_start_reason = int8_t(Reason::VALID_ACCEL_LIMITED_S4_DESCENT);
        return true;
    }

    // If we got this far then we do not need to start touchdown manoeuvre yet
    _td_start_reason = int8_t(Reason::NO_SOLUTION);
    return false;
}

// Determine if we should be performing a hover autorotation
// We use a speed height cone from flare height at zero speed to min touchdown height at cruise speed
// to decide whether we consider it a hover autorotation
bool AC_Autorotation::should_hover_autorotate(void) const
{
    // we cannot judge if we should do a hover autorotation we do not know what height we are at
    if (!_hagl_valid) {
        return false;
    }

    const float bf_forwad_spd = get_bf_speed_forward();

    // Get the max speed for a given hagl that we expect 
    const float min_height = linear_interpolate(_flare_hgt.get(), _touchdown_hgt.min_height.get(), bf_forwad_spd, 0.0, _param_target_speed);

    return _hagl < min_height;

}

// Zero velocity and accel to bring all of the controls into position to trip Copter's landing detector.
void AC_Autorotation::run_landed(void)
{
    // Update the position controller with zero vels and accels
    Vector2f desired_velocity_NE_cm;
    Vector2f desired_accel_NE_cm;
    _pos_control->input_vel_accel_NE_cm(desired_velocity_NE_cm, desired_accel_NE_cm, true);
    _pos_control->soften_for_landing_NE();
    _pos_control->update_NE_controller();

    // Output to the attitude controller
    const float yaw_rate_rads = 0.0;
    _attitude_control->input_thrust_vector_rate_heading_rads(_pos_control->get_thrust_vector(), yaw_rate_rads);

    // To get to this phase the collective has to be below land col min due to the landed check
    // So we do not need to do anything with collective at this point
}

// Determine the body frame forward speed in m/s
float AC_Autorotation::get_bf_speed_forward(void) const
{
    return get_bf_vel().x;
}

// Determine the body frame down speed in m/s
float AC_Autorotation::get_bf_speed_down(void) const
{
    return get_bf_vel().z;
}

// Determine the body frame forward speed in m/s
Vector3f AC_Autorotation::get_bf_vel(void) const
{
    Vector3f vel_NED = {0,0,0};
    const AP_AHRS &ahrs = AP::ahrs();
    if (ahrs.get_velocity_NED(vel_NED)) {
        vel_NED = ahrs.earth_to_body(vel_NED);
    }
    // TODO: need to improve the handling of the velocity NED not ok case
    return vel_NED;
}

// Determine the earth frame forward speed in m/s
float AC_Autorotation::get_ef_speed_forward(void) const
{
    const AP_AHRS &ahrs = AP::ahrs();
    Vector2f groundspeed_vector = ahrs.groundspeed_vector();
    // Calculate the forward speed in ef by projecting the ground speed vector onto our heading vector.
    float speed_forward = groundspeed_vector * Vector2f{ahrs.cos_yaw(), ahrs.sin_yaw()}; // (m/s)
    return speed_forward;
}

// Get the earth frame vertical velocity in m/s, positive is up
float AC_Autorotation::get_ef_velocity_up(void) const
{
    const AP_AHRS &ahrs = AP::ahrs();
    Vector3f vel_NED = {0,0,0};
    IGNORE_RETURN(ahrs.get_velocity_NED(vel_NED));
    // TODO: need to improve the handling of the velocity NED not ok case
    return vel_NED.z * -1.0;
}

// Update the height above ground estimate in meters
void AC_Autorotation::update_hagl(void)
{
    // Always reset the hagl valid flag
    _hagl_valid = false;

#if AP_RANGEFINDER_ENABLED

    // Keep the ground surface object up to date
    _ground_surface.update();

    // Get the height above ground estimate from the surface tracker library.
    int32_t hagl = 0;
    _hagl_valid = _ground_surface.get_rangefinder_height_interpolated_cm(hagl);
    _hagl = float(hagl) * 0.01;

    // TODO: improve fail over to terrain and then home

#endif //AP_RANGEFINDER_ENABLED
}

#if HAL_LOGGING_ENABLED
// Logging of lower rate autorotation specific variables. This is meant for stuff that
// doesn't need a high rate, e.g. controller variables that are need for tuning.
void AC_Autorotation::log_write_autorotation(uint8_t phase) const
{
    // enum class for bitmask documentation in logging
    enum class AC_Autorotation_Landed_Reason : uint8_t {
        LOW_SPEED = 1<<0, // true if below 1 m/s
        LAND_COL  = 1<<1, // true if collective below land col min
        IS_STILL  = 1<<2, // passes inertial nav is_still() check
    };

    uint8_t reason = 0;
    if (_landed_reason.min_speed) {
        reason |= uint8_t(AC_Autorotation_Landed_Reason::LOW_SPEED);
    }
    if (_landed_reason.land_col) {
        reason |= uint8_t(AC_Autorotation_Landed_Reason::LAND_COL);
    }
    if (_landed_reason.is_still) {
        reason |= uint8_t(AC_Autorotation_Landed_Reason::IS_STILL);
    }

    // @LoggerMessage: ARO1
    // @Vehicles: Copter
    // @Description: First Helicopter AutoROtation (ARO1) information message
    // @Field: TimeUS: Time since system startup
    // @Field: Phase: Autorotation flight phase
    // @Field: MH: Measured Height
    // @Field: CFH: Unfiltered Calculated Flare Height
    // @Field: FH: Flare Height

    // Write to data flash log
    AP::logger().WriteStreaming("ARO1",
                                "TimeUS,Phase,MH,CFH,FH",
                                "s-mmm",
                                "F-000",
                                "QBfff",
                                AP_HAL::micros64(),
                                phase,
                                _hagl,
                                _calculated_flare_hgt,
                                _flare_hgt.get()
                                );

    // @LoggerMessage: ARO2
    // @Vehicles: Copter
    // @Description: Second Helicopter AutoROation (ARO2) information message
    // @Field: TimeUS: Time since system startup
    // @Field: CTDH: Unfiltered Calculated Touchdown Height
    // @Field: TDH: Touchdown Height
    // @Field: T1: Touchdown phase 1 time
    // @Field: T2: Touchdown phase 2 time
    // @Field: T3: Touchdown phase 3 time
    // @Field: TDR: Touchdown reason
    // @Field: LR: Landed Reason state flags
    // @FieldBitmaskEnum: LR: AC_Autorotation_Landed_Reason

    // Write to data flash log
    AP::logger().WriteStreaming("ARO2",
                                "TimeUS,CTDH,TDH,T1,T2,T3,TDR,LR",
                                "smmsss--",
                                "F00000--",
                                "QfffffbB",
                                AP_HAL::micros64(),
                                _calculated_touchdown_hgt,
                                _touchdown_hgt.get(),
                                _tj1*2.0,
                                _tj2,
                                _tj3*2.0,
                                _td_start_reason,
                                reason);
}
#endif  // HAL_LOGGING_ENABLED

// Arming checks for autorotation, mostly checking for miss-configurations
bool AC_Autorotation::arming_checks(size_t buflen, char *buffer) const
{
    if (!enabled()) {
        // Don't run arming checks if not enabled
        return true;
    }

    // Check for correct RPM sensor config
#if AP_RPM_ENABLED
    // Get singleton for RPM library
    const AP_RPM *rpm = AP_RPM::get_singleton();

    // Get current rpm, checking to ensure no nullptr
    if (rpm == nullptr) {
        hal.util->snprintf(buffer, buflen, "Can't access RPM");
        return false;
    }

    // Sanity check that the designated rpm sensor instance is there
    if (_param_rpm_instance.get() < 0) {
        hal.util->snprintf(buffer, buflen, "RPM instance <0");
        return false;
    }

    if (!rpm->enabled(_param_rpm_instance.get())) {
        hal.util->snprintf(buffer, buflen, "RPM%i not enabled", _param_rpm_instance.get()+1);
        return false;
    }
#endif

    // Check that heli motors is configured for autorotation
    if (!_motors_heli->rsc_autorotation_enabled()) {
        hal.util->snprintf(buffer, buflen, "H_RSC_AROT_* not configured");
        return false;
    }

#if AP_RANGEFINDER_ENABLED
    // Check that we can see a healthy rangefinder
    if (!_ground_surface.rangefinder_configured()) {
        hal.util->snprintf(buffer, buflen, "Downward rangefinder not configured");
        return false;
    }

    const RangeFinder *rangefinder = RangeFinder::get_singleton();
    if (rangefinder == nullptr) {
        hal.util->snprintf(buffer, buflen, "Downward rangefinder not configured");
        return false;
    }

    // Sanity check that the rangefinder is adequate and can at least read up to the minimum height required to flare
    if (float(rangefinder->max_distance_orient(ROTATION_PITCH_270)) < _flare_hgt.min_height.get()) {
        hal.util->snprintf(buffer, buflen, "Rngfnd max distance < min flare height");
        return false;
    }
#else
    // We currently rely too heavily on rangefinders, thow an arming check if we do not have Rangefinder compiled in
    hal.util->snprintf(buffer, buflen, "Rngfnd not compiled in");
    return false;
#endif

    // Check that the blade pitch collective appears plausible. Would expect at least 1 deg of blade pitch required for hover.
    if (_motors_heli->get_hover_coll_ang() < 1.0) {
        hal.util->snprintf(buffer, buflen, "Hover pit < 1 deg. Check H_COL_* setup");
        return false;
    }

    // Check that the collective trim angle appears plausible
    if (_motors_heli->get_zero_thrust_angle_deg() < col_angle_trim.get()) {
        hal.util->snprintf(buffer, buflen, "AROT_COL_TRIM > H_COL_ZERO_THST");
        return false;
    }

    // Sanity check that min touchdown height is less than min flare height
    if (_flare_hgt.min_height.get() < _touchdown_hgt.min_height) {
        hal.util->snprintf(buffer, buflen, "FLR_MIN_HGT < TD_MIN_HGT");
        return false;
    }

    return true;
}

// Check if we believe we have landed. We need the landed state to zero all
// controls and make sure that the copter landing detector will trip
bool AC_Autorotation::check_landed(void)
{
    // minimum speed (m/s) used for "is moving" check
    const float min_moving_speed = 1.0;

    Vector3f velocity;
    const AP_AHRS &ahrs = AP::ahrs();
    _landed_reason.min_speed = ahrs.get_velocity_NED(velocity) && (velocity.length() < min_moving_speed);
    _landed_reason.land_col = _motors_heli->get_below_land_min_coll();
    _landed_reason.is_still = AP::ins().is_still();

    return _landed_reason.min_speed && _landed_reason.land_col && _landed_reason.is_still;
}

// Dynamically update time step used in autorotation controllers
void AC_Autorotation::set_dt(float delta_sec)
{
    if (is_positive(delta_sec)) {
        _dt = delta_sec;
        return;
    }
    _dt = 2.5e-3; // Assume 400 Hz
}

int32_t AC_Autorotation::get_wp_bearing(void) const
{
    Vector2f origin;
    return get_bearing_cd(origin, _bearing_vector) * 0.01;
}

float AC_Autorotation::wp_distance_m(void) const
{
    if (is_positive(_hagl) && _hagl_valid) {
    // We don't have waypoints to fly towards in this mode, so instead we send a crude estimate of
    // where we think we might land if we continue on this course.  We will likely be getting a glide
    // ratio of roughly 1:1 so reporting the HAGL will give an approximate distance travelled.
        return _hagl;
    }

    // We may not be in rangefinder range in which case we just return height above origin
    const AP_AHRS &ahrs = AP::ahrs();
    float down;
    if (ahrs.get_relative_position_D_origin_float(down)) {
        return fabsf(down);
    }

    // If we got this far things went really wrong, just return a fixed value so a direction vector
    // at least displays in GCS
    return 200.0;
}

float AC_Autorotation::crosstrack_error(void) const
{
    return _pos_control->crosstrack_error();
}

// Set height value with protections in place to ensure we do not exceed the minimum value
void AC_Autorotation::GuardedHeight::set(float hgt)
{
    // ensure that min and max heights are positive and remotely sensible
    const float min_hgt = MAX(min_height.get(), 0.2);
    const float max_hgt = MAX(max_height.get(), 1.0);
    height = constrain_float(hgt, min_hgt, max_hgt);
}

// Reset height to midpoint between min and max
void AC_Autorotation::GuardedHeight::reset(void)
{
    set((max_height.get() * min_height.get()) * 0.5);
}
