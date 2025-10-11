#include "Rover.h"

bool ModeLoiter::_enter()
{
    // set _destination to reasonable stopping point
    if (!g2.wp_nav.get_stopping_location(_destination)) {
        return false;
    }

    // initialise desired speed to current speed
    if (!attitude_control.get_forward_speed(_desired_speed)) {
        _desired_speed = 0.0f;
    }

    // initialise heading to current heading
    _desired_yaw_cd = ahrs.yaw_sensor;

    // always reset lazy loiter state
    lazy_loiter_active = false;

    return true;
}

void ModeLoiter::update()
{
    // get distance (in meters) to destination
    _distance_to_destination = rover.current_loc.get_distance(_destination);

    // Check if we should be doing a lazy loiter
    if (update_lazy_loiter()) {
        // We are in a lazy loiter, no need to run the rest of this function
        return;
    }

    const float loiter_radius = g2.sailboat.tack_enabled() ? g2.sailboat.get_loiter_radius() : g2.loit_radius.get();

    // 0 turn rate is no limit
    float turn_rate = 0.0;

    if (g2.sailboat.sail_enabled()) {
        // Update speed and heading for sailboat
        if (!g2.sailboat.calc_loiter_speed_and_heading(_distance_to_destination, _desired_speed, _desired_yaw_cd)) {
            // Run the normal loiter calculations if sailboat specific calcs are not needed
            calc_heading_and_speed(loiter_radius, _desired_yaw_cd, _desired_speed);
        }

        // update turn rate and heading if sailboat cannot take the direct route to the destination
        g2.sailboat.calc_loiter_turn_rate_and_heading(turn_rate, _desired_yaw_cd);

    } else {
        // Just regular loiter behaviour
        if (_distance_to_destination <= g2.loit_radius.get()) {
            // Bring rover to a stop, observing accel limits
            _desired_speed = attitude_control.get_desired_speed_accel_limited(0.0, rover.G_Dt);
        } else {
            calc_heading_and_speed(g2.loit_radius.get(), _desired_yaw_cd, _desired_speed);
        }
    }

    // run steering and throttle controllers
    calc_steering_to_heading(_desired_yaw_cd, turn_rate);
    calc_throttle(_desired_speed, true);
}

// get desired location
bool ModeLoiter::get_desired_location(Location& destination) const
{
    destination = _destination;
    return true;
}

void ModeLoiter::calc_heading_and_speed(const float limit_radius, float& des_heading_cd, float& des_speed)
{
    // P controller to convert distance to desired speed
    des_speed = MIN((_distance_to_destination - limit_radius) * g2.loiter_speed_gain, g2.wp_nav.get_default_speed());

    des_speed = attitude_control.get_desired_speed_accel_limited(des_speed, rover.G_Dt);

    // calculate bearing to destination
    des_heading_cd = rover.current_loc.get_bearing_to(_destination);
    float yaw_error_cd = wrap_180_cd(des_heading_cd - ahrs.yaw_sensor);
    // if destination is behind vehicle, reverse towards it
    if ((fabsf(yaw_error_cd) > 9000 && g2.loit_type == 0) || g2.loit_type == 2) {
        des_heading_cd = wrap_180_cd(des_heading_cd + 18000);
        yaw_error_cd = wrap_180_cd(des_heading_cd - ahrs.yaw_sensor);
        des_speed = -des_speed;
    }

    // reduce desired speed if yaw_error is large
    // 45deg of error reduces speed to 75%, 90 deg of error reduces speed to 50%
    float yaw_error_ratio = 1.0f - constrain_float(fabsf(yaw_error_cd / 9000.0f), 0.0f, 1.0f) * 0.5f;
    des_speed *= yaw_error_ratio;
}

// In a lazy loiter we want to leave vehicle drift out to the loiter radius, not using throttle or steering actuators to save energy.
// When we reach the loiter radius we reposition back to be within the waypoint radius, then allowing the vehicle to drift again.
bool ModeLoiter::update_lazy_loiter(void)
{

    if (!(g2.loiter_options.get() & uint32_t(Loiter_Options::Lazy_Loiter_Enable))) {
        // we don't want to use a lazy loiter
        return false;
    }

    // Sanity check params to ensure we don't get odd behaviour from an unintended config
    if (g2.wp_nav.get_radius() >= g2.loit_radius.get()) {
        if (lazy_loiter_active) {
            // Send the deactivated message if someone changed a param when lazy loiter was active
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Lazy Loiter: Deactivated");
        }
        lazy_loiter_active = false;
    }

    // Update state and send a message to GCS if we change state
    if (!lazy_loiter_active && _distance_to_destination < g2.wp_nav.get_radius()) {
        // Activate lazy loiter and let the vehicle drift
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Lazy Loiter: Active");
        lazy_loiter_active = true;
    }
    if (lazy_loiter_active && _distance_to_destination > g2.loit_radius.get()) {
        // We have just drifted to the edge of our loiter radius, get the vehicle to motor back the wp location
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Lazy Loiter: Deactivated");
        lazy_loiter_active = false;
    }

    if (lazy_loiter_active) {
        // Don't output on steering and throttle, just let vehicle drift
        g2.motors.set_throttle(0);
        g2.motors.set_steering(0);

        // Keep I-terms relaxed to prevent jumps in output when we come out of a lazy loiter
        // attitude_control.relax_I();
        return true;
    }

    // If we got this far then lazy loiter is not active and we need to move back to be within the wp_radius
    calc_heading_and_speed(g2.wp_nav.get_radius(), _desired_yaw_cd, _desired_speed);

    // run steering and throttle controllers
    const float turn_rate = 0.0; // unlimited
    calc_steering_to_heading(_desired_yaw_cd, turn_rate);
    calc_throttle(_desired_speed, true);

    return true;
}

void ModeLoiter::_exit() {
    if (lazy_loiter_active) {
        // We have just exited loiter whilst in a lazy loiter. Send message to GCS to tell user.
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Lazy Loiter: Deactivated");
        lazy_loiter_active = false;
    }
}
