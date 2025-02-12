#include "AP_Containment_Manager.h"
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

// Constructor
AP_Containment_Manager::AP_Containment_Manager(AC_WPNav *&wpnav) :
_wpnav(wpnav)
{}

// Function for running time scaler calculation and applying to mission time
void AP_Containment_Manager::update(const Location current_loc)
{
    if (_state == State::DISABLE) {
        return;
    }

    AP_Mission *mission = AP::mission();
    if (mission == nullptr) {
        return;
    }

    // We only want to run containment checks if the mission is running
    if (mission->state() != AP_Mission::mission_state::MISSION_RUNNING) {
        return;
    }

    // Rate limit to 5 Hz
    const uint32_t now = AP_HAL::millis();
    if (now - _last_scaler_update_ms < 200) {
        return;
    }

    // Reset state back to the desired state as we may have been in an error state in the last update.
    _state = _desired_sate;

    // Calculate the normal-planer radius from the target vector of travel
    // Get the last WP that we passed.
    const uint16_t prev_wp_index = mission->get_prev_nav_cmd_index();

    if (prev_wp_index == AP_MISSION_CMD_INDEX_NONE || prev_wp_index < AP_MISSION_FIRST_REAL_COMMAND) {
        // we have hit the end of the mission or dont have a mission
        // the calculation below will give nonsense, do not run
        return;
    }

    AP_Mission::Mission_Command prev_nav_cmd;
    if (!mission->get_next_nav_cmd(prev_wp_index, prev_nav_cmd)) {
        _state = State::ERROR;
        return;
    }

    // Extract the location content from the previous 
    Location prev_wp_loc = prev_nav_cmd.content.location;

    // Ensure that all locations are in the same alt frame for comparison
    if (!prev_wp_loc.change_alt_frame(current_loc.get_alt_frame())) {
        // this could fail due missing terrain database alt
        _state = State::ERROR;
        return;
    }

    // Get the WP that we are heading toward.
    const AP_Mission::Mission_Command current_nav_cmd = mission->get_current_nav_cmd();

    // Do not run if we are in a landing, we cannot scale mission time if we are in a landing anyway
    if (mission->is_landing_type_cmd(current_nav_cmd.id)) {
        return;
    }

    Location current_wp_loc = current_nav_cmd.content.location;

    // Ensure that all locations are in the same alt frame for comparison
    if (!current_wp_loc.change_alt_frame(current_loc.get_alt_frame())) {
        // this could fail due missing terrain database alt
        _state = State::ERROR;
        return;
    }

    if (!mission->calc_norm_radius_to_mission_leg(prev_wp_loc, current_wp_loc, current_loc, _radius)) {
        // we may have failed to do the norm radius calculation
        _state = State::ERROR;
        return;
    }

    // Calculate the time scaler that we can apply to the mission target kinematics
    const float DEAD_ZONE = 0.1; // Allow a dead zone in the centre of the target position were we always apply full target speeds and accels
    const float MIN_TIME_SCALER = 0.1; // Never allow the time scaler to go completely to zero. We do not want the copter to get "trapped" in really windy conditions
    const float norm_err = _radius / _max_radius;
    _time_scaler = 1.0 + DEAD_ZONE - norm_err;
    _time_scaler = constrain_float(_time_scaler, MIN_TIME_SCALER, 1.0);

    if (_state == State::ACTIVE) {
        // Apply active control to WP nav
        _wpnav->set_target_time_scaler(_time_scaler);
    }

    if (_state != State::ACTIVE && _last_state == State::ACTIVE) {
        // We have disabled the active control so ensure we reset the time scaler in WPNav
        _wpnav->set_target_time_scaler(1.0);
    }

    _last_state = _state;
    _last_scaler_update_ms = now;
}

void AP_Containment_Manager::reset(void)
{
    _desired_sate = State::DISABLE;
    _state = State::DISABLE;
    _last_state = State::DISABLE;
    _time_scaler = 1.0;
    _wpnav->set_target_time_scaler(_time_scaler);
}

// Function to update state from mission command
void AP_Containment_Manager::update_state(const AP_Mission::Mission_Command& cmd)
{
    _desired_sate = (State) constrain_int16(cmd.content.containment_manager.active, 0, 2);

    _state = _desired_sate;
    _max_radius = cmd.content.containment_manager.radius;

    gcs().send_text(MAV_SEVERITY_INFO, "Containment: %s", get_state_string());
}

const char* AP_Containment_Manager::get_state_string(void) const
{
    switch (_state) {
        case State::DISABLE:
            return "Disabled";
        case State::MONITORING_ONLY:
            return "Monitoring";
        case State::ACTIVE:
            return "Active";
        case State::ERROR:
            return "Error";
        default:
            return "Unknown";
    }
}

void AP_Containment_Manager::update_logging(void)
{
    if (_state == State::DISABLE) {
        return;
    }

    // Rate limit to 5 Hz
    const uint32_t now = AP_HAL::millis();
    if (now - _last_logging_update_ms < 200) {
        return;
    }

    // Check if we have timed-out and the scaler has not been applied recently
    if (now - _last_scaler_update_ms > 500) {
        _state = State::TIMEOUT;
    }

    // @LoggerMessage: CONT
    // @Vehicles: Copter
    // @Description: Containment Manager Information
    // @Field: TimeUS: Time since system startup
    // @Field: Mode: Mode of operation
    // @Field: DesM: Desired mode of operation set by do command
    // @Field: Rad: Measured planer radius from mission leg
    // @Field: MRad: Max acceptable radius set by do command
    // @Field: TS: Time Scaler

    //Write to data flash log
    AP::logger().WriteStreaming("CONT",
            "TimeUS,Mode,DesM,Rad,MRad,TS",
            "s--mm-", // units
            "F--000", // multipliers
            "Qiifff", // format
            AP_HAL::micros64(),
            (uint8_t)_state,
            (uint8_t)_desired_sate,
            _radius,
            _max_radius,
            _time_scaler);

    _last_logging_update_ms = now;
}
