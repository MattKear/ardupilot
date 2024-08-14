#include "AP_GPS_UBLOX_ILM.h"

#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

#define HACC_MIN_DEFAULT 3.0f
#define VACC_MIN_DEFAULT 3.0f
#define HDOP_MAX_DEFAULT 3.0f
#define HDOP_MIN_DEFAULT 0.5f
#define SATELLITES_MAX_DEFAULT 33
#define SATELLITES_MIN_DEFAULT 3
#define CLOCK_ACC_MIN_DEFAULT 1500
#define TARGET_CN0_DEFAULT 40

const AP_Param::GroupInfo AP_GPS_UBLOX_ILM_CONFIG::var_info[] = {

    // @Param: HACC_MIN
    // @DisplayName: Horizontal accuracy minimum
    // @Description: Minimum horizontal accuracy above which the contribution to the ILM score is maximal
    // @Units: m
    // @Range: 0 6
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("HACC_MIN", 1, AP_GPS_UBLOX_ILM_CONFIG, hacc_min, HACC_MIN_DEFAULT),

    // @Param: VACC_MIN
    // @DisplayName: Vertical accuracy minimum
    // @Description: Minimum vertical accuracy above which the contribution to the ILM score is maximal
    // @Units: m
    // @Range: 0 6
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("VACC_MIN", 2, AP_GPS_UBLOX_ILM_CONFIG, vacc_min, VACC_MIN_DEFAULT),

    // @Param: HDOP_MAX
    // @DisplayName: Maximum HDOP
    // @Description: Maximum HDOP above which the contribution to the ILM score is maximal
    // @Units: m
    // @Range: 0 6
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("HDOP_MAX", 3, AP_GPS_UBLOX_ILM_CONFIG, hdop_max, HDOP_MAX_DEFAULT),

    // @Param: HDOP_MIN
    // @DisplayName: Minimum HDOP
    // @Description: Minimum HDOP above which the contribution to the ILM score is minimal
    // @Units: m
    // @Range: 0 6
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("HDOP_MIN", 4, AP_GPS_UBLOX_ILM_CONFIG, hdop_min, HDOP_MIN_DEFAULT),

    // @Param: SATS_MAX
    // @DisplayName: Maximum number of satellites
    // @Description: Maximum number of satellites above which the contribution to the ILM score is maximal
    // @Range: 0 50
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("SATS_MAX", 5, AP_GPS_UBLOX_ILM_CONFIG, satellites_max, SATELLITES_MAX_DEFAULT),

    // @Param: SATS_MIN
    // @DisplayName: Minimum number of satellites
    // @Description: Minimum number of satellites above which the contribution to the ILM score is minimal
    // @Range: 0 50
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("SATS_MIN", 6, AP_GPS_UBLOX_ILM_CONFIG, satellites_min, SATELLITES_MIN_DEFAULT),

    // @Param: CACC_MIN
    // @DisplayName: Clock accuracy minimum
    // @Description: Minimum clock accuracy above which it will contribute to the ILM score
    // @Range: 0 10000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("CACC_MIN", 7, AP_GPS_UBLOX_ILM_CONFIG, clock_acc_min, CLOCK_ACC_MIN_DEFAULT),

    // @Param: TGT_CN0
    // @DisplayName: Target CN0
    // @Description: Target CN0 beneath which it will contribute to the ILM score
    // @Range: 0 50
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("TGT_CN0", 8, AP_GPS_UBLOX_ILM_CONFIG, target_cn0, TARGET_CN0_DEFAULT),

    // @Param: SCRE_MIN
    // @DisplayName: Minimum ILM score
    // @Description: Minimum ILM score beneath which it will generate a pre-arm error. 0 to disable.
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("SCRE_MIN", 9, AP_GPS_UBLOX_ILM_CONFIG, min_ilm_score, 0),

    AP_GROUPEND
};

AP_GPS_UBLOX_ILM_CONFIG::AP_GPS_UBLOX_ILM_CONFIG() {
    AP_Param::setup_object_defaults(this, var_info);
}

AP_GPS_UBLOX_ILM::AP_GPS_UBLOX_ILM(const AP_GPS_UBLOX_ILM_CONFIG &config) :
    _config(config) {}

int16_t AP_GPS_UBLOX_ILM::get_score() const
{
    int16_t score = 0;
    score += hacc_score;
    score += vacc_score;
    score += antenna_score;
    score += jamming_score;
    score += spoofing_score;
    score += yaw_score;
    score += sats_score;
    score += cpu_score;
    score += cn0_score;
    score += dop_score;
    score += fix_score;
    score += corrections_score;
    score += clock_score;
    return score;
}

int8_t AP_GPS_UBLOX_ILM::get_hacc_score() const
{
    return hacc_score;
}

int8_t AP_GPS_UBLOX_ILM::get_vacc_score() const
{
    return vacc_score;
}

int8_t AP_GPS_UBLOX_ILM::get_antenna_score() const
{
    return antenna_score;
}

int8_t AP_GPS_UBLOX_ILM::get_jamming_score() const
{
    return jamming_score;
}

int8_t AP_GPS_UBLOX_ILM::get_spoofing_score() const
{
    return spoofing_score;
}

int8_t AP_GPS_UBLOX_ILM::get_yaw_score() const
{
    return yaw_score;
}

int8_t AP_GPS_UBLOX_ILM::get_sats_score() const
{
    return sats_score;
}

int8_t AP_GPS_UBLOX_ILM::get_cpu_score() const
{
    return cpu_score;
}

int8_t AP_GPS_UBLOX_ILM::get_cn0_score() const
{
    return cn0_score;
}

int8_t AP_GPS_UBLOX_ILM::get_dop_score() const
{
    return dop_score;
}

int8_t AP_GPS_UBLOX_ILM::get_fix_score() const
{
    return fix_score;
}

int8_t AP_GPS_UBLOX_ILM::get_corrections_score() const
{
    return corrections_score;
}

int8_t AP_GPS_UBLOX_ILM::get_clock_score() const
{
    return clock_score;
}

void AP_GPS_UBLOX_ILM::update_antenna_status(ubx_ant_status &ant_status)
{
    bool is_antenna_connected{false};

    switch (ant_status) {      
        case ubx_ant_status::ASTATUS_INIT:
        case ubx_ant_status::ASTATUS_SHORT:
        case ubx_ant_status::ASTATUS_OPEN:
            is_antenna_connected = false;
            break;
        case ubx_ant_status::ASTATUS_DONTKNOW:  //Default if system disabled
        case ubx_ant_status::ASTATUS_OK:
        default:
            is_antenna_connected = true;
            break;
    }

    if (is_antenna_connected) {
        antenna_score = ILM_SCORE_OK;
    } else {
        antenna_score = ILM_SCORE_AWFUL;
    }
}

void AP_GPS_UBLOX_ILM::update_cpu_load(uint8_t cpu_load_percent)
{
    static constexpr uint8_t cpu_load_max = 100;

    if (cpu_load_percent <= _cpu_load_expected) {
        cpu_score = ILM_SCORE_OK;
        return;
    }

    if (cpu_load_percent >= cpu_load_max) {
        cpu_score = ILM_SCORE_BAD;
        return;
    }

    // calculate how far above the expected load we are
    int8_t cpu_percent_above_expected = cpu_load_percent - _cpu_load_expected;
    constrain_int8(cpu_percent_above_expected, 0, (int8_t)(cpu_load_max - _cpu_load_expected));
    cpu_score = ILM_SCORE_OK - cpu_percent_above_expected;
}

void AP_GPS_UBLOX_ILM::update_jamming_status(ubx_jam_state jamming_status)
{
    switch (jamming_status) {
        case ubx_jam_state::JAM_STATE_NONE:
            jamming_score = ILM_SCORE_OK;
            break;
        case ubx_jam_state::JAM_STATE_WARNING:
            jamming_score = ILM_SCORE_BAD;
            break;
        case ubx_jam_state::JAM_STATE_UNKNOWN:
        default:
            jamming_score = ILM_SCORE_DEGRADED;
            break;
    }
}

void AP_GPS_UBLOX_ILM::update_spoofing_status(ubx_spoof_state spoofing_status)
{
    //TODO(cb): when reviewing the next itteration of the ILM
    //          figure out a way to handle these magic numbers
    switch (spoofing_status) {
        case ubx_spoof_state::SPOOF_STATE_INDICATED:
            spoofing_score = -5;
            break;
        case ubx_spoof_state::SPOOF_STATE_AFFIRMED:
            spoofing_score = -15;
            break;
        case ubx_spoof_state::SPOOF_STATE_UNKNOWN:
        case ubx_spoof_state::SPOOF_STATE_NONE:
        default:
            spoofing_score = 0;
            break;
    }
}

void AP_GPS_UBLOX_ILM::update_has_yaw(bool has_yaw)
{
    yaw_score = has_yaw ? ILM_SCORE_OK : ILM_SCORE_BAD;
}

void AP_GPS_UBLOX_ILM::update_fix_status(ubx_nav_fix_type fix_status)
{
    switch (fix_status) {
        case ubx_nav_fix_type::FIX_3D:
            fix_score = ILM_SCORE_OK;
            break;
        case ubx_nav_fix_type::FIX_2D:
            fix_score = ILM_SCORE_ERROR;
            break;
        case ubx_nav_fix_type::FIX_NONE:
        case ubx_nav_fix_type::FIX_DEAD_RECKONING:
        case ubx_nav_fix_type::FIX_GPS_DEAD_RECKONING:
        case ubx_nav_fix_type::FIX_TIME:
        default:
            fix_score = ILM_SCORE_AWFUL;
            break;
    }
}

void AP_GPS_UBLOX_ILM::update_corrections_status(ubx_nav_fix_flags corrections_status)
{
    switch (corrections_status) {
        case ubx_nav_fix_flags::FIX_FLAGS_DGPS:
            corrections_score = ILM_SCORE_DEGRADED;
            break;
        case ubx_nav_fix_flags::FIX_FLAGS_RTK_FLOAT:
            corrections_score = ILM_SCORE_MEH;
            break;
        case ubx_nav_fix_flags::FIX_FLAGS_RTK_FIXED:
            corrections_score = ILM_SCORE_EMPHASIS;
            break;
        default:
            corrections_score = 0;
            break;
    }
}

void AP_GPS_UBLOX_ILM::update_hacc(float hacc)
{
    float score = (_config.hacc_min - hacc) * 3.0f;
    constrain_float(score, ILM_SCORE_ERROR, ILM_SCORE_EMPHASIS);
    hacc_score = static_cast<int8_t>(score);
}

void AP_GPS_UBLOX_ILM::update_vacc(float vacc)
{
    float score = (_config.vacc_min - vacc) * 3.0f;
    constrain_float(score, ILM_SCORE_ERROR, ILM_SCORE_EMPHASIS);
    vacc_score = static_cast<int8_t>(score);
}

void AP_GPS_UBLOX_ILM::update_hdop(uint16_t hdop)
{
    if (hdop <= _config.hdop_min) {
        dop_score = ILM_SCORE_OK;
        return;
    }

    if (hdop >= _config.hdop_max) {
        dop_score = ILM_SCORE_POOR;
        return;
    }

    // hdop max must be greater than hdop min (prevent divide by zero)
    if (_config.hdop_max <= _config.hdop_min) {
        return;
    }

    float percent_above_min = (hdop - _config.hdop_min) / (_config.hdop_max - _config.hdop_min);
    constrain_float(percent_above_min, 0.0f, 1.0f);

    // scale the score from ILM_SCORE_OK to ILM_SCORE_POOR
    dop_score = ILM_SCORE_OK - static_cast<int8_t>(percent_above_min * (ILM_SCORE_OK - ILM_SCORE_POOR));
}

void AP_GPS_UBLOX_ILM::update_number_of_tracked_satellites(uint8_t sats)
{
    if (sats >= _config.satellites_max) {
        sats_score = ILM_SCORE_OK;
        return;
    }

    // satellites max must be greater than satellites min (prevent divide by zero)
    if (_config.satellites_max <= _config.satellites_min) {
        return;
    }

    float percent_above_min = (sats - _config.satellites_min) / (_config.satellites_max - _config.satellites_min);
    constrain_float(percent_above_min, 0.0f, 1.0f);

    // scale the score from ILM_SCORE_POOR to ILM_SCORE_OK
    sats_score = ILM_SCORE_POOR + static_cast<int8_t>(percent_above_min * (ILM_SCORE_OK - ILM_SCORE_POOR));
}

void AP_GPS_UBLOX_ILM::update_clock_accuracy(uint32_t clock_accuracy)
{
    if (clock_accuracy <= static_cast<unsigned int>(_config.clock_acc_min)) {
        clock_score = ILM_SCORE_OK;
        return;
    }

    clock_score = ILM_SCORE_ERROR;
}

void AP_GPS_UBLOX_ILM::update_cn0(uint8_t cn0)
{
    // prevent divide by zero
    if (_config.target_cn0 == 0) {
        return;
    }

    float percent_above_target = (cn0 - _config.target_cn0) / (_config.target_cn0);

    // scale the score from ILM_SCORE_BAD to ILM_SCORE_OK
    float score = static_cast<float>(ILM_SCORE_POOR) + percent_above_target * static_cast<float>(ILM_SCORE_OK - ILM_SCORE_POOR);
    constrain_float(score, ILM_SCORE_BAD, ILM_SCORE_OK);

    cn0_score = static_cast<int8_t>(score);
}