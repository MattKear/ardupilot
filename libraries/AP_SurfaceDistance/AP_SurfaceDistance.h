#pragma once

#include <AP_Math/AP_Math.h>
#include <Filter/LowPassFilter.h>
#include <AP_InertialNav/AP_InertialNav.h>
#include <AP_HAL/Semaphores.h>
#include <AP_RangeFinder/AP_RangeFinder.h>

class AP_SurfaceDistance {
public:
    AP_SurfaceDistance(Rotation rot, const AP_InertialNav& inav, uint8_t i) : rotation(rot), inertial_nav(inav), instance(i) {};

    void update();

    // check if the last healthy range finder reading is too old to be considered valid
    bool data_stale(void);

    // helper to check that rangefinder was last reported as enabled and healthy
    bool enabled_and_healthy(void) const;

    // get inertially interpolated rangefinder height
    bool get_rangefinder_height_interpolated_cm(int32_t& ret);

    // return hagl measurement, using numerous sources to fail over to, depending on source health. Priority: 1) inertially interpolated rangefinder 2) terrain 3) height above home
    bool get_height_above_ground(float& hgt);

    bool enabled;                          // not to be confused with rangefinder enabled, this state is to be set by the vehicle.
    bool alt_healthy;                      // true if we can trust the altitude from the rangefinder
    int16_t alt_cm;                        // tilt compensated altitude (in cm) from rangefinder
    float inertial_alt_cm;                 // inertial alt at time of last rangefinder sample
    LowPassFilterFloat alt_cm_filt {0.5};  // altitude filter
    int16_t alt_cm_glitch_protected;       // last glitch protected altitude
    int8_t glitch_count;                   // non-zero number indicates rangefinder is glitching
    uint32_t glitch_cleared_ms;            // system time glitch cleared
    float terrain_offset_cm;               // filtered terrain offset (e.g. terrain's height above EKF origin)

private:
#if HAL_LOGGING_ENABLED
    void Log_Write(void) const;
#endif

    // multi-thread access support
    HAL_Semaphore sem;

    const uint8_t instance;
    uint8_t status;
    uint32_t last_healthy_ms;
    RangeFinder::Status last_rangefinder_status;  // keep track of the last received rangefinder state
    int32_t last_interp_rngfind_hagl;             // last recorded hagl measurement from interpolated rangefinder measurement, used for bumpless transfer to terrain database
    bool have_received_rangefinder;               // flag to know if we should apply bumpless transfer between rangefinder and terrain database

    const AP_InertialNav& inertial_nav;
    const Rotation rotation;
};
