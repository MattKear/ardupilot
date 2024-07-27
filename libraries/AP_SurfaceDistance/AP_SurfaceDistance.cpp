#include "AP_SurfaceDistance.h"

#ifndef RANGEFINDER_TIMEOUT_MS
 # define RANGEFINDER_TIMEOUT_MS 1000        // rangefinder filter reset if no updates from sensor in 1 second
#endif

#if AP_RANGEFINDER_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>

#ifndef RANGEFINDER_TILT_CORRECTION         // by disable tilt correction for use of range finder data by EKF
 # define RANGEFINDER_TILT_CORRECTION 1
#endif

#ifndef RANGEFINDER_GLITCH_NUM_SAMPLES
 # define RANGEFINDER_GLITCH_NUM_SAMPLES  3   // number of rangefinder glitches in a row to take new reading
#endif

#ifndef RANGEFINDER_GLITCH_ALT_CM
 # define RANGEFINDER_GLITCH_ALT_CM  200      // amount of rangefinder change to be considered a glitch
#endif

#ifndef RANGEFINDER_HEALTH_MIN
 # define RANGEFINDER_HEALTH_MIN 3          // number of good reads that indicates a healthy rangefinder
#endif

void AP_SurfaceDistance::update()
{
    WITH_SEMAPHORE(sem);

    const RangeFinder *rangefinder = RangeFinder::get_singleton();
    if (rangefinder == nullptr) {
        alt_healthy = false;
        return;
    }

#if RANGEFINDER_TILT_CORRECTION == 1
    const float tilt_correction = MAX(0.707f, AP::ahrs().get_rotation_body_to_ned().c.z);
#else
    const float tilt_correction = 1.0f;
#endif

    const uint32_t now = AP_HAL::millis();

    // assemble bitmask assistance, definition is used to generate log documentation
    enum class Surface_Distance_Status : uint8_t {
        Enabled         = 1U<<0, // true if rangefinder has been set to enable by vehicle
        Unhealthy       = 1U<<1, // true if rangefinder is considered unhealthy
        Stale_Data      = 1U<<2, // true if the last healthy rangefinder reading is no longer valid
        Glitch_Detected = 1U<<3, // true if a measurement glitch detected
    };

    // reset status and add to the bitmask as we progress through the update
    status = 0;

    // update enabled status
    if (enabled) {
        status |= (uint8_t)Surface_Distance_Status::Enabled;
    }

    last_rangefinder_status = rangefinder->status_orient(rotation);

    // update health
    alt_healthy = (last_rangefinder_status == RangeFinder::Status::Good) &&
                            (rangefinder->range_valid_count_orient(rotation) >= RANGEFINDER_HEALTH_MIN);
    if (!alt_healthy) {
        status |= (uint8_t)Surface_Distance_Status::Unhealthy;
    }

    // tilt corrected but unfiltered, not glitch protected alt
    alt_cm = tilt_correction * rangefinder->distance_cm_orient(rotation);

    // remember inertial alt to allow us to interpolate rangefinder from the last healthy measurement
    if (alt_healthy) {
        inertial_alt_cm = inertial_nav.get_position_z_up_cm();
    }

    // glitch handling.  rangefinder readings more than RANGEFINDER_GLITCH_ALT_CM from the last good reading
    // are considered a glitch and glitch_count becomes non-zero
    // glitches clear after RANGEFINDER_GLITCH_NUM_SAMPLES samples in a row.
    // glitch_cleared_ms is set so surface tracking (or other consumers) can trigger a target reset
    const int32_t glitch_cm = alt_cm - alt_cm_glitch_protected;
    bool reset_terrain_offset = false;
    if (glitch_cm >= RANGEFINDER_GLITCH_ALT_CM) {
        glitch_count = MAX(glitch_count+1, 1);
        status |= (uint8_t)Surface_Distance_Status::Glitch_Detected;
    } else if (glitch_cm <= -RANGEFINDER_GLITCH_ALT_CM) {
        glitch_count = MIN(glitch_count-1, -1);
        status |= (uint8_t)Surface_Distance_Status::Glitch_Detected;
    } else {
        glitch_count = 0;
        alt_cm_glitch_protected = alt_cm;
    }
    if (abs(glitch_count) >= RANGEFINDER_GLITCH_NUM_SAMPLES) {
        // clear glitch and record time so consumers (i.e. surface tracking) can reset their target altitudes
        glitch_count = 0;
        alt_cm_glitch_protected = alt_cm;
        glitch_cleared_ms = now;
        reset_terrain_offset = true;
    }

    // filter rangefinder altitude
    const bool timed_out = now - last_healthy_ms > RANGEFINDER_TIMEOUT_MS;
    if (alt_healthy) {
        if (timed_out) {
            // reset filter if we haven't used it within the last second
            alt_cm_filt.reset(alt_cm);
            reset_terrain_offset = true;
            status |= (uint8_t)Surface_Distance_Status::Stale_Data;
        } else {
            // TODO: When we apply this library in plane we will need to be able to set the filter freq
            alt_cm_filt.apply(alt_cm, 0.05);
        }
        last_healthy_ms = now;
    }

    // handle reset of terrain offset
    if (reset_terrain_offset) {
        if (rotation == ROTATION_PITCH_90) {
            // upward facing
            terrain_offset_cm = inertial_alt_cm + alt_cm;
        } else {
            // assume downward facing
            terrain_offset_cm = inertial_alt_cm - alt_cm;
        }
    }
#if HAL_LOGGING_ENABLED
    Log_Write();
#endif
}

/*
  get inertially interpolated rangefinder height. Inertial height is
  recorded whenever we update the rangefinder height, then we use the
  difference between the inertial height at that time and the current
  inertial height to give us interpolation of height from rangefinder
 */
bool AP_SurfaceDistance::get_rangefinder_height_interpolated_cm(int32_t& ret)
{
    WITH_SEMAPHORE(sem);

    // We are allowing extrapolation from the last good rangefinder reading
    // if it reports out of range high or out of range low
    if (!enabled ||
        last_rangefinder_status == RangeFinder::Status::NotConnected ||
        last_rangefinder_status == RangeFinder::Status::NoData) {
        return false;
    }

    ret = alt_cm_filt.get();
    ret += inertial_nav.get_position_z_up_cm() - inertial_alt_cm;
    return true;
}

/*
  return hagl measurement, using numerous sources to fail over to,
  depending on source health. Priority: 1) inertially interpolated
  rangefinder 2) terrain 3) height above home.
  if we have previously received a healthy range finder measurement
  we uses an offset for bumpless transfer to terrain database
*/
bool AP_SurfaceDistance::get_height_above_ground(float& hgt)
{
    // function only valid for instances associated with downward facing rangefinders
    if (rotation != ROTATION_PITCH_270) {
        return false;
    }

    int32_t height_above_ground_cm;
    if (get_rangefinder_height_interpolated_cm(height_above_ground_cm)) {
        last_interp_rngfind_hagl = height_above_ground_cm;
        hgt = float(height_above_ground_cm) * 0.01;

        // Remember that we have received a healthy rangefinder reading
        if (enabled_and_healthy()) {
            have_received_rangefinder = true;
        }

        return true;
    }

    Location current_loc;
    if (!AP::ahrs().get_location(current_loc)) {
        // the remaining methods rely on having a location, exit early if we did not get one
        return false;
    }

    if (current_loc.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, height_above_ground_cm)) {

        // apply offset for bumpless transfer if we have previously received a healthy rangefinder measurement
        int32_t rangefinder_terrain_offset_cm = 0;
        if (have_received_rangefinder) {
            rangefinder_terrain_offset_cm = last_interp_rngfind_hagl - height_above_ground_cm;
        }

        hgt = float(height_above_ground_cm + rangefinder_terrain_offset_cm) * 0.01;
        return true;
    }

    // assume the Earth is flat and get height above home
    if (current_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, height_above_ground_cm)) {
        hgt = float(height_above_ground_cm) * 0.01;
        return true;
    }

    // We should not get this far, something is very wrong if we do
    return false;
}


#if HAL_LOGGING_ENABLED
void AP_SurfaceDistance::Log_Write(void) const
{
    // @LoggerMessage: SURF
    // @Vehicles: Copter
    // @Description: Surface distance measurement
    // @Field: TimeUS: Time since system startup
    // @Field: I: Instance
    // @FieldBitmaskEnum: St: Surface_Distance_Status
    // @Field: D: Raw Distance
    // @Field: FD: Filtered Distance
    // @Field: TO: Terrain Offset

    //Write to data flash log
    AP::logger().WriteStreaming("SURF",
                                "TimeUS,I,St,D,FD,TO",
                                "s#-mmm",
                                "F--000",
                                "QBBfff",
                                AP_HAL::micros64(),
                                instance,
                                status,
                                (float)alt_cm * 0.01,
                                (float)alt_cm_filt.get() * 0.01,
                                (float)terrain_offset_cm * 0.01
                                );
}
#endif  // HAL_LOGGING_ENABLED

#endif // AP_RANGEFINDER_ENABLED

bool AP_SurfaceDistance::data_stale(void)
{
    WITH_SEMAPHORE(sem);
    return (AP_HAL::millis() - last_healthy_ms) > RANGEFINDER_TIMEOUT_MS;
}

bool AP_SurfaceDistance::enabled_and_healthy(void) const
{
    return enabled && alt_healthy;
}
