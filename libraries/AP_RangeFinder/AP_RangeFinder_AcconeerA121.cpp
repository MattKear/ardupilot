/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "AP_RangeFinder_AcconeerA121.h"

#if AP_RANGEFINDER_A121_RADAR_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

AP_RangeFinder_AcconeerA121::AP_RangeFinder_AcconeerA121(RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params,
         AP_HAL::I2CDevice *dev_ptr)
    : AP_RangeFinder_Backend(_state, _params)
    , dev(dev_ptr)
    {}


AP_RangeFinder_Backend *AP_RangeFinder_AcconeerA121::detect(RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params,
        AP_HAL::I2CDevice *dev_ptr)
{
    if (!dev_ptr) {
        return nullptr;
    }

    AP_RangeFinder_AcconeerA121 *sensor = NEW_NOTHROW AP_RangeFinder_AcconeerA121(_state, _params, dev_ptr);

    if (!sensor) {
        return nullptr;
    }

    sensor->init();
    return sensor;
}

void AP_RangeFinder_AcconeerA121::init()
{
    // Init setup state
    setup_stage = SetupStage::NEEDS_RESET;

    dev->set_retries(1);

    // register for update of sensor state in I2C thread
    dev->register_periodic_callback(CALLBACK_TIME_US, FUNCTOR_BIND_MEMBER(&AP_RangeFinder_AcconeerA121::timer, void));

    filtered_distance_mm.set_cutoff_frequency(params.xm125_lpf_cutoff_hz.get());
}


// update the state of the sensor
void AP_RangeFinder_AcconeerA121::update(void)
{
    // Prevent the race conditions when fetching the distance_measurement_mm state
    dev->get_semaphore()->take_blocking();

    uint32_t now = AP_HAL::millis();

    // Update the rangefinder state
    state.distance_m = filtered_distance_mm.get() * 1e-3;
    state.last_reading_ms = last_update_ms;

    // Update rangefinder status
    if (setup_stage == SetupStage::NEEDS_RESET) {
        // setup has not advanced so we assume that the device is not connected
        state.status = RangeFinder::Status::NotConnected;

    } else if ((health != 0) || (setup_stage != SetupStage::COMPLETE)) {
        // XM125 health is marked as unhealthy
        state.status = RangeFinder::Status::NoData;

    } else {
        state.status = RangeFinder::Status::Good;
    }

    dev->get_semaphore()->give();

    // Send rate limited debug messages if param is enabled
    if ((params.xm125_debug.get() != 0) && (now - last_debug_print_ms > 1000)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "XM125: Health:%i NumDist:%i D0=%i Setup=%u T=%i", health, num_distances, dist_measurement_mm[0], uint8_t(setup_stage), temp_deg_c);
        last_debug_print_ms = now;
    }
}

// update the state of the sensor
void AP_RangeFinder_AcconeerA121::timer(void)
{
    dev->get_semaphore()->take_blocking();

    if (setup_stage != SetupStage::COMPLETE) {
        setup_radar();
        dev->get_semaphore()->give();
        return;
    }

    // Get the latest data from the radar
    update_measurement();

    dev->get_semaphore()->give();
}

// Setup the radar - Progress through states in a switch case tree to setup the device
void AP_RangeFinder_AcconeerA121::setup_radar(void)
{
    // First thing we always need to do is reset the device to ensure we can apply a config
    if (setup_stage == SetupStage::NEEDS_RESET && send_command(Command::RESET_MODULE)) {
        setup_stage = SetupStage::CONFIRMING_RESET;
        reset_time_ms = AP_HAL::millis();
        return;
    }

    // Update status
    bool received_status = update_detector_status();

    // Always check if device is busy before trying to write to a register
    if (!received_status || is_busy()) {
        // If device is stuck in busy state then the reset did not work and we should try again by regressing the state
        if ((setup_stage == SetupStage::CONFIRMING_RESET) && (AP_HAL::millis() - reset_time_ms > 2000)) {
            setup_stage = SetupStage::NEEDS_RESET;
        }
        return;
    }

    if (params.xm125_debug.get() != 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "XM125: Setup: %i", uint8_t(setup_stage));
    }

    switch (setup_stage) {
        case SetupStage::CONFIRMING_RESET:
            // If we got this far then the reset was successful, advance the state
            setup_stage = SetupStage::CHECK_ERRORS;
            FALLTHROUGH;

        case SetupStage::CHECK_ERRORS:
            if (!has_error()) {
                setup_stage = SetupStage::SET_START;
            }
            break;

        case SetupStage::SET_START:
            // Set closest distance for measurement range
            write_register(Register::START, uint32_t(params.min_distance.get()*1000.0));
            setup_stage = SetupStage::SET_END;
            break;

        case SetupStage::SET_END:
            // Set furthest distance for measurement range
            write_register(Register::END, uint32_t(params.max_distance.get()*1000.0));
            setup_stage = SetupStage::SET_PROFILE;
            break;

        case SetupStage::SET_PROFILE:
            // Set profile, which configures a group of settings in the device 
            write_register(Register::MAX_PROFILE, params.xm125_profile.get());
            setup_stage = SetupStage::SET_REFLECTOR_SHAPE;
            break;

        case SetupStage::SET_REFLECTOR_SHAPE:
            // Set profile, which configures a group of settings in the device 
            write_register(Register::REFLECTOR_SHAPE, params.xm125_shape.get());
            setup_stage = SetupStage::SET_SIGNAL_QUALITY;
            break;

        case SetupStage::SET_SIGNAL_QUALITY:
            // Set profile, which configures a group of settings in the device 
            write_register(Register::SIGNAL_QUALITY, params.xm125_signal_quality.get());
            setup_stage = SetupStage::SET_THRESHOLD_METHOD;
            break;

        case SetupStage::SET_THRESHOLD_METHOD:
            // Set profile, which configures a group of settings in the device 
            write_register(Register::THRESHOLD_METHOD, params.xm125_threshold_method.get());
            setup_stage = SetupStage::SET_NUM_FRAMES_THRESHOLD;
            break;

        case SetupStage::SET_NUM_FRAMES_THRESHOLD:
            // Set profile, which configures a group of settings in the device 
            write_register(Register::NUM_FRAMES_RECORDED_THRESHOLD, params.xm125_num_frames.get());
            setup_stage = SetupStage::ENABLE_CLOSE_RANGE_LEAKAGE;
            break;

        case SetupStage::ENABLE_CLOSE_RANGE_LEAKAGE:
           // Set profile, which configures a group of settings in the device 
           write_register(Register::CLOSE_RANGE_LEAKAGE_CANCELLATION, params.xm125_close_range_leakage.get());
           setup_stage = SetupStage::APPLY_AND_CAL;
           break;

        case SetupStage::APPLY_AND_CAL:
            // Apply all settings and run initial calibration
            send_command(Command::APPLY_CONFIG_AND_CALIBRATE);
            setup_stage = SetupStage::CHECK_CONFIG;
            break;

        case SetupStage::CHECK_CONFIG:
            if (received_status && !has_error() && config_ok()) {
                send_command(Command::MEASURE_DISTANCE);
                setup_stage = SetupStage::COMPLETE;
            }
            break;

        default:
            return;
    }

}

void AP_RangeFinder_AcconeerA121::update_measurement(void)
{
    // Reset health
    health = 0;
    dist_measurement_mm[0] = 0;
    dist_measurement_mm[1] = 0;
    dist_measurement_mm[2] = 0;

    uint32_t now = AP_HAL::millis();

    // update time out health bit
    if (now - last_update_ms > 500) {
        health |= uint8_t(Health::MEASUREMENT_TIMEOUT);
    }

    // Update errors
    if (!update_detector_status()) {
        // Don't proceed if we can't get the detector status
        health |= uint8_t(Health::FAILED_DEVICE_COMS);
        return;
    }

    // Check if device is busy
    if (is_busy()) {
        return;
    }

    if (!config_ok()) {
        // Don't proceed if we have a bad device config
        health |= uint8_t(Health::BAD_CONFIG);
        setup_stage = SetupStage::NEEDS_RESET;
        return;
    }

    if (has_error()) {
        // Don't proceed if we have an error
        health |= uint8_t(Health::DEVICE_ERROR);
        setup_stage = SetupStage::NEEDS_RESET;
        return;
    }

    // get the distance result bitmask
    uint32_t distance_result;
    if (!read_register(Register::DISTANCE_RESULT, distance_result)) {
        // Try again next loop
        health |= uint8_t(Health::FAILED_DEVICE_COMS);
        send_command(Command::MEASURE_DISTANCE);
        return;
    }

    // Check for errors in distance result
    if ((distance_result & uint32_t(DistanceResult::DISTANCE_RESULT_MEASUREMENT_ERROR)) != 0) {
        health |= uint8_t(Health::MEASUREMENT_ERROR);
        send_command(Command::MEASURE_DISTANCE);
        return;
    }

    // Check if the calibration error flag is set
    if ((distance_result & uint32_t(DistanceResult::DISTANCE_RESULT_CALIBRATION_NEEDED)) != 0) {
        // send the recalibration command
        send_command(Command::RECALIBRATE);
        health |= uint8_t(Health::RECALIBRATION_REQUIRED);
        return;
    }

    // Note: DistanceResult::DISTANCE_RESULT_NEAR_START_EDGE this is not an error but a warning that there is potentially a result closer than the minimum configured measurement range 

    // Update the temperature measurement
    if (params.xm125_debug.get() != 0) {
        temp_deg_c = uint16_t((distance_result & uint32_t(DistanceResult::TEMPERATURE_MASK)) >> TEMPERATURE_SHIFT);
    }

    // Loop over the number of returns to get the distances
    num_distances = uint8_t(distance_result) & uint8_t(DistanceResult::NUM_DISTANCE_MASK);

    // If zero distances have been reported then request a new measurement and comeback later
    if (num_distances == 0) {
        send_command(Command::MEASURE_DISTANCE);
        return;
    }

    for (uint8_t i=0; i<MIN(MAX_PEAKS,num_distances); i++) {
        // Get the ith distance from the device
        uint32_t distance_mm;
        if (!read_register(Register(dist_reg[i]), distance_mm)) {
            // If we got here then we failed to read the register, mark unhealthy and move on
            health |= uint8_t(Health::FAILED_DEVICE_COMS);
            continue;
        }

        // If we got this far then we can store the measurement for later processing
        dist_measurement_mm[i] = distance_mm;
    }


    // Check for long valid measurement times and reset the filter as appropriate
    const float dt = (now - last_update_ms) * 1.0e-3;

    // Note: Always using the strongest return, which is the first index

    // Cut off is set to zero so keep reseting the filter to effectively disable it
    if (!is_positive(params.xm125_lpf_cutoff_hz.get())) {
        filtered_distance_mm.reset(float(dist_measurement_mm[0]));

    } else if ((health & uint8_t(Health::MEASUREMENT_TIMEOUT)) != 0) {
        // reset filter to current measurement if we have had a time out
        filtered_distance_mm.reset(float(dist_measurement_mm[0]));

    } else {
        // Apply low pass filter at the higher call back rate.
        // Just report the strongest return, which is the first value.
        filtered_distance_mm.apply(float(dist_measurement_mm[0]), dt);
    }

    // Update the measurement timer
    last_update_ms = now;

    // Send the measurement command to the sensor and hope that it has completed when we come back for the next update
    send_command(Command::MEASURE_DISTANCE);
}

// Check if all of the config bits are true
bool AP_RangeFinder_AcconeerA121::config_ok(void)
{
    // Config is only ok if all bits are true
    return (detector_status & CONFIG_OK_MASK) == CONFIG_OK_MASK;
}

// Return the config bitmask
uint32_t AP_RangeFinder_AcconeerA121::get_config_mask(void)
{
    return detector_status & CONFIG_OK_MASK;
}

// Check if we have received an error code
bool AP_RangeFinder_AcconeerA121::has_error(void)
{
    return (detector_status & ALL_ERRORS_MASK) != 0;
}

// Return the full error bit mask
uint32_t AP_RangeFinder_AcconeerA121::get_error()
{
    return detector_status & ALL_ERRORS_MASK;
}

// Check if device is busy
bool AP_RangeFinder_AcconeerA121::is_busy()
{
    return (detector_status & uint32_t(DetectorStatus::BUSY)) != 0;
}

// Get detector status from device and update local state
// Note that this register contains config, error, and busy bits
bool AP_RangeFinder_AcconeerA121::update_detector_status(void)
{
    uint32_t regVal;
    if (!read_register(Register::DETECTOR_STATUS, regVal)) {
        return false;
    }

    detector_status = regVal;
    return true;
}

bool AP_RangeFinder_AcconeerA121::send_command(Command cmd)
{
    uint32_t command = uint32_t(cmd);
    return write_register(Register::COMMAND, command);
}

// Helper for writting to registers
bool AP_RangeFinder_AcconeerA121::write_register(Register reg, uint32_t data)
{
    if (!dev) {
        return false;
    }

    dev->get_semaphore()->take_blocking();

    const uint16_t regAddress = uint16_t(reg);
    uint8_t payload[6];

    // Register address (2 bytes, MSB first)
    payload[0] = uint8_t((regAddress >> 8) & 0xFF); // Address [15:8]
    payload[1] = uint8_t(regAddress & 0xFF);        // Address [7:0]

    // Data (4 bytes, MSB first)
    payload[2] = uint8_t((data >> 24) & 0xFF); // Data [31:24]
    payload[3] = uint8_t((data >> 16) & 0xFF); // Data [23:16]
    payload[4] = uint8_t((data >> 8) & 0xFF);  // Data [15:8]
    payload[5] = uint8_t(data & 0xFF);         // Data [7:0]

    bool result = dev->transfer(payload, sizeof(payload), nullptr, 0);

    dev->get_semaphore()->give();

    return result;
}

// Helper for writting to registers
bool AP_RangeFinder_AcconeerA121::read_register(Register reg, uint32_t& data)
{
    if (!dev) {
        return false;
    }

    // reset data variable
    data = 0;

    // Register address (2 bytes, MSB first)
    uint8_t reg_add[2];
    reg_add[0] = (uint16_t(reg) >> 8) & 0xFF; // Address [15:8]
    reg_add[1] = (uint16_t(reg) & 0xFF);      // Address [7:0]

    dev->get_semaphore()->take_blocking();

    // read data from register
    uint8_t buf[4] = {};
    if (!dev->transfer(reg_add, sizeof(reg_add), buf, sizeof(buf))) {
        dev->get_semaphore()->give();
        return false;
    }

    dev->get_semaphore()->give();

    // Unpack the data from the buffer into a uint32_t
    data |= uint32_t(buf[0]) << 24; // Data [31:24]
    data |= uint32_t(buf[1]) << 16; // Data [23:16]
    data |= uint32_t(buf[2]) << 8;  // Data [15:8]
    data |= uint32_t(buf[3]);       // Data [7:0]
    return true;
}

#endif  // AP_RANGEFINDER_A121_RADAR_ENABLED
