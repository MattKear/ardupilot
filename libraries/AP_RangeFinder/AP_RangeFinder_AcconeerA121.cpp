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

#define INSTALLED_OFFSET 43

// Settings to be applied 
#define MY_XM125_RANGE_START 43 // (mm)
#define MY_XM125_RANGE_END 250 // (mm)
#define MY_XM125_PROFILE 2

#define MY_XM125_GENERIC_SHAPE 1
#define MY_XM125_PLANER_SHAPE 2

#define THRESHOLD_METHOD_CFAR 3

#define MY_XM125_SIGNAL_QUALITY 15000 // (Default = 15000) High signal quality results in a better SNR (because of higher HWAAS and longer measurement time) and higher power consumption

#define MY_XM125_NUM_FRAMES 300 // (Default = 100) The number frames to use for recorded threshold, This filters out: random noise peaks, one-off reflections, transient multipath artifacts

#define XM125_ENABLE_CLOSE_LEAKAGE_CALIBRATION 0

extern const AP_HAL::HAL& hal;

AP_RangeFinder_AcconeerA121::AP_RangeFinder_AcconeerA121(RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params,
         AP_HAL::I2CDevice &_dev)
    : AP_RangeFinder_Backend(_state, _params)
    , dev(_dev)
    {}

/*
   Detects if a Lightware rangefinder is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
AP_RangeFinder_Backend *AP_RangeFinder_AcconeerA121::detect(RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params,
        AP_HAL::I2CDevice *dev_ptr)
{
    if (!dev_ptr) {
        return nullptr;
    }

    AP_RangeFinder_AcconeerA121 *sensor = NEW_NOTHROW AP_RangeFinder_AcconeerA121(_state, _params, *dev_ptr);

    if (!sensor) {
        return nullptr;
    }

    WITH_SEMAPHORE(sensor->dev.get_semaphore());
    sensor->init();
    return sensor;
}

bool AP_RangeFinder_AcconeerA121::init()
{
    // Init setup state
    setup_stage = SetupStage::NEEDS_RESET;

    return true;
}


// update the state of the sensor
void AP_RangeFinder_AcconeerA121::update(void)
{
    if (setup_stage != SetupStage::COMPLETE) {
        setup_radar();
        return;
    }

    // Get the latest data from the radar
    update_measurement();

    // Update the rangefinder with the strongest return
    state.distance_m = dist_measurement_mm[0];
    state.last_reading_ms = last_update_ms;


    // update_logging();
}



// Setup the radar - Progress through states in a switch case tree to setup the device
void AP_RangeFinder_AcconeerA121::setup_radar(void)
{
    // First thing we always need to do is reset the device to ensure we can apply a config
    if (setup_stage == SetupStage::NEEDS_RESET) {
        send_command(Command::RESET_MODULE);
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
            write_register(Register::START, MY_XM125_RANGE_START);
            setup_stage = SetupStage::SET_END;
            break;

        case SetupStage::SET_END:
            // Set furthest distance for measurement range
            write_register(Register::END, MY_XM125_RANGE_END);
            setup_stage = SetupStage::SET_PROFILE;
            break;

        case SetupStage::SET_PROFILE:
            // Set profile, which configures a group of settings in the device 
            write_register(Register::MAX_PROFILE, MY_XM125_PROFILE);
            setup_stage = SetupStage::SET_REFLECTOR_SHAPE;
            break;

        case SetupStage::SET_REFLECTOR_SHAPE:
            // Set profile, which configures a group of settings in the device 
            write_register(Register::REFLECTOR_SHAPE, MY_XM125_PLANER_SHAPE);
            setup_stage = SetupStage::SET_SIGNAL_QAULITY;
            break;

        case SetupStage::SET_SIGNAL_QAULITY:
            // Set profile, which configures a group of settings in the device 
            write_register(Register::SIGNAL_QUALITY, MY_XM125_SIGNAL_QUALITY);
            setup_stage = SetupStage::SET_THRESHOLD_METHOD;
            break;

        case SetupStage::SET_THRESHOLD_METHOD:
            // Set profile, which configures a group of settings in the device 
            write_register(Register::THRESHOLD_METHOD, THRESHOLD_METHOD_CFAR);
            setup_stage = SetupStage::SET_NUM_FRAMES_THRESHOLD;
            break;

        case SetupStage::SET_NUM_FRAMES_THRESHOLD:
            // Set profile, which configures a group of settings in the device 
            write_register(Register::NUM_FRAMES_RECORDED_THRESHOLD, MY_XM125_NUM_FRAMES);
            setup_stage = SetupStage::ENABLE_CLOSE_RANGE_LEAKAGE;
            break;

        case SetupStage::ENABLE_CLOSE_RANGE_LEAKAGE:
           // Set profile, which configures a group of settings in the device 
           write_register(Register::CLOSE_RANGE_LEAKAGE_CANCELLATION, XM125_ENABLE_CLOSE_LEAKAGE_CALIBRATION);
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
        return;
    }

    if (has_error()) {
        // Don't proceed if we have an error
        health |= uint8_t(Health::DEVICE_ERROR);
        return;
    }

    // get the distance result bitmask
    uint32_t distance_result;
    if (!read_register(Register::DISTANCE_RESULT, distance_result)) {
        // Try again next loop
        health |= uint8_t(Health::FAILED_DEVICE_COMS);
        return;
    }

    // Check for errors in distance result
    if ((distance_result & uint32_t(DistanceResult::DISTANCE_RESULT_MEASUREMENT_ERROR)) != 0) {
        health |= uint8_t(Health::MEASUREMENT_ERROR);
        return;
    }

    // Check if the calibration error flag is set
    if ((distance_result & uint32_t(DistanceResult::DISTANCE_RESULT_CALIBRATION_NEEDED)) != 0) {
        // send the recalibration command
        send_command(Command::RECALIBRATE);
        health |= uint8_t(Health::RECALIBRATION_REQUIRED);
        return;
    }

    // Note: uint32_t(DistanceResult::DISTANCE_RESULT_NEAR_START_EDGE) this is not an error but a warning that there is potentially a result closer than the minimum configured measurement range 

    // Update the temperature measurement
    temp_deg_c = uint16_t((distance_result & uint32_t(DistanceResult::TEMPERATURE_MASK)) >> TEMPERATURE_SHIFT);

    // Loop over the number of returns to get the distances and powers
    uint8_t num_distances = uint8_t(distance_result) & uint8_t(DistanceResult::NUM_DISTANCE_MASK);

    for (uint8_t i=0; i<MAX_PEAKS; i++) {
        // Reset measurement values to 0 if we have not received data for this peak
        if (i >= num_distances) {
            dist_measurement_mm[i] = 0;
            strength_measurement[i] = 0;
            continue;
        }

        uint32_t distance_mm;
        if (!read_register(Register(dist_reg[i]), distance_mm)) {
            health |= uint8_t(Health::FAILED_DEVICE_COMS);
            dist_measurement_mm[i] = 0;
            strength_measurement[i] = 0;
            continue;
        }

        // Apply fixed offset
        distance_mm -= INSTALLED_OFFSET;

        uint32_t peak_strength;
        if (!read_register(Register(strength_reg[i]), peak_strength)) {
            health |= uint8_t(Health::FAILED_DEVICE_COMS);
            dist_measurement_mm[i] = 0;
            strength_measurement[i] = 0;
            continue;
        }

        // If we got this far then we can commit the measurements to memory
        dist_measurement_mm[i] = distance_mm;
        strength_measurement[i] = peak_strength;
    }

    // Update the measurement timer
    last_update_ms = now;

    // Send the measurement command to the sensor and hope that it has completed when we come back for the next update
    send_command(Command::MEASURE_DISTANCE);
}

// Print to serial for logging
// void AP_RangeFinder_AcconeerA121::update_logging(void)
// {
//     Serial.print(millis());
//     Serial.print(",");

//     // Output distances and strength
//     for (uint8_t i=0; i<MAX_PEAKS; i++) {
//         Serial.print(dist_measurement_mm[i]);
//         Serial.print(",");
//         Serial.print(strength_measurement[i]);
//         Serial.print(",");
//     }

//     // Output last update time
//     Serial.print(dt);
//     Serial.print(",");

//     // Log the reported temperature
//     Serial.print(temp_deg_c);
//     Serial.print(",");

//     // update the error and config ok bitmasks in the log
//     Serial.print(get_config_mask());
//     Serial.print(",");
//     Serial.print(get_error());
//     Serial.print(",");
//     Serial.println(health);
// }

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

void AP_RangeFinder_AcconeerA121::send_command(Command cmd)
{
    uint32_t command = uint32_t(cmd);
    write_register(Register::COMMAND, command);
}

// Helper for writting to registers
bool AP_RangeFinder_AcconeerA121::write_register(Register reg, uint32_t data)
{
    const uint16_t regAddress = uint16_t(reg);

    constexpr uint8_t len = 6;
    uint8_t payload[len];

    // Register address (2 bytes, MSB first)
    payload[0] = (regAddress >> 8) & 0xFF; // Address [15:8]
    payload[1] = regAddress & 0xFF;        // Address [7:0]

    // Data (4 bytes, MSB first)
    payload[2] = (data >> 24) & 0xFF; // Data [31:24]
    payload[3] = (data >> 16) & 0xFF; // Data [23:16]
    payload[4] = (data >> 8) & 0xFF;  // Data [15:8]
    payload[5] = data & 0xFF;         // Data [7:0]

    return dev.transfer(payload, len, NULL, 0);
}

// Helper for writting to registers
bool AP_RangeFinder_AcconeerA121::read_register(Register reg, uint32_t& data)
{
    // reset data variable
    data = 0;

    // Register address (2 bytes, MSB first)
    uint8_t reg_add[2];
    reg_add[0] = (uint16_t(reg) >> 8) & 0xFF; // Address [15:8]
    reg_add[1] = (uint16_t(reg) & 0xFF);      // Address [7:0]

    // read data from register
    uint8_t buf[4] = {};
    if (!dev.transfer(reg_add, sizeof(reg_add), buf, sizeof(buf))) {
        return false;
    }

    data |= uint32_t(buf[0]) << 24; // Data [31:24]
    data |= uint32_t(buf[1]) << 16; // Data [23:16]
    data |= uint32_t(buf[2]) << 8;  // Data [15:8]
    data |= uint32_t(buf[3]);       // Data [7:0]
    return true;
}

#endif  // AP_RANGEFINDER_A121_RADAR_ENABLED
