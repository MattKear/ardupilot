#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_A121_RADAR_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"

#include <AP_HAL/I2CDevice.h>

class AP_RangeFinder_AcconeerA121 : public AP_RangeFinder_Backend
{

public:
    // I2C default address
    static constexpr uint8_t default_i2c_add = 0x52;

    // constructor
    AP_RangeFinder_AcconeerA121(RangeFinder::RangeFinder_State &_state,
                                AP_RangeFinder_Params &_params,
                                AP_HAL::I2CDevice *dev_ptr);

    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state,
                                          AP_RangeFinder_Params &_params,
                                          AP_HAL::I2CDevice *dev_ptr);

    // update state
    void update(void) override;

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override { return MAV_DISTANCE_SENSOR_RADAR; }

private:

    enum class Register : uint16_t {
        VERSION                            = 0x0000,
        PROTOCOL_STATUS                    = 0x0001,
        MEASURE_COUNTER                    = 0x0002,
        DETECTOR_STATUS                    = 0x0003,
        DISTANCE_RESULT                    = 0x0010,
        PEAK0_DISTANCE                     = 0x0011,
        PEAK1_DISTANCE                     = 0x0012,
        PEAK2_DISTANCE                     = 0x0013,
        PEAK3_DISTANCE                     = 0x0014,
        PEAK4_DISTANCE                     = 0x0015,
        PEAK5_DISTANCE                     = 0x0016,
        PEAK6_DISTANCE                     = 0x0017,
        PEAK7_DISTANCE                     = 0x0018,
        PEAK8_DISTANCE                     = 0x0019,
        PEAK9_DISTANCE                     = 0x001A,
        PEAK0_STRENGTH                     = 0x001B,
        PEAK1_STRENGTH                     = 0x001C,
        PEAK2_STRENGTH                     = 0x001D,
        PEAK3_STRENGTH                     = 0x001E,
        PEAK4_STRENGTH                     = 0x001F,
        PEAK5_STRENGTH                     = 0x0020,
        PEAK6_STRENGTH                     = 0x0021,
        PEAK7_STRENGTH                     = 0x0022,
        PEAK8_STRENGTH                     = 0x0023,
        PEAK9_STRENGTH                     = 0x0024,
        START                              = 0x0040,
        END                                = 0x0041,
        MAX_STEP_LENGTH                    = 0x0042,
        CLOSE_RANGE_LEAKAGE_CANCELLATION   = 0x0043,
        SIGNAL_QUALITY                     = 0x0044,
        MAX_PROFILE                        = 0x0045,
        THRESHOLD_METHOD                   = 0x0046,
        PEAK_SORTING                       = 0x0047,
        NUM_FRAMES_RECORDED_THRESHOLD      = 0x0048,
        FIXED_AMPLITUDE_THRESHOLD_VALUE    = 0x0049,
        THRESHOLD_SENSITIVITY              = 0x004A,
        REFLECTOR_SHAPE                    = 0x004B,
        FIXED_STRENGTH_THRESHOLD_VALUE     = 0x004C,
        MEASURE_ON_WAKEUP                  = 0x0080,
        COMMAND                            = 0x0100,
        APPLICATION_ID                     = 0xFFFF
    };

    const uint8_t dist_reg[10] = {uint8_t(Register::PEAK0_DISTANCE),
                                  uint8_t(Register::PEAK1_DISTANCE),
                                  uint8_t(Register::PEAK2_DISTANCE),
                                  uint8_t(Register::PEAK3_DISTANCE),
                                  uint8_t(Register::PEAK4_DISTANCE),
                                  uint8_t(Register::PEAK5_DISTANCE),
                                  uint8_t(Register::PEAK6_DISTANCE),
                                  uint8_t(Register::PEAK7_DISTANCE),
                                  uint8_t(Register::PEAK8_DISTANCE),
                                  uint8_t(Register::PEAK9_DISTANCE)};

    const uint8_t strength_reg[10] = {uint8_t(Register::PEAK0_STRENGTH),
                                      uint8_t(Register::PEAK1_STRENGTH),
                                      uint8_t(Register::PEAK2_STRENGTH),
                                      uint8_t(Register::PEAK3_STRENGTH),
                                      uint8_t(Register::PEAK4_STRENGTH),
                                      uint8_t(Register::PEAK5_STRENGTH),
                                      uint8_t(Register::PEAK6_STRENGTH),
                                      uint8_t(Register::PEAK7_STRENGTH),
                                      uint8_t(Register::PEAK8_STRENGTH),
                                      uint8_t(Register::PEAK9_STRENGTH)};

    enum class Command: uint32_t {
        APPLY_CONFIG_AND_CALIBRATE   = 1,
        MEASURE_DISTANCE             = 2,
        APPLY_CONFIGURATION          = 3,
        CALIBRATE                    = 4,
        RECALIBRATE                  = 5,
        ENABLE_UART_LOGS             = 32,
        DISABLE_UART_LOGS            = 33,
        LOG_CONFIGURATION            = 34,
        RESET_MODULE                 = 1381192737
    };

    enum class SetupStage : uint8_t {
        NEEDS_RESET,
        CONFIRMING_RESET,
        CHECK_ERRORS,
        SET_START,
        SET_END,
        SET_PROFILE,
        SET_REFLECTOR_SHAPE,
        SET_SIGNAL_QUALITY,
        SET_NUM_FRAMES_THRESHOLD,
        SET_THRESHOLD_METHOD,
        ENABLE_CLOSE_RANGE_LEAKAGE,
        APPLY_AND_CAL,
        CHECK_CONFIG,
        COMPLETE,
    } setup_stage;

    enum class DetectorStatus: uint32_t {
        RSS_REGISTER_OK            = 0x00000001,
        CONFIG_CREATE_OK           = 0x00000002,
        SENSOR_CREATE_OK           = 0x00000004,
        DETECTOR_CREATE_OK         = 0x00000008,
        DETECTOR_BUFFER_OK         = 0x00000010,
        SENSOR_BUFFER_OK           = 0x00000020,
        CALIBRATION_BUFFER_OK      = 0x00000040,
        CONFIG_APPLY_OK            = 0x00000080,
        SENSOR_CALIBRATE_OK        = 0x00000100,
        DETECTOR_CALIBRATE_OK      = 0x00000200,
        RSS_REGISTER_ERROR         = 0x00010000,
        CONFIG_CREATE_ERROR        = 0x00020000,
        SENSOR_CREATE_ERROR        = 0x00040000,
        DETECTOR_CREATE_ERROR      = 0x00080000,
        DETECTOR_BUFFER_ERROR      = 0x00100000,
        SENSOR_BUFFER_ERROR        = 0x00200000,
        CALIBRATION_BUFFER_ERROR   = 0x00400000,
        CONFIG_APPLY_ERROR         = 0x00800000,
        SENSOR_CALIBRATE_ERROR     = 0x01000000,
        DETECTOR_CALIBRATE_ERROR   = 0x02000000,
        DETECTOR_ERROR             = 0x10000000,
        BUSY                       = 0x80000000,
    };

    const uint32_t CONFIG_OK_MASK = uint32_t(DetectorStatus::RSS_REGISTER_OK) |
                                    uint32_t(DetectorStatus::CONFIG_CREATE_OK) |
                                    uint32_t(DetectorStatus::SENSOR_CREATE_OK) |
                                    uint32_t(DetectorStatus::DETECTOR_CREATE_OK) |
                                    uint32_t(DetectorStatus::DETECTOR_BUFFER_OK) |
                                    uint32_t(DetectorStatus::SENSOR_BUFFER_OK) |
                                    uint32_t(DetectorStatus::CALIBRATION_BUFFER_OK) |
                                    uint32_t(DetectorStatus::CONFIG_APPLY_OK) |
                                    uint32_t(DetectorStatus::SENSOR_CALIBRATE_OK) |
                                    uint32_t(DetectorStatus::DETECTOR_CALIBRATE_OK);

    const uint32_t ALL_ERRORS_MASK = uint32_t(DetectorStatus::RSS_REGISTER_ERROR) |
                                     uint32_t(DetectorStatus::CONFIG_CREATE_ERROR) |
                                     uint32_t(DetectorStatus::SENSOR_CREATE_ERROR) |
                                     uint32_t(DetectorStatus::DETECTOR_CREATE_ERROR) |
                                     uint32_t(DetectorStatus::DETECTOR_BUFFER_ERROR) |
                                     uint32_t(DetectorStatus::SENSOR_BUFFER_ERROR) |
                                     uint32_t(DetectorStatus::CALIBRATION_BUFFER_ERROR) |
                                     uint32_t(DetectorStatus::CONFIG_APPLY_ERROR) |
                                     uint32_t(DetectorStatus::SENSOR_CALIBRATE_ERROR) |
                                     uint32_t(DetectorStatus::DETECTOR_CALIBRATE_ERROR);

    enum class DistanceResult : uint32_t {
        NUM_DISTANCE_MASK                     = 0xF,
        DISTANCE_RESULT_NEAR_START_EDGE       = 0x100,
        DISTANCE_RESULT_CALIBRATION_NEEDED    = 0x200,
        DISTANCE_RESULT_MEASUREMENT_ERROR     = 0x400,
        TEMPERATURE_MASK                      = 0xFFFF0000,
    };
    const uint32_t TEMPERATURE_SHIFT = 16;
    const uint32_t DISTANCE_RESULT_ERRORS_MASK = uint32_t(DistanceResult::DISTANCE_RESULT_NEAR_START_EDGE) | uint32_t(DistanceResult::DISTANCE_RESULT_CALIBRATION_NEEDED) | uint32_t(DistanceResult::DISTANCE_RESULT_MEASUREMENT_ERROR);

    enum class Health : uint8_t {
        FAILED_DEVICE_COMS       = 1 << 0,
        BAD_CONFIG               = 1 << 1,
        DEVICE_ERROR             = 1 << 2,
        RECALIBRATION_REQUIRED   = 1 << 3,
        MEASUREMENT_ERROR        = 1 << 4,
        MEASUREMENT_TIMEOUT      = 1 << 5,
    };


    bool init();

    // Setup the radar - Progress through states in a switch case tree to setup the device
    void setup_radar(void);

    void update_measurement(void);

    bool update_detector_status(void);

    bool is_busy(void);

    // Check if all of the config bits are true
    bool config_ok(void);

    // Return the config bitmask
    uint32_t get_config_mask(void);

    // Check if we have received an error code
    bool has_error(void);

    // Return the full error bit mask
    uint32_t get_error();

    bool send_command(Command cmd);

    bool write_register(Register reg, uint32_t data);

    bool read_register(Register reg, uint32_t& data);

    AP_HAL::I2CDevice *dev;

    static constexpr uint8_t MAX_PEAKS = 3;
    uint32_t dist_measurement_mm[MAX_PEAKS];
    uint32_t strength_measurement[MAX_PEAKS];
    uint16_t temp_deg_c;
    uint32_t detector_status;
    uint32_t reset_time_ms;   // track timeout for reseting device if it gets stuck in setup
    uint8_t health;           // bitmask of reasons that we could be unhealthy
    uint32_t last_update_ms;  // last time we succesfully updated the measurment

    // dirty hacks:
    uint32_t time_init_ms;
};

#endif  // AP_RANGEFINDER_A121_RADAR_ENABLED
