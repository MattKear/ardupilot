#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_TOUGHSONIC_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

class AP_RangeFinder_ToughSonic : public AP_RangeFinder_Backend_Serial
{

public:

    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return new AP_RangeFinder_ToughSonic(_state, _params);
    }

    void init_serial(uint8_t serial_instance) override;

protected:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

private:

    const uint8_t sensor_ID = 1;

    uint8_t buffer[7];
    uint8_t buffer_offset;

    bool get_reading(float &reading_m) override;

    void send_read_holding_registers(const uint8_t ID, const uint16_t start_address, const uint16_t count);

};
#endif // AP_RANGEFINDER_TOUGHSONIC_ENABLED
