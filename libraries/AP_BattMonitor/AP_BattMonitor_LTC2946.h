#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/I2CDevice.h>
#include "AP_BattMonitor_Backend.h"
#include <utility>

#if AP_BATTERY_LTC2946_ENABLED

class AP_BattMonitor_LTC2946 : public AP_BattMonitor_Backend
{
public:
    /// Constructor
    AP_BattMonitor_LTC2946(AP_BattMonitor &mon,
                          AP_BattMonitor::BattMonitor_State &mon_state,
                          AP_BattMonitor_Params &params);

    bool has_cell_voltages() const override { return false; }
    bool has_temperature() const override { return false; }
    bool has_current() const override { return true; }
    bool get_cycle_count(uint16_t &cycles) const override { return false; }

    virtual void init(void) override;
    virtual void read() override;
    static const struct AP_Param::GroupInfo var_info[];
    
private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;

    bool read_word(const uint8_t reg, uint16_t& data) const;
    void timer(void);

    struct {
        uint16_t count;
        float volt_sum;
        float current_sum;
        HAL_Semaphore sem;
    } accumulate;
    float current_LSB;
    float voltage_LSB;
    AP_Float rShunt;
};

#endif // AP_BATTERY_LTC2946_ENABLED
