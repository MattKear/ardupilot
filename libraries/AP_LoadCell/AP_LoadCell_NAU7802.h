#include "AP_LoadCell_Backend.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

class AP_LoadCell_NAU7802 : public AP_LoadCell_Backend
{
public:
    AP_LoadCell_NAU7802() :
    AP_LoadCell_Backend()
    {}

    bool get_measurement(float& val) override;

};
