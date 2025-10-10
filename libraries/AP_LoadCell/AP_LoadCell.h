#pragma once

#include "AP_LoadCell_Backend.h"

class AP_LoadCell
{
public:
    AP_LoadCell() {};

    // Do not allow copies
    // CLASS_NO_COPY(AP_LoadCell);

    void init(void);

    bool get(float& val);

private:
    AP_LoadCell_Backend *sensor;

};
