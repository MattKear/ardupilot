#pragma once

/*
 Generic PI for systems like heater control, no filtering
*/

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

class AC_PI {
public:
    // Constructor
    AC_PI(float initial_p, float initial_i, float initial_imax);

    CLASS_NO_COPY(AC_PI);

    // update controller - set target and measured inputs to PI controller and calculate outputs
    // The integral is updated based on the setting of the limit flag
    float update(float measurement, float target, float dt, bool limit = false);
    float update(float measurement, float target, float dt, bool limit_neg, bool limit_pos);

    // integrator setting functions
    void set_integrator(float target, float measurement, float i);
    void set_integrator(float error, float i);
    void set_integrator(float i);

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

    float get_P() const {
        return output_P;
    }
    float get_I() const {
        return integrator;
    }
    float get_error() const {
        return err;
    }

protected:
    AP_Float        kP;
    AP_Float        kI;
    AP_Float        imax;
    float           integrator;
    float           output_P;
    float           err;

private:

    // Update I term with limit handling
    void update_i(float dt, bool limit_neg, bool limit_pos);

    const float default_kp;
    const float default_ki;
    const float default_imax;

};
