/*
  Generic PI algorithm for controllers that don't need filtering (such as heaters)
*/

#include <AP_Math/AP_Math.h>
#include "AC_PI.h"

const AP_Param::GroupInfo AC_PI::var_info[] = {
    // @Param: P
    // @DisplayName: PID Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("P",    1, AC_PI, kP, default_kp),

    // @Param: I
    // @DisplayName: PID Integral Gain
    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("I",    2, AC_PI, kI, default_ki),

    // @Param: IMAX
    // @DisplayName: PID Integral Maximum
    // @Description: The maximum/minimum value that the I term can output
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("IMAX", 3, AC_PI, imax, default_imax),

    AP_GROUPEND
};

// Constructor
AC_PI::AC_PI(float initial_p, float initial_i, float initial_imax) :
    default_kp(initial_p),
    default_ki(initial_i),
    default_imax(initial_imax)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);
}

float AC_PI::update(float measurement, float target, float dt, bool limit)
{
    return update(measurement, target, dt, (limit && is_negative(integrator)), (limit && is_positive(integrator)));
}

float AC_PI::update(float measurement, float target, float dt, bool limit_neg, bool limit_pos)
{
    err = target - measurement;

    update_i(dt, limit_neg, limit_pos);

    output_P = kP * err;

    return output_P + integrator;
}

//  update_i - update the integral
//  if limit_neg is true, the integral can only increase
//  if limit_pos is true, the integral can only decrease
void AC_PI::update_i(float dt, bool limit_neg, bool limit_pos)
{
    if (!is_zero(kI.get())) {
        // Ensure that integrator can only be reduced if the output is saturated
        if (!((limit_neg && is_negative(err)) || (limit_pos && is_positive(err)))) {
            integrator += (err * kI.get()) * dt;
            integrator = constrain_float(integrator, -imax.get(), imax.get());
        }
    } else {
        integrator = 0.0f;
    }
}

void AC_PI::set_integrator(float target, float measurement, float i)
{
    set_integrator(target - measurement, i);
}

void AC_PI::set_integrator(float error, float i)
{
    err = error; // store error for logging
    set_integrator(i - error * kP.get());
}

void AC_PI::set_integrator(float i)
{
    integrator = constrain_float(i, -imax.get(), imax.get());
}