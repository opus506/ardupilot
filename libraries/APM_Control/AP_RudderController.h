// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_RUDDER_CONTROLLER_H__
#define __AP_RUDDER_CONTROLLER_H__

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <DataFlash/DataFlash.h>
#include <AP_Math/AP_Math.h>

class AP_RudderController {
public:
    AP_RudderController(AP_AHRS &ahrs, const AP_Vehicle::FixedWing &parms, DataFlash_Class &_dataflash) :
        aparm(parms),
        _ahrs(ahrs)
    { 
        AP_Param::setup_object_defaults(this, var_info);
    }

    int32_t get_servo_out(float lateral_accel_demand, float scaler, bool disable_integrator);

    void reset_I();

    const       DataFlash_Class::PID_Info& get_pid_info(void) const { return _pid_info; }

    static const struct AP_Param::GroupInfo var_info[];

private:

    // Parameters
    AP_Float    _kp;
    AP_Float    _ki;
    AP_Float    _kd;
    AP_Float    _kff;
    AP_Float    _tau;
    AP_Float    _imax;
    AP_Float    _minspeed;
    AP_Int16    _yaw_max;

    const AP_Vehicle::FixedWing &aparm;
    uint32_t _last_t;
    float _last_out;
    float _last_rate_hp_out;
    float _last_rate_hp_in;

    DataFlash_Class::PID_Info _pid_info;

    AP_AHRS &_ahrs;

};

#endif // __AP_RUDDER_CONTROLLER_H__
