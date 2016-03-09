// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

//  Code by Robert Lefebvre
//  Controller for guiding a plane by rudder only with no aileron control

#include <AP_HAL/AP_HAL.h>
#include "AP_RudderController.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_RudderController::var_info[] = {

    // @Param: TCONST
    // @DisplayName: Yaw Time Constant
    // @Description: This controls the time constant in seconds from demanded to achieved yaw rate. A value of 0.5 is a good default and will work with nearly all models. Advanced users may want to reduce this time to obtain a faster response but there is no point setting a time less than the aircraft can achieve.
    // @Range: 0.4 1.0
    // @Units: seconds
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("TCONST",      0, AP_RudderController, _tau,       0.5f),

    // @Param: P
    // @DisplayName: Proportional Gain
    // @Description: This is the gain from yaw rate error to rudder output.
    // @Range: 0.1 4.0
    // @Increment: 0.1
    // @User: User
    AP_GROUPINFO("P",        1, AP_RudderController, _kp,        0.4f),

    // @Param: D
    // @DisplayName: Damping Gain
    // @Description: This is the gain from yaw acceleration to rudder. This adjusts the damping of the yaw control loop. This gain helps to reduce yawing in turbulence. It should be increased in 0.01 increments as too high a value can lead to a high frequency roll oscillation that could overstress the airframe.
    // @Range: 0 0.1
    // @Increment: 0.01
    // @User: User
    AP_GROUPINFO("D",        2, AP_RudderController, _kd,        0.02f),

    // @Param: I
    // @DisplayName: Integrator Gain
    // @Description: This is the gain from the integral of yaw rate error to rudder. Increasing this gain causes the controller to trim out steady offsets due to an out of trim aircraft.
    // @Range: 0 1.0
    // @Increment: 0.05
    // @User: User
    AP_GROUPINFO("I",        3, AP_RudderController, _ki,        0.04f),

    // @Param: RMAX
    // @DisplayName: Maximum Roll Rate
    // @Description: This sets the maximum roll rate that the controller will demand (degrees/sec). Setting it to zero disables the limit. If this value is set too low, then the roll can't keep up with the navigation demands and the plane will start weaving. If it is set too high (or disabled by setting to zero) then rudder will get large inputs at the start of turns. A limit of 5 degrees/sec is a good default.
    // @Range: 0 25
    // @Units: degrees/second
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RMAX",   4, AP_RudderController, _rate_max,       0),

    // @Param: IMAX
    // @DisplayName: Integrator limit
    // @Description: This limits the number of degrees of rudder in centi-degrees over which the integrator will operate. At the default setting of 3000 centi-degrees, the integrator will be limited to +- 30 degrees of servo travel. The maximum servo deflection is +- 45 centi-degrees, so the default value represents a 2/3rd of the total control throw which is adequate unless the aircraft is severely out of trim.
    // @Range: 0 4500
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("IMAX",      5, AP_RudderController, _imax,        3000),

    // @Param: FF
    // @DisplayName: Feed forward Gain
    // @Description: This is the gain from demanded yaw rate to rudder output. 
    // @Range: 0.1 4.0
    // @Increment: 0.1
    // @User: User
    AP_GROUPINFO("FF",        6, AP_RudderController, _kff,          0.2),

    // @Param: FILT_HZ
    // @DisplayName: PID Input filter frequency in Hz
    // @Description: Input filter frequency in Hz
    // @Unit: Hz
    AP_GROUPINFO("FILT_HZ",     7, AP_RudderController, _filt_hz, 10),

    AP_GROUPEND
};

int32_t AP_RudderController::get_servo_out(float lateral_accel_demand, float scaler, bool disable_integrator)
{
    uint32_t tnow = AP_HAL::millis();
    _dt = (float)(tnow - _last_t)/1000.0;
    if (_last_t == 0 || _dt > 1.0) {
        _dt = 0;
    }
    _last_t = tnow;

    // calculate low-pass filter alpha
    float rc = 1/(M_2PI_F*_filt_hz);
    float alpha = _dt / (_dt + rc);

    // Calculate nav_roll_deg using a maximum bank angle of 45 degrees.
    float nav_roll_deg;
    nav_roll_deg = cosf(_ahrs.pitch)*degrees(atanf(lateral_accel_demand/9.81));
    nav_roll_deg = constrain_float(nav_roll_deg, -45, 45);
    float roll_angle_deg = ToDeg(_ahrs.roll);
    float roll_error_deg = nav_roll_deg - roll_angle_deg;

    float roll_rate_demand_deg = roll_error_deg / _tau;
    // Limit the demanded yaw rate
    if (_rate_max && roll_rate_demand_deg < -_rate_max) {
        roll_rate_demand_deg = - _rate_max;
    } else if (_rate_max && roll_rate_demand_deg > _rate_max) {
        roll_rate_demand_deg = _rate_max;
    }

    float yaw_rate_demand_deg = roll_rate_demand_deg;

    float yaw_rate_deg = ToDeg(_ahrs.get_gyro().z);
    float yaw_rate_error_deg = yaw_rate_demand_deg - yaw_rate_deg;
    float yaw_rate_filter_change = (yaw_rate_error_deg - _yaw_rate_error_filtered) * alpha;
    float derivative = yaw_rate_filter_change / _dt;
    _yaw_rate_error_filtered += yaw_rate_filter_change;

    _pid_info.desired = yaw_rate_demand_deg;
    _pid_info.FF = yaw_rate_demand_deg * _kff * scaler;
    _pid_info.P = _yaw_rate_error_filtered * _kp * scaler;
    _pid_info.D = derivative * _kd * scaler;

    if(!is_zero(_ki) && !is_zero(_dt)) {
        _integrator += ((float)yaw_rate_error_deg * _ki) * _dt * scaler;
        if (_integrator < -_imax) {
            _integrator = -_imax;
        } else if (_integrator > _imax) {
            _integrator = _imax;
        }
        _pid_info.I = _integrator;
    }

    float output = _pid_info.FF + _pid_info.P + _pid_info.I + _pid_info.D;

    return output;
}

void AP_RudderController::reset_I()
{
    _pid_info.I = 0;
}
