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

    // @Param: YMAX
    // @DisplayName: Maximum Yaw Rate
    // @Description: This sets the maximum yaw rate that the controller will demand (degrees/sec). Setting it to zero disables the limit. If this value is set too low, then the yaw can't keep up with the navigation demands and the plane will start weaving. If it is set too high (or disabled by setting to zero) then rudder will get large inputs at the start of turns. A limit of 5 degrees/sec is a good default.
    // @Range: 0 25
    // @Units: degrees/second
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("YMAX",   4, AP_RudderController, _yaw_max,       0),

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

	// @Param: MINSPD
	// @DisplayName: Minimum speed
	// @Description: This is the minimum assumed ground speed in meters/second for rudder control. Having a minimum speed prevents oscillations when the vehicle first starts moving. The vehicle can still fly slower than this limit, but the rudder calculations will be done based on this minimum speed.
	// @Range: 0 5
	// @Increment: 0.1
    // @Units: m/s
	// @User: User
	AP_GROUPINFO("MINSPD",   6, AP_RudderController, _minspeed,    1.0f),

    AP_GROUPEND
};

int32_t AP_RudderController::get_servo_out(float lateral_accel_demand, float scaler, bool disable_integrator)
{
    uint32_t tnow = AP_HAL::millis();
    uint32_t dt = tnow - _last_t;
    if (_last_t == 0 || dt > 1000) {
        dt = 0;
    }
    _last_t = tnow;

    // get ground speed from AHRS
    float ground_speed = _ahrs.groundspeed();
    if (ground_speed < _minspeed) {
        // assume a minimum speed. This stops osciallations when first starting to move
        ground_speed = _minspeed;
    }
    // Calculate the desired earth-frame turn rate given lateral_accel_demand and ground speed
    float ef_desired_rate_deg = ToDeg(lateral_accel_demand / ground_speed);
    _pid_info.desired = ef_desired_rate_deg;

    float airspeed;
    int16_t aspd_min = aparm.airspeed_min;
    if (aspd_min < 1) {
        aspd_min = 1;
    }
    if (!_ahrs.airspeed_estimate(&airspeed)) {
    // If no airspeed available use average of min and max
        airspeed = 0.5f*(float(aspd_min) + float(aparm.airspeed_max));
    }
    if (airspeed < aspd_min) {
        // assume a minimum airspeed. This stops osciallations when first starting to move
        airspeed = aspd_min;
    }

    // Get body-frame rate vector (radians/sec)
    float omega_z = _ahrs.get_gyro().z;

    // Calculate turn rate feedforward by first calculating rate_offset, which is assumed turning rate due to
    // plane's bank angle.  Subtract rate_offset from the desired rate to determing the desired
    // turning rate due to yaw.
    float bank_angle = _ahrs.roll;
    float rate_offset_deg;
    // limit bank angle between +- 80 deg if right way up
    if (fabsf(bank_angle) < 1.5707964f)	{
        bank_angle = constrain_float(bank_angle,-1.3962634f,1.3962634f);
    }
    rate_offset_deg = ToDeg((GRAVITY_MSS / airspeed) * tanf(bank_angle) * cosf(bank_angle));

    // Subtract the roll-angle turn component of rate from the measured rate
    // to calculate the rate relative to the turn requirement in degrees/sec
    float rate_hp_in = ef_desired_rate_deg - rate_offset_deg;
    
    // Apply a high-pass filter to the rate to washout any steady state error
    // due to bias errors in rate_offset_deg
    // Use a cut-off frequency of omega = 0.2 rad/sec
    // Could make this adjustable by replacing 0.9960080 with (1 - omega * dt)
    float rate_hp_out = 0.9960080f * _last_rate_hp_out + rate_hp_in - _last_rate_hp_in;
    _last_rate_hp_out = rate_hp_out;
    _last_rate_hp_in = rate_hp_in;

    _pid_info.FF = rate_hp_out * _kff * scaler;

    float output = _pid_info.FF;

    return output;
}

void AP_RudderController::reset_I()
{
    _pid_info.I = 0;
}
