/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
/*
  helicopter simulator class
*/

// Rough Approximation of a 700 size helicopter
// With Zenoah G29RC Engine

#pragma once

#include "SIM_Aircraft.h"

namespace SITL {

/*
  a helicopter simulator
 */
class Helicopter : public Aircraft {
public:
    Helicopter(const char *home_str, const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input);

    /* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new Helicopter(home_str, frame_str);
    }

private:
    float terminal_rotation_rate = 4*radians(360.0f);
    float hover_throttle = 0.65f;
    float terminal_velocity = 40;                       // m/s
    float hover_lean = 3.0f;                            // Degrees
    float yaw_zero = 0.1f;
    float rotor_rot_accel = radians(20);
    float roll_rate_max = radians(1400);
    float pitch_rate_max = radians(1400);
    float yaw_rate_max = radians(1400);
    float rsc_setpoint = 0.8f;
    float thrust_scale;
    float tail_thrust_scale;
    float tail_rotor_moment_arm = 0.9;                  // Meter, distance from main rotor centerline to tail rotor
    float rotor_rpm = 0.0;                              // rotor rpm
    float rotor_diameter = 1.620;                       // Meter
    float rotor_mass = 0.520;                           // Kg, all blades
    float rotor_drag_coef = 0.0065;
    float rotor_moment_inertia;
    float peak_torque_rpm = 9000.0;                     // RPM where engine reaches peak torque
    float peak_torque = 1.7;                            // Nm
    float gear_ratio = 115/17;                          // ratio
    enum frame_types {
        HELI_FRAME_CONVENTIONAL,
        HELI_FRAME_DUAL,
        HELI_FRAME_COMPOUND
    } frame_type = HELI_FRAME_CONVENTIONAL;
    bool gas_heli = false;
};

} // namespace SITL
