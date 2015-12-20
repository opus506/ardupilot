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

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include "RPM_PX4_PWM.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_pwm_input.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>
#include <uORB/topics/pwm_input.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>

extern const AP_HAL::HAL& hal;

/* 
   open the sensor in constructor
*/
AP_RPM_PX4_PWM::AP_RPM_PX4_PWM(AP_RPM &_ap_rpm, uint8_t instance, AP_RPM::RPM_State &_state) :
	AP_RPM_Backend(_ap_rpm, instance, _state)
{
    _fd = open(PWMIN0_DEVICE_PATH, O_RDONLY);
    if (_fd == -1) {
        hal.console->printf("Unable to open %s\n", PWMIN0_DEVICE_PATH);
        return;
    }

    // keep a queue of 5 samples to reduce noise by averaging
    if (ioctl(_fd, SENSORIOCSQUEUEDEPTH, 5) != 0) {
        hal.console->printf("Failed to setup RPM queue\n");
        close(_fd);
        _fd = -1;
        return;
    }
}

/* 
   close the file descriptor
*/
AP_RPM_PX4_PWM::~AP_RPM_PX4_PWM()
{
    if (_fd != -1) {
        close(_fd);
        _fd = -1;
    }
}

void AP_RPM_PX4_PWM::update(void)
{
    if (_fd == -1) {
        return;
    }

    struct pwm_input_s pwm;
    uint16_t count = 0;
    const float scaling = ap_rpm._scaling[state.instance];
    float maximum = ap_rpm._maximum[state.instance];
    float minimum = ap_rpm._minimum[state.instance];
    float alpha = ap_rpm._filter_alpha[state.instance];
    float quality = 0;

    while (::read(_fd, &pwm, sizeof(pwm)) == sizeof(pwm)) {
        // the px4 pwm_input driver reports the period in microseconds
        if (pwm.period == 0) {
            continue;
        }

        // calculate new sample RPM value
        float rpm = scaling * (1.0e6f * 60) / pwm.period;

        // check if new RPM sample fits within specified min-max range.  If outside range, we will reject sample and not move the filtered value.
        if ((maximum <= 0 || rpm <= maximum) && (rpm >= minimum)) {

            // calculate synthetic quality value
            // if measured RPM is zero, avoid div-by-zero
            if (is_zero(state.rate_rpm)){
                quality = 0;
            } else {
                quality = 1 - constrain_float((fabsf(rpm-state.rate_rpm))/state.rate_rpm, 0.0, 1.0);
                quality = powf(quality, 2.0);
            }

            // apply simple LPF to new sample value
            state.rate_rpm = (alpha * rpm) + ((1.0 - alpha) * state.rate_rpm);

            // increment data-point counter
            count++;
        } else {
            // we have rejected an RPM sample, so assign quality value of 0 to this sample.
            quality = 0;
        }

        // LPF new sample quality value
        state.signal_quality = (0.1 * quality) + (0.9 * state.signal_quality);      // simple LPF
    }

    // we received at least one data point this cycle, so increment time-out counter
    if (count != 0) {
        state.last_reading_ms = hal.scheduler->millis();
    }
}

#endif // CONFIG_HAL_BOARD
