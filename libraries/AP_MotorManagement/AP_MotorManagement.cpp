/*
  Motor Management I2C PX4 driver
*/
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

#include <AP_MotorManagement.h>
#include <AP_HAL.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_motor_management.h>
#include <uORB/topics/motor_management.h>

extern const AP_HAL::HAL& hal;

// Constructor
AP_MotorManagement::AP_MotorManagement()
{
    // orb subscription for motor manager
    _motor_manager_sub = orb_subscribe(ORB_ID(motor_management));
}

// read - read latest voltage and current
void AP_MotorManagement::read()
{
    bool updated = false;
    struct motor_management_s mm_status;

    // check if new info has arrived from the orb
    orb_check(_motor_manager_sub, &updated);

    // retrieve latest info
    if (updated) {
        if (OK == orb_copy(ORB_ID(motor_management), _motor_manager_sub, &mm_status)) {
            _ppm1 = mm_status.ppm_1;
        }
    }
}