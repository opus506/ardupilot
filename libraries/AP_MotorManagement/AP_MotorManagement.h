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
#ifndef AP_MOTORMANAGEMENT_H
#define AP_MOTORMANAGEMENT_H

#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Math.h>

class AP_MotorManagement {

    public:
        // Methods
        AP_MotorManagement();               // Constructor
        float get_ppm1(){return _ppm1;}     // Returns first ppm
        float get_temp1(){return _temp1;}   // Returns first temperature

    private:
        // Methods
        void read();

        // Members
    int         _motor_manager_sub;         // orb subscription description
    float       _ppm1;                      // ppm of channel 1
    float       _temp1;                     // temp of channel 1
};

#endif // AP_MOTORMANAGEMENT_H