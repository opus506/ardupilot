// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_MotorsHeli_Coax.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsHeli_Coax::var_info[] PROGMEM = {
    AP_NESTEDGROUPINFO(AP_MotorsHeli, 0),
    
    // @Param: SV1_POS
    // @DisplayName: Servo 1 Position
    // @Description: Angular location of swash servo #1
    // @Range: -180 180
    // @Units: Degrees
    // @User: Standard
    // @Increment: 1
    AP_GROUPINFO("SV1_POS", 1, AP_MotorsHeli_Coax, _servo1_pos, AP_MOTORS_HELI_COAX_SERVO1_POS),

    // @Param: SV2_POS
    // @DisplayName: Servo 2 Position
    // @Description: Angular location of swash servo #2
    // @Range: -180 180
    // @Units: Degrees
    // @User: Standard
    // @Increment: 1
    AP_GROUPINFO("SV2_POS", 2, AP_MotorsHeli_Coax, _servo2_pos, AP_MOTORS_HELI_COAX_SERVO2_POS),

    // @Param: SV3_POS
    // @DisplayName: Servo 3 Position
    // @Description: Angular location of swash servo #3
    // @Range: -180 180
    // @Units: Degrees
    // @User: Standard
    // @Increment: 1
    AP_GROUPINFO("SV3_POS", 3, AP_MotorsHeli_Coax, _servo3_pos, AP_MOTORS_HELI_COAX_SERVO3_POS),

    // @Param: PHANG
    // @DisplayName: Swashplate Phase Angle Compensation
    // @Description: Phase angle correction for rotor head.  If pitching the swash forward induces a roll, this can be correct the problem
    // @Range: -90 90
    // @Units: Degrees
    // @User: Advanced
    // @Increment: 1
    AP_GROUPINFO("PHANG", 4, AP_MotorsHeli_Coax, _phase_angle, 0),
    
    // @Param: COL_SETPOINT
    // @DisplayName: Collective Pitch Set Point
    // @Description: Desired fixed collective pitch setpoint
    // @Range: 0 1000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("COL_SETPOINT", 5, AP_MotorsHeli_Coax,  _collective_setpoint, AP_MOTORS_HELI_COLLECTIVE_SETPOINT),

    AP_GROUPEND
};

// set update rate to motors - a value in hertz
void AP_MotorsHeli_Coax::set_update_rate( uint16_t speed_hz )
{
    // record requested speed
    _speed_hz = speed_hz;

    // setup fast channels
    uint32_t mask = 
        1U << pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_1]) |
        1U << pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_2]) |
        1U << pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_3]) |
        1U << pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_4]);
    hal.rcout->set_freq(mask, _speed_hz);
}

// enable - starts allowing signals to be sent to motors and servos
void AP_MotorsHeli_Coax::enable()
{
    // enable output channels
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_1]));    // swash servo 1
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_2]));    // swash servo 2
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_3]));    // swash servo 3
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_4]));    // yaw
    hal.rcout->enable_ch(AP_MOTORS_HELI_COAX_CCW_ROTOR);                                 // output for gyro gain or direct drive variable pitch tail motor
    hal.rcout->enable_ch(AP_MOTORS_HELI_COAX_CW_ROTOR);                                 // output for main rotor esc

    // disable channels 7 and 8 from being used by RC_Channel_aux
    RC_Channel_aux::disable_aux_channel(_motor_to_channel_map[AP_MOTORS_HELI_COAX_CCW_ROTOR]);
    RC_Channel_aux::disable_aux_channel(_motor_to_channel_map[AP_MOTORS_HELI_COAX_CW_ROTOR]);
}

// init_outputs - initialise Servo/PWM ranges and endpoints
void AP_MotorsHeli_Coax::init_outputs()
{
    // reset swash servo range and endpoints
    reset_swash_servo (_swash_servo_1);
    reset_swash_servo (_swash_servo_2);
    reset_swash_servo (_swash_servo_3);

    // set servo range
    _ccw_rotor.set_range(0,1000);
    _cw_rotor.set_range(0,1000);

}

// output_test - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsHeli_Coax::output_test(uint8_t motor_seq, int16_t pwm)
{
    // exit immediately if not armed
    if (!armed()) {
        return;
    }

    // output to motors and servos
    switch (motor_seq) {
        case 1:
            // swash servo 1
            hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_1]), pwm);
            break;
        case 2:
            // swash servo 2
            hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_2]), pwm);
            break;
        case 3:
            // swash servo 3
            hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_3]), pwm);
            break;
        default:
            // do nothing
            break;
    }
}

// allow_arming - check if it's safe to arm
bool AP_MotorsHeli_Coax::allow_arming() const
{
    // all other cases it is OK to arm
    return true;
}

// calculate_scalars - recalculates various scalers used.
void AP_MotorsHeli_Coax::calculate_scalars()
{
    // range check collective min, max and mid
    if( _collective_min >= _collective_max ) {
        _collective_min = AP_MOTORS_HELI_COLLECTIVE_MIN;
        _collective_max = AP_MOTORS_HELI_COLLECTIVE_MAX;
    }

    // determine roll, pitch and collective input scaling
    _roll_scaler = (float)_roll_max/4500.0f;
    _pitch_scaler = (float)_pitch_max/4500.0f;
    _collective_scalar = ((float)(_collective_max-_collective_min))/1000.0f;

    // calculate factors based on swash type and servo position
    calculate_roll_pitch_collective_factors();
}

// calculate_roll_pitch_collective_factors - calculate factors based on swash type and servo position
void AP_MotorsHeli_Coax::calculate_roll_pitch_collective_factors()
{

	// roll factors
	_rollFactor[CH_1] = cosf(radians(_servo1_pos + 90 - (_phase_angle + _delta_phase_angle)));
	_rollFactor[CH_2] = cosf(radians(_servo2_pos + 90 - (_phase_angle + _delta_phase_angle)));
	_rollFactor[CH_3] = cosf(radians(_servo3_pos + 90 - (_phase_angle + _delta_phase_angle)));

	// pitch factors
	_pitchFactor[CH_1] = cosf(radians(_servo1_pos - (_phase_angle + _delta_phase_angle)));
	_pitchFactor[CH_2] = cosf(radians(_servo2_pos - (_phase_angle + _delta_phase_angle)));
	_pitchFactor[CH_3] = cosf(radians(_servo3_pos - (_phase_angle + _delta_phase_angle)));

	// collective factors
	_collectiveFactor[CH_1] = 1;
	_collectiveFactor[CH_2] = 1;
	_collectiveFactor[CH_3] = 1;

}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsHeli_Coax::get_motor_mask()
{
    // heli uses channels 1,2,3,4,7 and 8
    return (1U << 0 | 1U << 1 | 1U << 2 | 1U << 3 | 1U << AP_MOTORS_HELI_COAX_CCW_ROTOR | 1U << AP_MOTORS_HELI_COAX_CW_ROTOR);
}

// set_delta_phase_angle for setting variable phase angle compensation and force
// recalculation of collective factors
void AP_MotorsHeli_Coax::set_delta_phase_angle(int16_t angle)
{
    angle = constrain_int16(angle, -90, 90);
    _delta_phase_angle = angle;
    calculate_roll_pitch_collective_factors();
}

//
// move_actuators - moves swash plate and tail rotor
//                 - expected ranges:
//                       roll : -4500 ~ 4500
//                       pitch: -4500 ~ 4500
//                       collective: 0 ~ 1000
//                       yaw:   -4500 ~ 4500
//
void AP_MotorsHeli_Coax::move_actuators(int16_t roll_out, int16_t pitch_out, int16_t throttle_out, int16_t yaw_out)
{
    int16_t coll_out_scaled;

    // initialize limits flag
    limit.roll_pitch = false;
    limit.yaw = false;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    // rescale roll_out and pitch-out into the min and max ranges to provide linear motion
    // across the input range instead of stopping when the input hits the constrain value
    // these calculations are based on an assumption of the user specified roll_max and pitch_max
    // coming into this equation at 4500 or less, and based on the original assumption of the
    // total _servo_x.servo_out range being -4500 to 4500.
    roll_out = roll_out * _roll_scaler;
    if (roll_out < -_roll_max) {
        roll_out = -_roll_max;
        limit.roll_pitch = true;
    }
    if (roll_out > _roll_max) {
        roll_out = _roll_max;
        limit.roll_pitch = true;
    }

    // scale pitch and update limits
    pitch_out = pitch_out * _pitch_scaler;
    if (pitch_out < -_pitch_max) {
        pitch_out = -_pitch_max;
        limit.roll_pitch = true;
    }
    if (pitch_out > _pitch_max) {
        pitch_out = _pitch_max;
        limit.roll_pitch = true;
    }

    if (throttle_out < 200){
            yaw_out = 0;
            limit.yaw = true;
    } else {
        // scale yaw and update limits
        yaw_out = yaw_out / 4.5;
        if (yaw_out < -100) {
            yaw_out = -100;
            limit.yaw = true;
        }
        if (yaw_out > 100) {
            yaw_out = 100;
            limit.yaw = true;
        }
    }

    // constrain collective input
    _collective_out = _collective_setpoint;
    if (_collective_out <= 0) {
        _collective_out = 0;
        limit.throttle_lower = true;
    }
    if (_collective_out >= 1000) {
        _collective_out = 1000;
        limit.throttle_upper = true;
    }

    // scale collective pitch
    coll_out_scaled = _collective_out * _collective_scalar + _collective_min - 1000;

    _cw_rotor_output = throttle_out - yaw_out;
    _ccw_rotor_output = throttle_out + yaw_out;

    // swashplate servos
    _swash_servo_1.servo_out = (_rollFactor[CH_1] * roll_out + _pitchFactor[CH_1] * pitch_out)/10 + _collectiveFactor[CH_1] * coll_out_scaled + (_swash_servo_1.radio_trim-1500);
    _swash_servo_2.servo_out = (_rollFactor[CH_2] * roll_out + _pitchFactor[CH_2] * pitch_out)/10 + _collectiveFactor[CH_2] * coll_out_scaled + (_swash_servo_2.radio_trim-1500);
    _swash_servo_3.servo_out = (_rollFactor[CH_3] * roll_out + _pitchFactor[CH_3] * pitch_out)/10 + _collectiveFactor[CH_3] * coll_out_scaled + (_swash_servo_3.radio_trim-1500);

    // use servo_out to calculate pwm_out and radio_out
    _swash_servo_1.calc_pwm();
    _swash_servo_2.calc_pwm();
    _swash_servo_3.calc_pwm();

    // actually move the servos
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_1]), _swash_servo_1.radio_out);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_2]), _swash_servo_2.radio_out);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_3]), _swash_servo_3.radio_out);
}

// update_motor_controls - sends commands to motor controllers
void AP_MotorsHeli_Coax::update_motor_control(uint8_t state)
{
    switch (state) {

        case ROTOR_CONTROL_STOP:
            _ccw_rotor_output = 0;
            _cw_rotor_output = 0;
            break;

        case ROTOR_CONTROL_IDLE:
            _ccw_rotor_output = 0;
            _cw_rotor_output = 0;
            break;

        case ROTOR_CONTROL_ACTIVE:
            break;
    }

    _ccw_rotor.servo_out = _ccw_rotor_output;
    _ccw_rotor.calc_pwm();
    hal.rcout->write(AP_MOTORS_HELI_COAX_CCW_ROTOR, _ccw_rotor.radio_out);

    _cw_rotor.servo_out = _cw_rotor_output;
    _cw_rotor.calc_pwm();
    hal.rcout->write(AP_MOTORS_HELI_COAX_CW_ROTOR, _cw_rotor.radio_out);
}
