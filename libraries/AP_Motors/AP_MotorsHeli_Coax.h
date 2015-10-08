// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_MotorsHeli_Coax.h
/// @brief	Motor control class for traditional heli

#ifndef __AP_MOTORS_HELI_COAX_H__
#define __AP_MOTORS_HELI_COAX_H__

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>      // RC Channel Library
#include "AP_MotorsHeli.h"


// servo position defaults
#define AP_MOTORS_HELI_COAX_SERVO1_POS                      -60
#define AP_MOTORS_HELI_COAX_SERVO2_POS                      60
#define AP_MOTORS_HELI_COAX_SERVO3_POS                      180

// coax motor output default channels
#define AP_MOTORS_HELI_COAX_CCW_ROTOR                       CH_8
#define AP_MOTORS_HELI_COAX_CW_ROTOR                        CH_7

#define AP_MOTORS_HELI_COLLECTIVE_SETPOINT                  500
#define AP_MOTORS_HELI_COAX_THROTTLE_IDLE_DEFAULT           200

// rotor controller states
#define ROTOR_CONTROL_STOP                      0
#define ROTOR_CONTROL_IDLE                      1
#define ROTOR_CONTROL_ACTIVE                    2

/// @class      AP_MotorsHeli_Coax
class AP_MotorsHeli_Coax : public AP_MotorsHeli {
public:
    // constructor
    AP_MotorsHeli_Coax(RC_Channel&    ccw_rotor,
                         RC_Channel&    cw_rotor,
                         RC_Channel&    servo_1,
                         RC_Channel&    servo_2,
                         RC_Channel&    servo_3,
                         RC_Channel&    servo_4,
                         uint16_t       loop_rate,
                         uint16_t       speed_hz = AP_MOTORS_HELI_SPEED_DEFAULT) :
        AP_MotorsHeli(loop_rate, speed_hz),
        _ccw_rotor(ccw_rotor),
        _cw_rotor(cw_rotor),
        _swash_servo_1(servo_1),
        _swash_servo_2(servo_2),
        _swash_servo_3(servo_3)
    {
        AP_Param::setup_object_defaults(this, var_info);
    };

    // set update rate to motors - a value in hertz
    // you must have setup_motors before calling this
    void set_update_rate(uint16_t speed_hz);

    // enable - starts allowing signals to be sent to motors and servos
    void enable();

    // output_test - spin a motor at the pwm value specified
    //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    void output_test(uint8_t motor_seq, int16_t pwm);

    // allow_arming - returns true if main rotor is spinning and it is ok to arm
    bool allow_arming() const;

    // set_desired_rotor_speed - sets target rotor speed as a number from 0 ~ 1000
    void set_desired_rotor_speed(int16_t desired_speed) {};

    // get_main_rotor_speed - gets estimated or measured main rotor speed
    int16_t get_main_rotor_speed() const { return 0; }

    // get_desired_rotor_speed - gets target rotor speed as a number from 0 ~ 1000
    int16_t get_desired_rotor_speed() const { return 0; }

    // rotor_speed_above_critical - return true if rotor speed is above that critical for flight
    bool rotor_speed_above_critical() const { return true; }

    // calculate_scalars - recalculates various scalars used
    void calculate_scalars();

    // get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    uint16_t get_motor_mask();

    // get_phase_angle - returns phase angle
    int16_t get_phase_angle() const { return _phase_angle; }

    // set_delta_phase_angle for setting variable phase angle compensation and force
    // recalculation of collective factors
    void set_delta_phase_angle(int16_t angle);
    
    // var_info
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // init_outputs - initialise Servo/PWM ranges and endpoints
    void init_outputs();

    // update_motor_controls - sends commands to motor controllers
    void update_motor_control(uint8_t state);

    // calculate_roll_pitch_collective_factors - calculate factors based on swash type and servo position
    void calculate_roll_pitch_collective_factors();

    // heli_move_actuators - moves swash plate and tail rotor
    void move_actuators(int16_t roll_out, int16_t pitch_out, int16_t throttle_out, int16_t yaw_out);

    // internal variables
    int16_t         _ccw_rotor_output;          // output to be sent to ccw rotor
    int16_t         _cw_rotor_output;           // outptut to be sent to cw rotor
    float           _throttle_scalar;           // scalar used to rescale throttle input to account for throttle_idle

    // external objects we depend upon
    RC_Channel&     _ccw_rotor;                 // output to counter-clock-wise rotating rotor (ch7)
    RC_Channel&     _cw_rotor;                  // output to clock-wise rotating rotor (ch8)
    RC_Channel&     _swash_servo_1;             // swash plate servo #1
    RC_Channel&     _swash_servo_2;             // swash plate servo #2
    RC_Channel&     _swash_servo_3;             // swash plate servo #3

    // parameters
    AP_Int16        _servo1_pos;                // Angular location of swash servo #1
    AP_Int16        _servo2_pos;                // Angular location of swash servo #2
    AP_Int16        _servo3_pos;                // Angular location of swash servo #3    
    AP_Int16        _phase_angle;               // Phase angle correction for rotor head.  If pitching the swash forward induces a roll, this can be correct the problem
    AP_Int16        _collective_setpoint;       // Swash servo position for fixed collective pitch
    AP_Float        _throttle_idle;             // minimum throttle output when rotor control engaged
    
};

#endif  // __AP_MOTORS_HELI_COAX_H__
