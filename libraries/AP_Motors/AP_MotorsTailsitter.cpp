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
 *       AP_MotorsTailsitter.cpp - ArduCopter motors library for tailsitters and bicopters
 *
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorsTailsitter.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define SERVO_OUTPUT_RANGE  4500

// init
void AP_MotorsTailsitter::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // setup default motor and servo mappings
    //设置默认电机和舵机映射，地面站上给的

    uint8_t chan;

    // right throttle defaults to servo output 1
    //右油门默认为舵机输出输出1
    //现在只是普通的左电机和右电机
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_throttleRight, CH_1);
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleRight, chan)) {
        //有右油门电机
        motor_enabled[chan] = true;
    }


    // left throttle defaults to servo output 2
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_throttleLeft, CH_2);
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleLeft, chan)) {
        //有左油门电机
        motor_enabled[chan] = true;
    }
    /*

    // right servo defaults to servo output 3
    //右舵机默认为舵机输出3
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorRight, CH_3);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorRight, SERVO_OUTPUT_RANGE);

    // left servo defaults to servo output 4
    //左舵机默认为舵机输出4
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorLeft, CH_4);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorLeft, SERVO_OUTPUT_RANGE);
    */

    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor1, CH_3);
    SRV_Channels::set_angle(SRV_Channel::k_motor1, SERVO_OUTPUT_RANGE);

    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor2, CH_4);
    SRV_Channels::set_angle(SRV_Channel::k_motor2, SERVO_OUTPUT_RANGE);

    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor3, CH_5);
    SRV_Channels::set_angle(SRV_Channel::k_motor3, SERVO_OUTPUT_RANGE);

    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor4, CH_6);
    SRV_Channels::set_angle(SRV_Channel::k_motor4, SERVO_OUTPUT_RANGE);

    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor5, CH_7);
    SRV_Channels::set_angle(SRV_Channel::k_motor5, SERVO_OUTPUT_RANGE);

    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor6, CH_8);
    SRV_Channels::set_angle(SRV_Channel::k_motor6, SERVO_OUTPUT_RANGE);


    // record successful initialisation if what we setup was the desired frame_class
    //如果我们设置的是所需的机架构型，则记录成功的初始化
    _flags.initialised_ok = (frame_class == MOTOR_FRAME_TAILSITTER);
}


/// Constructor
AP_MotorsTailsitter::AP_MotorsTailsitter(uint16_t loop_rate, uint16_t speed_hz) :
    AP_MotorsMulticopter(loop_rate, speed_hz)
{
    set_update_rate(speed_hz);
}


// set update rate to motors - a value in hertz
void AP_MotorsTailsitter::set_update_rate(uint16_t speed_hz)
{
    // record requested speed
    _speed_hz = speed_hz;

    SRV_Channels::set_rc_frequency(SRV_Channel::k_throttleLeft, speed_hz);
    SRV_Channels::set_rc_frequency(SRV_Channel::k_throttleRight, speed_hz);
}

void AP_MotorsTailsitter::output_to_motors()
{
    if (!_flags.initialised_ok) {
        return;
    }

//下面是两个电机在地面，怠速以及飞行的pwm值
//应该加两个电机
    switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, get_pwm_output_min());
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, get_pwm_output_min());
            break;
        case SpoolState::GROUND_IDLE:
            set_actuator_with_slew(_actuator[1], actuator_spin_up_to_ground_idle());
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, output_to_pwm(actuator_spin_up_to_ground_idle()));
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, output_to_pwm(actuator_spin_up_to_ground_idle()));
            break;
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:

            //原始函数
            //SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, output_to_pwm(thrust_to_actuator(_thrust_left)));
            //SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, output_to_pwm(thrust_to_actuator(_thrust_right)));

            //新函数
            //这两个电机不提供油门功能，所以将其设置为怠速情况下的转速
            set_actuator_with_slew(_actuator[1], actuator_spin_up_to_ground_idle());
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, output_to_pwm(actuator_spin_up_to_ground_idle()));
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, output_to_pwm(actuator_spin_up_to_ground_idle()));
            break;
    }
    // Always output to tilt servos

    //SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, _tilt_left*SERVO_OUTPUT_RANGE);
    //SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, _tilt_right*SERVO_OUTPUT_RANGE);

    //这六个包括了全部操纵，俯仰，偏航，滚转，油门跟以前不一样，前面俯仰偏航耦合，但是和滚转与油门是解耦的，但是现在都是耦合的。

    SRV_Channels::set_output_scaled(SRV_Channel::k_motor1, _left1*SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor2, _left2*SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor3, _left3*SERVO_OUTPUT_RANGE);

    SRV_Channels::set_output_scaled(SRV_Channel::k_motor4, _right1*SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor5, _right2*SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor6, _right3*SERVO_OUTPUT_RANGE);

    /*

    SRV_Channels::set_output_scaled(SRV_Channel::k_motor1, (_left1+thrust_to_actuator(_left1))*SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor2, (_left2+thrust_to_actuator(_left2))*SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor3, (_left3+thrust_to_actuator(_left3))*SERVO_OUTPUT_RANGE);

    SRV_Channels::set_output_scaled(SRV_Channel::k_motor4, (_right1+thrust_to_actuator(_right1))*SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor5, (_right2+thrust_to_actuator(_right2))*SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor6, (_right3+thrust_to_actuator(_right3))*SERVO_OUTPUT_RANGE);
    */

}

// get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsTailsitter::get_motor_mask()
{
    uint32_t motor_mask = 0;
    uint8_t chan;
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleLeft, chan)) {
        motor_mask |= 1U << chan;
    }
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleRight, chan)) {
        motor_mask |= 1U << chan;
    }

    // add parent's mask
    motor_mask |= AP_MotorsMulticopter::get_motor_mask();

    return motor_mask;
}

// calculate outputs to the motors
void AP_MotorsTailsitter::output_armed_stabilizing()
{
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0
    float   thrust_max;                 // highest motor value
    float   thr_adj = 0.0f;             // the difference between the pilot's desired throttle and throttle_thrust_best_rpy 飞行员期望的油门和油门推力的差别

    // apply voltage and air pressure compensation
    const float compensation_gain = get_compensation_gain();
    //_代表姿态控制器算出来的值
    //这些力矩是我需要的值，也就是我要输入的值
    //_roll_in等都是-1..1 油门是0..1
    roll_thrust = (_roll_in + _roll_in_ff) * compensation_gain;
    pitch_thrust = (_pitch_in + _pitch_in_ff) * compensation_gain;
    yaw_thrust = (_yaw_in + _yaw_in_ff) * compensation_gain;
    throttle_thrust = get_throttle() * compensation_gain;

    // sanity check throttle is above zero and below current limited throttle
    if (throttle_thrust <= 0.0f) {
        throttle_thrust = 0.0f;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= _throttle_thrust_max) {
        throttle_thrust = _throttle_thrust_max;
        limit.throttle_upper = true;
    }

    // calculate left and right throttle outputs
    _thrust_left  = throttle_thrust + roll_thrust * 0.5f;
    _thrust_right = throttle_thrust - roll_thrust * 0.5f;

    // if max thrust is more than one reduce average throttle
    thrust_max = MAX(_thrust_right,_thrust_left);
    if (thrust_max > 1.0f) {
        thr_adj = 1.0f - thrust_max;
        limit.throttle_upper = true;
        limit.roll = true;
        limit.pitch = true;
    }

    // Add adjustment to reduce average throttle
    //左右油门和总油门都限制在0-1之间
    _thrust_left  = constrain_float(_thrust_left  + thr_adj, 0.0f, 1.0f);
    _thrust_right = constrain_float(_thrust_right + thr_adj, 0.0f, 1.0f);
    _throttle = throttle_thrust + thr_adj;
    // compensation_gain can never be zero
    _throttle_out = _throttle / compensation_gain;

    // thrust vectoring
    //左右倾转都限制在-1-1之间
    //不明白这块为什么就变成了在（-1,1）之间了
    _tilt_left  = pitch_thrust - yaw_thrust;
    _tilt_right = pitch_thrust + yaw_thrust;

    //重新分配
    //左边
    //应该限制在0..1
    _left3 = _thrust_left + _tilt_left * 0.3f;
    _left2 = _thrust_left - _tilt_left * 0.3f;
    _left1 = _thrust_left - yaw_thrust * 0.3f;


    //右边
    _right3 = _thrust_right + _tilt_right * 0.3f;
    _right2 = _thrust_right - _tilt_right * 0.3f;
    _right1 = _thrust_right + yaw_thrust * 0.3f;

    //接下来应该对这六个值进行限幅，限制在0..1之间
    //因为初始情况下，6个舵机的行程是最小的，这时候总矩最小
    //解释下这个限幅函数，不确定用这个限幅函数是否合理

    _left1=constrain_float(_left1, 0.0f, 1.0f);
    _left2=constrain_float(_left2, 0.0f, 1.0f);
    _left3=constrain_float(_left3, 0.0f, 1.0f);

    _right1=constrain_float(_right1, 0.0f, 1.0f);
    _right2=constrain_float(_right2, 0.0f, 1.0f);
    _right3=constrain_float(_right3, 0.0f, 1.0f);


}

// output_test_seq - spin a motor at the pwm value specified
//以指定pwm值旋转电机
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//motor_seq是电机的序列号，从1到机架上的电机数量
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsTailsitter::output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // exit immediately if not armed
    if (!armed()) {
        return;
    }

    // output to motors and servos
    switch (motor_seq) {
    /*
                    原始代码
        case 1:
            // right throttle
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, pwm);
            break;
        case 2:
            // right tilt servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_tiltMotorRight, pwm);
            break;
        case 3:
            // left throttle
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, pwm);
            break;
        case 4:
            // left tilt servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_tiltMotorLeft, pwm);
            break;
        default:
            // do nothing
            break;
       */
    //修改代码
    case 1:
            //motor1
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor1, pwm);
            break;
            case 2:
            //motor2
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor2, pwm);
            break;
            case 3:
            //motor3
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor3, pwm);
            break;
            case 4:
            //motor4
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor4, pwm);
            break;

            case 5:
            //motor5
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor5, pwm);
            break;

            case 6:
            //motor6
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor6, pwm);
            break;

            default:
            // do nothing
            break;

    }
}
