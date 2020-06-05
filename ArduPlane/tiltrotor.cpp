#include "Plane.h"

/*
  control code for tiltrotors and tiltwings. Enabled by setting
  Q_TILT_MASK to a non-zero value
 */
//锟斤拷斜转锟接猴拷锟斤拷斜锟斤拷锟侥匡拷锟狡达拷锟诫。通锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
//Q_TILT_锟斤拷锟斤拷为锟斤拷锟斤拷值

/*
  calculate maximum tilt change as a proportion from 0 to 1 of tilt
 */
//锟斤拷锟斤拷斜锟斤拷0锟斤拷1锟侥憋拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟叫憋拷浠�
float QuadPlane::tilt_max_change(bool up)
{
    float rate;
    //锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟阶�
    if (up || tilt.max_rate_down_dps <= 0) {
        rate = tilt.max_rate_up_dps;
    } else {
        rate = tilt.max_rate_down_dps;
    }
    if (tilt.tilt_type != TILT_TYPE_BINARY && !up) {
        bool fast_tilt = false;
        if (plane.control_mode == &plane.mode_manual) {
            fast_tilt = true;
        }
        if (hal.util->get_soft_armed() && !in_vtol_mode() && !assisted_flight) {
            fast_tilt = true;
        }
        if (fast_tilt) {
            // allow a minimum of 90 DPS in manual or if we are not
            // stabilising, to give fast control
            rate = MAX(rate, 90);
        }
    }
    //锟斤拷bicopter锟斤拷锟斤拷锟斤拷模式锟斤拷锟斤拷锟秸猴拷锟斤拷锟斤拷锟斤拷值为锟斤拷锟斤拷站锟斤拷锟斤拷锟斤拷转锟劫讹拷值* plane.G_Dt / 90.0f
    return rate * plane.G_Dt / 90.0f;
}

/*
  output a slew limited tiltrotor angle. tilt is from 0 to 1
 */
//锟斤拷锟斤拷锟阶拷锟斤拷锟斤拷锟阶拷锟斤拷锟斤拷嵌取锟斤拷锟叫憋拷却锟�0锟斤拷1
void QuadPlane::tiltrotor_slew(float newtilt)
{
    float max_change = tilt_max_change(newtilt<tilt.current_tilt);
    //constrain_float锟角讹拷newtilt一锟斤拷锟睫凤拷锟斤拷锟节猴拷锟斤拷锟斤拷之锟戒，锟斤拷锟斤拷锟斤拷锟斤拷锟叫★拷锟斤拷锟叫★拷锟斤拷锟斤拷锟睫ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
    tilt.current_tilt = constrain_float(newtilt, tilt.current_tilt-max_change, tilt.current_tilt+max_change);

    // translate to 0..1000 range and output
    //转锟斤拷锟斤拷0..1000锟斤拷围锟斤拷锟斤拷锟�
    //锟斤拷锟斤拷嵌锟�
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, 1000 * tilt.current_tilt);
}

/*
  update motor tilt for continuous tilt servos
 */
//连续性模式倾转和你选的飞行模式有关
void QuadPlane::tiltrotor_continuous_update(void)
{
    // default to inactive
    //默锟较碉拷锟斤拷锟阶�
    tilt.motors_active = false;

    // the maximum rate of throttle change
    //锟斤拷锟斤拷锟斤拷锟戒化锟斤拷
    float max_change;
    
    if (!in_vtol_mode() && (!hal.util->get_soft_armed() || !assisted_flight)) {
        // we are in pure fixed wing mode. Move the tiltable motors all the way forward and run them as
        // a forward motor
        //锟斤拷锟角达拷锟节达拷锟教讹拷锟斤拷模式锟斤拷锟斤拷锟斤拷锟斤拷斜锟斤拷锟揭恢憋拷锟角帮拷贫锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟轿拷锟角帮拷锟斤拷锟斤拷锟斤拷
        //锟斤拷锟斤拷锟斤拷前
        tiltrotor_slew(1);

        max_change = tilt_max_change(false);
        //锟斤拷锟斤拷锟斤拷锟斤拷
        float new_throttle = constrain_float(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)*0.01, 0, 1);
        if (tilt.current_tilt < 1) {
            tilt.current_throttle = constrain_float(new_throttle,
                                                    tilt.current_throttle-max_change,
                                                    tilt.current_throttle+max_change);
        } else {
            tilt.current_throttle = new_throttle;
        }
        if (!hal.util->get_soft_armed()) {
            tilt.current_throttle = 0;
        } else {
            // the motors are all the way forward, start using them for fwd thrust
            uint8_t mask = is_zero(tilt.current_throttle)?0:(uint8_t)tilt.tilt_mask.get();
            motors->output_motor_mask(tilt.current_throttle, mask, plane.rudder_dt);
            // prevent motor shutdown
            tilt.motors_active = true;
        }
        return;
    }

    // remember the throttle level we're using for VTOL flight
    //锟斤拷住锟斤拷锟斤拷锟节达拷直锟金降凤拷锟斤拷锟斤拷使锟矫碉拷锟斤拷锟脚高讹拷
    float motors_throttle = motors->get_throttle();
    max_change = tilt_max_change(motors_throttle<tilt.current_throttle);
    tilt.current_throttle = constrain_float(motors_throttle,
                                            tilt.current_throttle-max_change,
                                            tilt.current_throttle+max_change);
    
    /*
      we are in a VTOL mode. We need to work out how much tilt is
      needed. There are 3 strategies we will use:
               锟斤拷锟角达拷锟节达拷直锟斤拷模式锟斤拷锟斤拷锟斤拷锟斤拷要弄锟斤拷锟斤拷锟揭拷锟叫憋拷锟斤拷佟锟斤拷锟斤拷墙锟斤拷锟斤拷锟斤拷锟斤拷植锟斤拷裕锟�

      1) in QSTABILIZE or QHOVER the angle will be set to zero. This
         enables these modes to be used as a safe recovery mode.
                       锟斤拷QSTABILIZE锟斤拷QHOVER锟叫ｏ拷锟角度斤拷锟斤拷锟斤拷为锟姐。锟斤拷使锟斤拷锟斤拷些模式锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷全锟街革拷模式锟斤拷

      2) in fixed wing assisted flight or velocity controlled modes we
         will set the angle based on the demanded forward throttle,
         with a maximum tilt given by Q_TILT_MAX. This relies on
         Q_VFWD_GAIN being set
                       锟节固讹拷锟斤拷锟斤拷锟斤拷锟斤拷锟叫伙拷锟劫度匡拷锟斤拷模式锟铰ｏ拷锟斤拷锟角斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟角帮拷锟斤拷锟斤拷锟斤拷锟斤拷媒嵌龋锟�
                       锟斤拷锟斤拷锟斤拷锟斤拷Q_TILT_MAX锟斤拷锟斤拷锟斤拷锟斤拷取锟斤拷锟斤拷锟斤拷锟矫碉拷Q_VFWD_GAIN

      3) if we are in TRANSITION_TIMER mode then we are transitioning
         to forward flight and should put the rotors all the way forward
                       锟斤拷锟斤拷锟斤拷谴锟斤拷锟絋RANSITION_TIMER锟斤拷锟斤拷么锟斤拷锟角斤拷锟斤拷锟缴碉拷锟斤拷前锟斤拷锟叫ｏ拷锟斤拷锟斤拷应锟矫斤拷转锟斤拷一直锟斤拷前
    */
    //锟解几锟斤拷模式锟斤拷锟斤拷锟轿�0
    if (plane.control_mode == &plane.mode_qstabilize ||
        plane.control_mode == &plane.mode_qhover ||
        plane.control_mode == &plane.mode_qautotune) {
        tiltrotor_slew(0);
        return;
    }

    if (assisted_flight &&
        transition_state >= TRANSITION_TIMER) {
        // we are transitioning to fixed wing - tilt the motors all
        // the way forward
        //锟斤拷锟斤拷锟斤拷锟节癸拷锟缴碉拷锟教讹拷锟斤拷-锟斤拷锟斤拷锟斤拷一直锟斤拷前锟斤拷斜
        tiltrotor_slew(1);
    } else {
        // until we have completed the transition we limit the tilt to
        // Q_TILT_MAX. Anything above 50% throttle gets
        // Q_TILT_MAX. Below 50% throttle we decrease linearly. This
        // relies heavily on Q_VFWD_GAIN being set appropriately.
        //锟斤拷锟斤拷晒锟斤拷锟街帮拷锟斤拷锟斤拷墙锟斤拷锟叫憋拷锟斤拷锟斤拷锟斤拷锟絈_tilt_MAX锟斤拷锟轿何革拷锟斤拷50%锟斤拷锟斤拷锟脚讹拷锟斤拷玫锟絈_tilt_MAX锟斤拷
        //锟斤拷锟斤拷50%锟斤拷锟斤拷锟脚ｏ拷锟斤拷锟角伙拷锟斤拷锟皆硷拷小锟斤拷锟斤拷锟节很达拷潭锟斤拷锟斤拷锟斤拷锟斤拷锟絈-VFWD-u锟斤拷锟斤拷锟斤拷实锟斤拷锟斤拷谩锟�
        float settilt = constrain_float(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) / 50.0f, 0, 1);
        tiltrotor_slew(settilt * tilt.max_angle_deg / 90.0f);
    }
}


/*
  output a slew limited tiltrotor angle. tilt is 0 or 1
 */
//锟斤拷锟斤拷锟阶拷锟斤拷锟斤拷锟阶拷锟斤拷锟斤拷嵌取锟斤拷锟叫蔽�0锟斤拷1
//slew摆动
void QuadPlane::tiltrotor_binary_slew(bool forward)
{
    // The servo output is binary, not slew rate limited
    //锟脚凤拷锟斤拷锟轿拷锟斤拷锟斤拷疲锟斤拷锟斤拷芑锟阶拷锟斤拷锟斤拷锟�
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, forward?1000:0);

    // rate limiting current_tilt has the effect of delaying throttle in tiltrotor_binary_update
    //锟斤拷tiltrotor_binary_update锟叫ｏ拷锟斤拷锟斤拷锟斤拷锟狡碉拷锟斤拷_tilt锟斤拷锟斤拷锟接筹拷锟斤拷锟脚碉拷效锟斤拷
    float max_change = tilt_max_change(!forward);
    if (forward) {
        tilt.current_tilt = constrain_float(tilt.current_tilt+max_change, 0, 1);
    } else {
        tilt.current_tilt = constrain_float(tilt.current_tilt-max_change, 0, 1);
    }
}

/*
  update motor tilt for binary tilt servos
 */
//二位倾转模式，0或1

void QuadPlane::tiltrotor_binary_update(void)
{
    // motors always active
    tilt.motors_active = true;

    if (!in_vtol_mode()) {
        // we are in pure fixed wing mode. Move the tiltable motors
        // all the way forward and run them as a forward motor
        //我们处于纯固定翼模式。将可倾斜电机一直向前移动，并将其作为向前电机运行
        //二进制倾转调用的是tiltrotor_binary_slew，结果是0或者90度，中间角度是不可控的。
        //如果我想要控制倾转时间，可以选择调用连续型的倾转方式，
        //即tiltrotor_slew（1），这个时候倾转时间是可以改变的，因为有倾转速率，速率为地面站中设置的Q_TILT_RATE_UP
        //tiltrotor_binary_slew(true);
        tiltrotor_slew(1);

        //更新油门
        float new_throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)*0.01f;
        if (tilt.current_tilt >= 1) {
            uint8_t mask = is_zero(new_throttle)?0:(uint8_t)tilt.tilt_mask.get();
            // the motors are all the way forward, start using them for fwd thrust
            motors->output_motor_mask(new_throttle, mask, plane.rudder_dt);
        }
    } else {
        //直升机模态，倾转为0
        //若要修改倾转时间，改成tiltrotor_slew（0）
        //tiltrotor_binary_slew(false);
        tiltrotor_slew(0);
    }
}


/*
  update motor tilt
 */
//倾转的几种模式
void QuadPlane::tiltrotor_update(void)
{
    if (tilt.tilt_mask <= 0) {
        // no motors to tilt
        return;
    }

    if (tilt.tilt_type == TILT_TYPE_BINARY || TILT_TYPE_BICOPTER) {
        tiltrotor_binary_update();
        //如果是二进制模式，进入二进制倾转
        //本文设置的是bicopter，所以是连续性倾转
        //若要改成二进制倾转，else后也是tiltrotor_binary_update();
    } else {
        tiltrotor_continuous_update();
    }

    if (tilt.tilt_type == TILT_TYPE_VECTORED_YAW) {
        tiltrotor_vectored_yaw();
    }
}


/*
  compensate for tilt in a set of motor outputs

  Compensation is of two forms. The first is to apply _tilt_factor,
  which is a compensation for the reduces vertical thrust when
  tilted. This is supplied by set_motor_tilt_factor().

  The second compensation is to use equal thrust on all tilted motors
  when _tilt_equal_thrust is true. This is used when the motors are
  tilted by a large angle to prevent the roll and yaw controllers from
  causing instability. Typically this would be used when the motors
  are tilted beyond 45 degrees. At this angle it is assumed that roll
  control can be achieved using fixed wing control surfaces and yaw
  control with the remaining multicopter motors (eg. tricopter tail).

  By applying _tilt_equal_thrust the tilted motors effectively become
  a single pitch control motor.

  Note that we use a different strategy for when we are transitioning
  into VTOL as compared to from VTOL flight. The reason for that is
  we want to lean towards higher tilted motor throttle when
  transitioning to fixed wing flight, in order to gain airspeed,
  whereas when transitioning to VTOL flight we want to lean to towards
  lower fwd throttle. So we raise the throttle on the tilted motors
  when transitioning to fixed wing, and lower throttle on tilted
  motors when transitioning to VTOL
 */
/*补偿一组电机输出中的倾斜补偿有两种形式。首先是应用倾斜系数，这是对减少垂直推力的补偿
倾斜的。这是由set U motor U tilt U factor（）提供的。第二种补偿是对所有倾斜的电机使用相同的推力当倾斜度等于推力时为真。当电机
大角度倾斜以防止侧倾和偏航控制器引起不稳定。通常，当电机倾斜超过45度。在这个角度，假设控制可以通过固定的机翼控制面和偏航来实现
用剩余的多翼发动机（如三翼机尾部）控制。通过施加等推力，倾斜电机有效地单节距控制马达。注意，我们在转换时使用不同的策略
与垂直起降飞行相比进入垂直起降。原因是当过渡到固定翼飞行，以获得空速，然而，当转换到垂直起降航班时，我们希望倾向于
降低前进油门。所以我们加大倾斜马达的油门当过渡到固定翼时，倾斜时降低油门转换到垂直起降时的电机
*/
void QuadPlane::tilt_compensate_down(float *thrust, uint8_t num_motors)
{
    float inv_tilt_factor;
    if (tilt.current_tilt > 0.98f) {
        inv_tilt_factor = 1.0 / cosf(radians(0.98f*90));
    } else {
        inv_tilt_factor = 1.0 / cosf(radians(tilt.current_tilt*90));
    }

    // when we got past Q_TILT_MAX we gang the tilted motors together
    // to generate equal thrust. This makes them act as a single pitch
    // control motor while preventing them trying to do roll and yaw
    // control while angled over. This greatly improves the stability
    // of the last phase of transitions
    float tilt_threshold = (tilt.max_angle_deg/90.0f);
    bool equal_thrust = (tilt.current_tilt > tilt_threshold);

    float tilt_total = 0;
    uint8_t tilt_count = 0;
    
    // apply inv_tilt_factor first
    for (uint8_t i=0; i<num_motors; i++) {
        if (is_motor_tilting(i)) {
            thrust[i] *= inv_tilt_factor;
            tilt_total += thrust[i];
            tilt_count++;
        }
    }

    float largest_tilted = 0;

    // now constrain and apply _tilt_equal_thrust if enabled
    for (uint8_t i=0; i<num_motors; i++) {
        if (is_motor_tilting(i)) {
            if (equal_thrust) {
                thrust[i] = tilt_total / tilt_count;
            }
            largest_tilted = MAX(largest_tilted, thrust[i]);
        }
    }

    // if we are saturating one of the tilted motors then reduce all
    // motors to keep them in proportion to the original thrust. This
    // helps maintain stability when tilted at a large angle
    if (largest_tilted > 1.0f) {
        float scale = 1.0f / largest_tilted;
        for (uint8_t i=0; i<num_motors; i++) {
            thrust[i] *= scale;
        }
    }
}


/*
  tilt compensation when transitioning to VTOL flight
 */
void QuadPlane::tilt_compensate_up(float *thrust, uint8_t num_motors)
{
    float tilt_factor = cosf(radians(tilt.current_tilt*90));

    // when we got past Q_TILT_MAX we gang the tilted motors together
    // to generate equal thrust. This makes them act as a single pitch
    // control motor while preventing them trying to do roll and yaw
    // control while angled over. This greatly improves the stability
    // of the last phase of transitions
    float tilt_threshold = (tilt.max_angle_deg/90.0f);
    bool equal_thrust = (tilt.current_tilt > tilt_threshold);

    float tilt_total = 0;
    uint8_t tilt_count = 0;
    
    // apply tilt_factor first
    for (uint8_t i=0; i<num_motors; i++) {
        if (!is_motor_tilting(i)) {
            thrust[i] *= tilt_factor;
        } else {
            tilt_total += thrust[i];
            tilt_count++;
        }
    }

    // now constrain and apply _tilt_equal_thrust if enabled
    for (uint8_t i=0; i<num_motors; i++) {
        if (is_motor_tilting(i)) {
            if (equal_thrust) {
                thrust[i] = tilt_total / tilt_count;
            }
        }
    }
}

/*
  choose up or down tilt compensation based on flight mode When going
  to a fixed wing mode we use tilt_compensate_down, when going to a
  VTOL mode we use tilt_compensate_up
 */
void QuadPlane::tilt_compensate(float *thrust, uint8_t num_motors)
{
    if (tilt.current_tilt <= 0) {
        // the motors are not tilted, no compensation needed
        return;
    }
    if (in_vtol_mode()) {
        // we are transitioning to VTOL flight
        tilt_compensate_up(thrust, num_motors);
    } else {
        tilt_compensate_down(thrust, num_motors);
    }
}

/*
  return true if the rotors are fully tilted forward
     纯固定翼模式
 */
bool QuadPlane::tiltrotor_fully_fwd(void)
{
    //tilt_mask代表倾转舵机的数量
    if (tilt.tilt_mask <= 0) {
        return false;
    }
    return (tilt.current_tilt >= 1);
}

/*
  control vectored yaw with tilt multicopters
 */
void QuadPlane::tiltrotor_vectored_yaw(void)
{
    // total angle the tilt can go through
    float total_angle = 90 + tilt.tilt_yaw_angle;
    // output value (0 to 1) to get motors pointed straight up
    float zero_out = tilt.tilt_yaw_angle / total_angle;

    // calculate the basic tilt amount from current_tilt
    float base_output = zero_out + (tilt.current_tilt * (1 - zero_out));
    
    float tilt_threshold = (tilt.max_angle_deg/90.0f);
    bool no_yaw = (tilt.current_tilt > tilt_threshold);
    if (no_yaw) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  1000 * base_output);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, 1000 * base_output);
    } else {
        float yaw_out = motors->get_yaw();
        float yaw_range = zero_out;

        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  1000 * (base_output + yaw_out * yaw_range));
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, 1000 * (base_output - yaw_out * yaw_range));
    }
}

/*
  control bicopter tiltrotors
 */
void QuadPlane::tiltrotor_bicopter(void)
{
    if (tilt.tilt_type != TILT_TYPE_BICOPTER) {
        return;
    }

   //锟缴伙拷模态
   //SRV_Channels
    if (!in_vtol_mode() && tiltrotor_fully_fwd()) {
        //纯固定翼模式
        //左右不应该倾转，所以应该注释掉
        //这个函数的意义就是将倾转舵机的范围设置为-45度
        //functions[75].output_scaled = -4500,SRV_Channel::have_pwm_mask &= ~functions[75].channel_mask;
        //原来75和76是倾转舵机，将其限制为45度，变为矢量舵机。
        //Ser设0
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  -SERVO_MAX);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, -SERVO_MAX);
        //如果是纯固定翼模式，到这里就退出了
        //return代表从这里退出，已验证。
        return;
    }

    //跳出来说明现在是直升机模式
    float throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    //直升机模式
    if (assisted_flight) {
        hold_stabilize(throttle * 0.01f);
        motors_output(true);
    } else {
        motors_output(false);
    }

    // bicopter assumes that trim is up so we scale down so match
    //双选择机假设平衡时向上，所以我们缩小比例，以便匹配
    //现在是不知道倾转多少，下面算的只是一个初步值，后面还要进一步计算
    //get_output_scaled这个函数没太看懂，姑且认为已经知道
    float tilt_left = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorLeft);
    float tilt_right = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorRight);

    if (is_negative(tilt_left)) {
        tilt_left *= tilt.tilt_yaw_angle / 90.0f;
    }
    if (is_negative(tilt_right)) {
        tilt_right *= tilt.tilt_yaw_angle / 90.0f;
    }

    // reduce authority of bicopter as motors are tilted forwards
    //锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷前锟斤拷斜时锟斤拷锟斤拷锟斤拷双锟斤拷锟斤拷锟斤拷权锟斤拷
    //M_PI_2锟斤拷什么锟斤拷
    const float scaling = cosf(tilt.current_tilt * M_PI_2);
    //锟斤拷锟斤拷转锟斤拷锟斤拷锟叫ｏ拷某锟斤拷锟斤拷转锟角度ｏ拷锟铰碉拷锟斤拷锟斤拷锟斤拷锟斤拷
    tilt_left  *= scaling;
    tilt_right *= scaling;

    // add current tilt and constrain
    //添加当前倾斜和约束
    tilt_left  = constrain_float(-(tilt.current_tilt * SERVO_MAX) + tilt_left,  -SERVO_MAX, SERVO_MAX);
    tilt_right = constrain_float(-(tilt.current_tilt * SERVO_MAX) + tilt_right, -SERVO_MAX, SERVO_MAX);

    //输出值为tilt_left何，tilt_right
   //不注释
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  tilt_left);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);
}
