/**
 * @file pid_controller.c
 * @brief PID控制器实现 - 高性能电机控制核心
 */

#include "pid_controller.h"
#include <string.h>

/* ==================== PID控制器实现 ==================== */

void pid_init(PIDController* pid, float kp, float ki, float kd,
              float output_min, float output_max) {
    memset(pid, 0, sizeof(PIDController));
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->deadband = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

void pid_reset(PIDController* pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

void pid_set_params(PIDController* pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void pid_set_limits(PIDController* pid, float min, float max) {
    pid->output_min = min;
    pid->output_max = max;
}

void pid_set_deadband(PIDController* pid, float deadband) {
    pid->deadband = deadband;
}

float pid_compute(PIDController* pid, float setpoint, 
                  float measurement, float dt) {
    // 计算误差
    float error = setpoint - measurement;
    
    // 死区处理
    if (fabsf(error) < pid->deadband) {
        error = 0.0f;
    }
    
    // P项
    float p_term = pid->kp * error;
    
    // I项（带抗饱和）
    pid->integral += error * dt;
    
    // 计算临时输出用于抗饱和判断
    float temp_output = p_term + pid->ki * pid->integral;
    
    // 抗积分饱和：当输出饱和时限制积分（ki=0时跳过，避免除零）
    if (temp_output > pid->output_max) {
        if (fabsf(pid->ki) > 1e-6f) {
            pid->integral = (pid->output_max - p_term) / pid->ki;
        }
        temp_output = pid->output_max;
    } else if (temp_output < pid->output_min) {
        if (fabsf(pid->ki) > 1e-6f) {
            pid->integral = (pid->output_min - p_term) / pid->ki;
        }
        temp_output = pid->output_min;
    }
    
    // D项（带低通滤波防止高频噪声放大）
    float d_term = 0.0f;
    if (dt > 0.0001f) {  // 防止除零
        float derivative = (error - pid->prev_error) / dt;
        d_term = pid->kd * derivative;
    }
    
    // 更新上次误差
    pid->prev_error = error;
    
    // 计算最终输出
    float output = temp_output + d_term;
    
    // 最终限幅
    output = CLAMP(output, pid->output_min, pid->output_max);
    
    return output;
}

float pid_compute_with_ff(PIDController* pid, float setpoint,
                          float measurement, float feedforward, float dt) {
    float output = pid_compute(pid, setpoint, measurement, dt);
    output += feedforward;
    return CLAMP(output, pid->output_min, pid->output_max);
}

/* ==================== 差速驱动运动学实现 ==================== */

void diff_drive_init(DiffDriveKinematics* kin, float wheel_base, float wheel_radius) {
    kin->wheel_base = wheel_base;
    kin->wheel_radius = wheel_radius;
    kin->max_wheel_speed = MOTOR_MAX_SPEED / wheel_radius;
    kin->max_linear = MOTOR_MAX_SPEED;
    kin->max_angular = MOTOR_MAX_ANGULAR;
}

void diff_drive_twist_to_wheels(const DiffDriveKinematics* kin,
                                 float linear, float angular,
                                 float* left_speed, float* right_speed) {
    // 限制输入范围
    linear = CLAMP(linear, -kin->max_linear, kin->max_linear);
    angular = CLAMP(angular, -kin->max_angular, kin->max_angular);
    
    // 差速驱动运动学公式
    // v_left = v - ω * L/2
    // v_right = v + ω * L/2
    *left_speed = linear - angular * kin->wheel_base / 2.0f;
    *right_speed = linear + angular * kin->wheel_base / 2.0f;
    
    // 如果轮速超过限制，进行缩放
    float max_speed = MAX(fabsf(*left_speed), fabsf(*right_speed));
    if (max_speed > kin->max_linear) {
        float scale = kin->max_linear / max_speed;
        *left_speed *= scale;
        *right_speed *= scale;
    }
}

void diff_drive_wheels_to_twist(const DiffDriveKinematics* kin,
                                 float left_speed, float right_speed,
                                 float* linear, float* angular) {
    // 逆运动学公式
    // v = (v_left + v_right) / 2
    // ω = (v_right - v_left) / L
    *linear = (left_speed + right_speed) / 2.0f;
    *angular = (right_speed - left_speed) / kin->wheel_base;
}

float encoder_to_speed(int32_t pulses, float dt,
                       uint16_t pulses_per_rev, float wheel_radius) {
    if (dt < 0.0001f) return 0.0f;
    
    // 计算角速度 (rad/s)
    float angular_vel = (2.0f * 3.14159265f * pulses) / (pulses_per_rev * dt);
    
    // 转换为线速度 (m/s)
    return angular_vel * wheel_radius;
}

void odometry_update(int32_t left_pulses, int32_t right_pulses,
                     const DiffDriveKinematics* kin,
                     uint16_t pulses_per_rev, Pose2D* pose) {
    // 计算左右轮行进距离
    float wheel_circumference = 2.0f * 3.14159265f * kin->wheel_radius;
    float left_dist = (float)left_pulses / pulses_per_rev * wheel_circumference;
    float right_dist = (float)right_pulses / pulses_per_rev * wheel_circumference;
    
    // 计算线位移和角位移
    float linear_dist = (left_dist + right_dist) / 2.0f;
    float angular_dist = (right_dist - left_dist) / kin->wheel_base;
    
    // 更新位姿（使用弧度制）
    pose->theta += angular_dist;
    
    // 归一化角度到 [-π, π]
    while (pose->theta > 3.14159265f) pose->theta -= 2.0f * 3.14159265f;
    while (pose->theta < -3.14159265f) pose->theta += 2.0f * 3.14159265f;
    
    // 更新位置
    pose->x += linear_dist * cosf(pose->theta);
    pose->y += linear_dist * sinf(pose->theta);
}

/* ==================== 双轮速度控制器实现 ==================== */

void velocity_controller_init(VelocityController* ctrl,
                               float wheel_base, float wheel_radius,
                               float kp, float ki, float kd) {
    // 初始化运动学
    diff_drive_init(&ctrl->kinematics, wheel_base, wheel_radius);
    
    // 初始化左右轮PID
    pid_init(&ctrl->left_pid, kp, ki, kd, -100.0f, 100.0f);
    pid_init(&ctrl->right_pid, kp, ki, kd, -100.0f, 100.0f);
    
    // 设置死区（减少低速抖动）
    pid_set_deadband(&ctrl->left_pid, 0.01f);
    pid_set_deadband(&ctrl->right_pid, 0.01f);
    
    ctrl->left_speed = 0.0f;
    ctrl->right_speed = 0.0f;
    ctrl->left_pwm = 0.0f;
    ctrl->right_pwm = 0.0f;
    ctrl->enabled = true;
}

void velocity_controller_set_target(VelocityController* ctrl,
                                     float linear, float angular) {
    diff_drive_twist_to_wheels(&ctrl->kinematics, linear, angular,
                                &ctrl->left_speed, &ctrl->right_speed);
}

void velocity_controller_update(VelocityController* ctrl,
                                 int32_t left_pulses, int32_t right_pulses,
                                 float dt) {
    if (!ctrl->enabled) {
        ctrl->left_pwm = 0.0f;
        ctrl->right_pwm = 0.0f;
        return;
    }
    
    // 计算当前轮速
    float current_left = encoder_to_speed(left_pulses, dt, 
                                           ENCODER_PULSES_PER_REV, 
                                           ctrl->kinematics.wheel_radius);
    float current_right = encoder_to_speed(right_pulses, dt,
                                            ENCODER_PULSES_PER_REV,
                                            ctrl->kinematics.wheel_radius);
    
    // 计算PWM输出
    ctrl->left_pwm = pid_compute(&ctrl->left_pid, ctrl->left_speed, current_left, dt);
    ctrl->right_pwm = pid_compute(&ctrl->right_pid, ctrl->right_speed, current_right, dt);
}

void velocity_controller_get_pwm(VelocityController* ctrl,
                                  float* left_pwm, float* right_pwm) {
    *left_pwm = ctrl->left_pwm;
    *right_pwm = ctrl->right_pwm;
}

void velocity_controller_enable(VelocityController* ctrl, bool enabled) {
    ctrl->enabled = enabled;
    if (!enabled) {
        velocity_controller_reset(ctrl);
    }
}

void velocity_controller_reset(VelocityController* ctrl) {
    pid_reset(&ctrl->left_pid);
    pid_reset(&ctrl->right_pid);
    ctrl->left_pwm = 0.0f;
    ctrl->right_pwm = 0.0f;
}
