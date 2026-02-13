/**
 * @file pid_controller.h
 * @brief PID控制器实现 - 支持差速驱动机器人的速度和角度控制
 * 
 * 该模块实现了高性能的PID控制器，用于电机速度控制和航向角控制。
 * 包含抗积分饱和、死区处理和输出限幅功能。
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "common_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== PID控制器API ==================== */

/**
 * @brief 初始化PID控制器
 * @param pid PID控制器指针
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 * @param output_min 输出下限
 * @param output_max 输出上限
 */
void pid_init(PIDController* pid, float kp, float ki, float kd,
              float output_min, float output_max);

/**
 * @brief 重置PID控制器状态
 * @param pid PID控制器指针
 */
void pid_reset(PIDController* pid);

/**
 * @brief 设置PID参数
 * @param pid PID控制器指针
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 */
void pid_set_params(PIDController* pid, float kp, float ki, float kd);

/**
 * @brief 设置输出限幅
 * @param pid PID控制器指针
 * @param min 输出下限
 * @param max 输出上限
 */
void pid_set_limits(PIDController* pid, float min, float max);

/**
 * @brief 设置死区
 * @param pid PID控制器指针
 * @param deadband 死区值
 */
void pid_set_deadband(PIDController* pid, float deadband);

/**
 * @brief 计算PID输出
 * @param pid PID控制器指针
 * @param setpoint 设定值
 * @param measurement 测量值
 * @param dt 时间间隔(s)
 * @return 控制输出
 */
float pid_compute(PIDController* pid, float setpoint, 
                  float measurement, float dt);

/**
 * @brief 计算PID输出（带前馈）
 * @param pid PID控制器指针
 * @param setpoint 设定值
 * @param measurement 测量值
 * @param feedforward 前馈值
 * @param dt 时间间隔(s)
 * @return 控制输出
 */
float pid_compute_with_ff(PIDController* pid, float setpoint,
                          float measurement, float feedforward, float dt);

/* ==================== 差速驱动运动学 ==================== */

/**
 * @brief 差速驱动运动学结构
 */
typedef struct {
    float wheel_base;       // 轮距(m)
    float wheel_radius;     // 轮半径(m)
    float max_wheel_speed;  // 最大轮速(rad/s)
    float max_linear;       // 最大线速度(m/s)
    float max_angular;      // 最大角速度(rad/s)
} DiffDriveKinematics;

/**
 * @brief 初始化差速驱动运动学
 * @param kin 运动学结构指针
 * @param wheel_base 轮距(m)
 * @param wheel_radius 轮半径(m)
 */
void diff_drive_init(DiffDriveKinematics* kin, float wheel_base, float wheel_radius);

/**
 * @brief 速度指令转换为左右轮速度
 * @param kin 运动学结构指针
 * @param linear 线速度(m/s)
 * @param angular 角速度(rad/s)
 * @param left_speed 输出左轮速度(m/s)
 * @param right_speed 输出右轮速度(m/s)
 */
void diff_drive_twist_to_wheels(const DiffDriveKinematics* kin,
                                 float linear, float angular,
                                 float* left_speed, float* right_speed);

/**
 * @brief 左右轮速度转换为速度指令
 * @param kin 运动学结构指针
 * @param left_speed 左轮速度(m/s)
 * @param right_speed 右轮速度(m/s)
 * @param linear 输出线速度(m/s)
 * @param angular 输出角速度(rad/s)
 */
void diff_drive_wheels_to_twist(const DiffDriveKinematics* kin,
                                 float left_speed, float right_speed,
                                 float* linear, float* angular);

/**
 * @brief 编码器脉冲转换为轮速
 * @param pulses 编码器脉冲数
 * @param dt 时间间隔(s)
 * @param pulses_per_rev 每转脉冲数
 * @param wheel_radius 轮半径(m)
 * @return 轮速(m/s)
 */
float encoder_to_speed(int32_t pulses, float dt, 
                       uint16_t pulses_per_rev, float wheel_radius);

/**
 * @brief 计算里程计增量
 * @param left_pulses 左轮脉冲增量
 * @param right_pulses 右轮脉冲增量
 * @param kin 运动学结构指针
 * @param pulses_per_rev 每转脉冲数
 * @param pose 当前位姿（会被更新）
 */
void odometry_update(int32_t left_pulses, int32_t right_pulses,
                     const DiffDriveKinematics* kin,
                     uint16_t pulses_per_rev, Pose2D* pose);

/* ==================== 双PID速度控制器 ==================== */

/**
 * @brief 双轮速度控制器
 */
typedef struct {
    PIDController left_pid;
    PIDController right_pid;
    DiffDriveKinematics kinematics;
    float left_speed;
    float right_speed;
    float left_pwm;
    float right_pwm;
    bool enabled;
} VelocityController;

/**
 * @brief 初始化速度控制器
 * @param ctrl 控制器指针
 * @param wheel_base 轮距(m)
 * @param wheel_radius 轮半径(m)
 * @param kp P增益
 * @param ki I增益
 * @param kd D增益
 */
void velocity_controller_init(VelocityController* ctrl,
                               float wheel_base, float wheel_radius,
                               float kp, float ki, float kd);

/**
 * @brief 设置目标速度
 * @param ctrl 控制器指针
 * @param linear 线速度(m/s)
 * @param angular 角速度(rad/s)
 */
void velocity_controller_set_target(VelocityController* ctrl,
                                     float linear, float angular);

/**
 * @brief 更新速度控制器（需要周期调用）
 * @param ctrl 控制器指针
 * @param left_pulses 左轮编码器增量
 * @param right_pulses 右轮编码器增量
 * @param dt 时间间隔(s)
 */
void velocity_controller_update(VelocityController* ctrl,
                                 int32_t left_pulses, int32_t right_pulses,
                                 float dt);

/**
 * @brief 获取PWM输出
 * @param ctrl 控制器指针
 * @param left_pwm 输出左轮PWM(-100~100)
 * @param right_pwm 输出右轮PWM(-100~100)
 */
void velocity_controller_get_pwm(VelocityController* ctrl,
                                  float* left_pwm, float* right_pwm);

/**
 * @brief 使能/禁用控制器
 * @param ctrl 控制器指针
 * @param enabled 使能标志
 */
void velocity_controller_enable(VelocityController* ctrl, bool enabled);

/**
 * @brief 重置控制器状态
 * @param ctrl 控制器指针
 */
void velocity_controller_reset(VelocityController* ctrl);

#ifdef __cplusplus
}
#endif

#endif /* PID_CONTROLLER_H */
