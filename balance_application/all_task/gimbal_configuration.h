#ifndef GIMBAL_CONFIGURATION_H
#define GIMBAL_CONFIGURATION_H

#include "c_data_typedef.h"
#include "dt7_remote_read.h"

/* ============================================================================
 * 命名与符号约定（第一阶段：pitch 轴 + 与 pitch 共用的量）
 *
 * 1. 物理量名称一律带单位后缀：
 *      _radian / _radian_per_second / _millimeter / _millimeter_per_second
 *      _revolution_per_minute / _degree_celsius / _count / _millisecond
 * 2. 不使用缩写。CAN / PID / LQR / IMU / USB 等标准协议、算法名按专有名词保留。
 * 3. 参数不再用 #define，改为带类型的常量（static const / enum），让编译器检查类型。
 * 4. 符号只在这里定义，其它文件不允许再出现裸的正负号。
 * ========================================================================== */

/* ---------------------------------------------------------------- 时间 */
static const uint32_t gimbal_initialization_delay_millisecond = 100U;
static const uint32_t gimbal_control_loop_period_millisecond = 1U;

/* ------------------------------------------------------ 数学与角度换算 */
static const float pi = 3.14159265358979f;

/* M3508 / GM6020：编码器一圈 8192，换算到转子机械角 */
static const float pitch_axis_encoder_count_to_rotor_radian = 0.000766990394f;

/* 9025：编码器一圈 65536，换算到关节角 */
static const float yaw_axis_encoder_count_to_radian = 0.0000958738f;

/* 转子转速 rpm -> 角速度 rad/s */
static const float motor_rotor_revolution_per_minute_to_radian_per_second = 0.1047197533333f;

enum { pitch_axis_encoder_count_per_revolution = 8192 };
enum { pitch_axis_half_encoder_count_range = 4096 };
enum { yaw_axis_encoder_count_per_revolution = 65536 };
enum { yaw_axis_half_encoder_count_range = 32768 };

/* 旧代码里按 16 位编码器写的解绕阈值（shoot 拨弹电机仍在用，保持原值） */
enum { encoder_count_half_range = 32768 };
enum { encoder_count_range = 65536 };
static const float motor_encoder_count_to_rotor_radian = 0.000766990394f;

/* ------------------------------------------------------------ 零点位置 */
enum { yaw_axis_zero_encoder_count = 6533 };
enum { yaw_axis_reversal_zero_encoder_count = 0 };
enum { yaw_axis_reversal_wait_cycle_count = 1600 };
enum { pitch_axis_zero_encoder_count = 1891 };

/* -------------------------------------------------------------- 遥控器 */
static const float yaw_axis_mouse_sensitivity_radian_per_count = 0.000008f;
static const float pitch_axis_mouse_sensitivity_radian_per_count = 0.000008f;
static const float yaw_axis_remote_control_sensitivity_radian_per_count = -0.000008f;
static const float pitch_axis_remote_control_sensitivity_radian_per_count = 0.000004f;

enum { yaw_axis_remote_control_channel = 0 };
enum { pitch_axis_remote_control_channel = 1 };
enum { remote_control_deadband_count = 10 };
enum { remote_control_left_switch = 1 };
enum { remote_control_right_switch = 0 };

static const uint16_t gimbal_heading_reversal_key = KEY_PRESSED_OFFSET_E;

/* ------------------------------------------------------------ 角度限幅 */
/* 编码器模式：pitch 转子角（不是关节角，见问题清单 A1/A2） */
static const float pitch_axis_relative_angle_maximum_radian = 0.1296f;
static const float pitch_axis_relative_angle_minimum_radian = -0.368f;

/* 绝对角度模式：IMU 关节角 */
static const float pitch_axis_absolute_angle_maximum_radian = 0.78f;
static const float pitch_axis_absolute_angle_minimum_radian = 0.00f;

/* ============================================================================
 * 符号约定（唯一换算点）
 *
 * 关节坐标系：抬头为正角度、正角速度。
 * 电机坐标系：C620 正电流为正转速（手册定义，不可更改）。
 *
 * pitch_axis_motor_rotation_to_joint_angle_sign：
 *     电机正转是否让关节角增大。+1 表示同向，-1 表示反向。
 * pitch_axis_inertial_measurement_angular_velocity_sign：
 *     IMU 陀螺仪 Y 轴读数是否与关节正角速度同向。
 * pitch_axis_screw_stick_direction_sign：
 *     遥控器俯仰杆向上时，丝杆速度环的目标转速取正还是取负。
 *
 * 这三个量互相独立：前者是机械安装方向，中间是 IMU 安装方向，
 * 后者是操作习惯。标定时只改这里；任何控制分支里不允许再写裸负号。
 * ========================================================================== */
static const float pitch_axis_motor_rotation_to_joint_angle_sign = -1.0f;
static const float pitch_axis_inertial_measurement_angular_velocity_sign = +1.0f;
static const float pitch_axis_screw_stick_direction_sign = +1.0f;

/* 反馈超时：超过该时间没有收到 C620 反馈就切断输出 */
static const uint32_t pitch_axis_feedback_timeout_millisecond = 20U;

/* 设定值速率限制：遥控器/鼠标/PC 目标每周期最多变化多少 */
static const float pitch_axis_setpoint_rate_limit_radian_per_second = 0.30f;

/* ============================================================================
 * 丝杆运动学（由实测/几何推导，单位统一为毫米与弧度）
 *     x^2 = C1 - C2 * cos(A + offset)
 *     A = acos((C1 - x^2) / C2) - offset
 * ========================================================================== */
static const float pitch_axis_kinematic_constant_1_millimeter_squared = 51984.1936f;
static const float pitch_axis_kinematic_constant_2_millimeter_squared = 51931.776f;
static const float pitch_axis_kinematic_angle_offset_radian = 1.033494f; /* 59.215 deg */
static const float pitch_axis_screw_length_minimum_millimeter = 121.49f;
static const float pitch_axis_screw_length_maximum_millimeter = 254.43f;
static const float pitch_axis_screw_length_at_zero_angle_millimeter = 159.39f;

/* 以下两个值待实测：断电转丝杆一整圈量导程，数电机圈数得减速比 */
static const float pitch_axis_screw_lead_millimeter_per_revolution = 8.0f;
static const float pitch_axis_motor_to_screw_reduction_ratio = 19.2f;

/* ============================================================================
 * 编码器模式 LQR（当前不可用，仅保留参数，见问题清单 A1）
 * ========================================================================== */
static const float pitch_axis_encoder_mode_speed_proportional_gain = 2000.0f;
static const float pitch_axis_encoder_mode_speed_integral_gain = 0.0f;
static const float pitch_axis_speed_loop_maximum_integral_output = 10000.0f;
static const float pitch_axis_speed_loop_maximum_output_count = 16384.0f;

static const float pitch_axis_encoder_mode_angle_proportional_gain = 100.0f;
static const float pitch_axis_encoder_mode_angle_integral_gain = 1.0f;
static const float pitch_axis_angle_loop_maximum_integral_output = 10000.0f;
static const float pitch_axis_angle_loop_maximum_output_count = 16384.0f;

static const float pitch_axis_encoder_mode_current_proportional_gain = 0.3f;
static const float pitch_axis_current_loop_maximum_output_count = 16384.0f;

/* 自瞄 LQR（当前只做初始化，未参与控制；保留原参数） */
static const float pitch_axis_automatic_aim_encoder_mode_speed_proportional_gain = 28000.0f;
static const float pitch_axis_automatic_aim_encoder_mode_speed_integral_gain = 0.0f;
static const float pitch_axis_automatic_aim_encoder_mode_angle_proportional_gain = 250000.0f;
static const float pitch_axis_automatic_aim_encoder_mode_angle_integral_gain = 500.0f;
static const float pitch_axis_automatic_aim_encoder_mode_current_proportional_gain = 0.3f;

/* ============================================================================
 * 丝杆速度环（电机坐标系：目标为转子 rpm，反馈为转子 rpm）
 * ========================================================================== */
static const float pitch_axis_screw_target_rotor_speed_revolution_per_minute = 3000.0f;
static const int16_t pitch_axis_screw_stick_threshold_count = 80;

static const float pitch_axis_screw_speed_proportional_gain = 2.0f;
static const float pitch_axis_screw_speed_integral_gain = 0.0f;
static const float pitch_axis_screw_speed_derivative_gain = 0.0f;
static const float pitch_axis_screw_speed_maximum_output_count = 8000.0f;
static const float pitch_axis_screw_speed_maximum_integral_output = 3000.0f;

/* ============================================================================
 * 绝对角度串级（陀螺仪模式，关节坐标系）
 * ========================================================================== */
static const float pitch_axis_absolute_angle_proportional_gain = 35.0f;
static const float pitch_axis_absolute_angle_integral_gain = 0.0f;
static const float pitch_axis_absolute_angle_derivative_gain = 0.0f;
static const float pitch_axis_absolute_angle_maximum_output_radian_per_second = 0.35f;
static const float pitch_axis_absolute_angle_maximum_integral_output = 2.1f;

static const float pitch_axis_absolute_speed_proportional_gain = 3000.0f;
static const float pitch_axis_absolute_speed_integral_gain = 0.0f;
static const float pitch_axis_absolute_speed_derivative_gain = 200.0f;
static const float pitch_axis_absolute_speed_maximum_output_count = 16000.0f;
static const float pitch_axis_absolute_speed_maximum_integral_output = 10000.0f;

/* ============================================================================
 * 自瞄串级（关节坐标系）
 * ========================================================================== */
static const float pitch_axis_automatic_aim_angle_proportional_gain = 40.5f;
static const float pitch_axis_automatic_aim_angle_integral_gain = 0.0f;
static const float pitch_axis_automatic_aim_angle_derivative_gain = 0.5f;
static const float pitch_axis_automatic_aim_angle_maximum_output_radian_per_second = 0.35f;
static const float pitch_axis_automatic_aim_angle_maximum_integral_output = 2.1f;

static const float pitch_axis_automatic_aim_speed_proportional_gain = 3500.0f;
static const float pitch_axis_automatic_aim_speed_integral_gain = 0.0f;
static const float pitch_axis_automatic_aim_speed_derivative_gain = 30.0f;
static const float pitch_axis_automatic_aim_speed_maximum_output_count = 30000.0f;
static const float pitch_axis_automatic_aim_speed_maximum_integral_output = 1000.0f;

#endif /* GIMBAL_CONFIGURATION_H */
