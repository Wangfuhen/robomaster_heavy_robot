#ifndef CHASSIS_CONFIGURATION_H
#define CHASSIS_CONFIGURATION_H

#include <stdint.h>
#include "dt7_remote_read.h"

/* 控制周期与初始化 */
static const uint32_t chassis_control_loop_period_millisecond = 2U;
static const uint32_t chassis_initialization_delay_millisecond = 100U;

/* 电机、电流与编码器 */
static const float motor_maximum_command_current_count = 16384.0f;
static const float current_count_to_ampere = 0.0012207031250f;
static const float motor_encoder_count_to_radian = 0.0000958738f;
static const int32_t motor_encoder_count_per_revolution = 65536;
static const int32_t motor_encoder_half_count_range = 32768;
static const float revolution_per_minute_to_radian_per_second = 0.1047197533333f;
static const float radian_to_degree_scale = 57.2957795f;

/* 遥控器与键盘 */
static const int32_t remote_control_deadband_count = 10;
static const uint8_t forward_speed_remote_control_channel = 3U;
static const uint8_t leftward_speed_remote_control_channel = 2U;
static const uint8_t rotation_speed_remote_control_channel = 1U;
static const uint8_t remote_control_switch_left = 1U;
static const uint8_t remote_control_switch_right = 0U;
static const int16_t remote_control_action_threshold_count = 80;
static const int16_t remote_control_forward_sensitivity_count = 48;
static const int16_t remote_control_leftward_sensitivity_count = 48;

static const uint8_t keyboard_forward_key_mask = (uint8_t)(1U << 0);
static const uint8_t keyboard_backward_key_mask = (uint8_t)(1U << 1);
static const uint8_t keyboard_rightward_key_mask = (uint8_t)(1U << 0);
static const uint8_t keyboard_leftward_key_mask = (uint8_t)(1U << 1);
static const uint8_t super_capacitor_key = KEY_PRESSED_OFFSET_SHIFT;
/* 【已停用】小陀螺开关键：机器人加装机械臂后不再支持小陀螺模式，
   对应遥控器解析已注释（云台侧 chassis_data_task.c 与底盘侧 CAN_ParseGimbalCommand），
   此常量当前无引用，仅保留备查。 */
static const uint8_t top_spin_key = KEY_PRESSED_OFFSET_C;

/* 麦轮运动学 */
static const float motor_rotor_speed_to_forward_speed_scale = 0.25f;
static const float motor_rotor_speed_to_leftward_speed_scale = 0.25f;
static const float motor_rotor_speed_to_rotation_speed_scale = 0.25f;
static const float motor_distance_to_chassis_center_meter = 0.364f;

/* 云台跟随 */
static const float chassis_follow_gimbal_angle_normal_radian = 0.0f;
/* 【已停用】一键掉头至正位：机器人加装机械臂后不再支持该模式，掉头角度设定不再使用 */
static const float chassis_follow_gimbal_angle_turnaround_radian = 0.0f;
static const float chassis_follow_gimbal_forward_proportional_gain = 0.07f;

/* 底盘速度环 */
static const float chassis_speed_loop_proportional_gain = 20.0f;
static const float chassis_speed_loop_integral_gain = 0.15f;
static const float chassis_speed_loop_derivative_gain = 10.0f;
static const float chassis_speed_loop_maximum_output_count = 16384.0f;
static const float chassis_speed_loop_maximum_integral_output_count = 2000.0f;

/* 云台跟随角度环 */
static const float chassis_follow_gimbal_angle_loop_proportional_gain = 28000.0f;
static const float chassis_follow_gimbal_angle_loop_integral_gain = 0.0f;
static const float chassis_follow_gimbal_angle_loop_derivative_gain = 3000.0f;
static const float chassis_follow_gimbal_angle_loop_maximum_output_count = 30000.0f;
static const float chassis_follow_gimbal_angle_loop_maximum_integral_output_count = 1500.0f;

/* 功率缓冲环 */
static const float chassis_power_buffer_loop_proportional_gain = -2.5f;
static const float chassis_power_buffer_loop_integral_gain = -0.001f;
static const float chassis_power_buffer_loop_derivative_gain = -0.0f;
static const float chassis_power_buffer_loop_maximum_output_count = 60.0f;
static const float chassis_power_buffer_loop_maximum_integral_output_count = 10.0f;

/* 速度上限 */
static const float chassis_forward_speed_maximum_remote_control_revolution_per_minute = 3300.0f;
static const float chassis_leftward_speed_maximum_remote_control_revolution_per_minute = 3300.0f;
static const float chassis_forward_speed_maximum_keyboard_revolution_per_minute = 8000.0f;
static const float chassis_leftward_speed_maximum_keyboard_revolution_per_minute = 6000.0f;
static const float chassis_rotation_speed_maximum_super_capacitor_on_revolution_per_minute = 26000.0f;
static const float chassis_rotation_speed_maximum_super_capacitor_off_revolution_per_minute = 26000.0f;

/* 功率控制 */
static const uint8_t chassis_power_test_enabled = 1U;
static const float chassis_power_fit_coefficient_0 = 0.6641993412640775f;
static const float chassis_power_fit_coefficient_1 = 0.006444284468539646f;
static const float chassis_power_fit_coefficient_2 = 0.0001423857226262331f;
static const float chassis_power_fit_coefficient_3 = 0.017644430204543864f;
static const float chassis_power_fit_coefficient_4 = 0.1650143850678085f;
static const float chassis_power_fit_coefficient_5 = 3.096721772539512e-05f;

static const float chassis_power_buffer_usage_maximum_without_gain_watt = 40.0f;
static const float chassis_power_buffer_usage_maximum_with_gain_watt = 200.0f;
static const float chassis_power_buffer_limit_without_gain_watt = 60.0f;
static const float chassis_power_buffer_limit_with_gain_watt = 250.0f;
static const float chassis_power_buffer_recovery_proportional_gain = 0.5f;
static const float chassis_power_action_attenuation_gain = 1.0f;
static const float chassis_power_forward_gain = 0.4f;
static const float chassis_power_leftward_gain = 0.3f;
static const float chassis_power_rotation_gain = 0.3f;

/* 低通与斜坡 */
static const float chassis_forward_speed_low_pass_filter_alpha = 0.08f;
static const float chassis_leftward_speed_low_pass_filter_alpha = 0.08f;
static const float chassis_rotation_speed_low_pass_filter_alpha = 0.02f;

static const float chassis_forward_speed_acceleration_rate = 8.5f;
static const float chassis_forward_speed_deceleration_rate = 19.5f;
static const float chassis_leftward_speed_acceleration_rate = 500.5f;
static const float chassis_leftward_speed_deceleration_rate = 1000.5f;
static const float chassis_rotation_speed_acceleration_rate = 5.5f;
static const float chassis_rotation_speed_deceleration_rate = 1.5f;

/* 云台掉头 */
/* 【已停用】一键掉头至正位：机器人加装机械臂后不再支持该模式，
   下面掉头等待时间与掉头零位编码器值仅在恢复掉头逻辑时才会用到，当前无引用；
   gimbal_yaw_zero_encoder_count 仍是正常跟随方向使用的云台零位，保持不变。 */
static const uint32_t gimbal_turnaround_wait_time_millisecond = 1600U;
static const uint16_t gimbal_yaw_zero_encoder_count = 16190U;
static const uint16_t gimbal_yaw_turnaround_encoder_count = 48929U;

#endif
