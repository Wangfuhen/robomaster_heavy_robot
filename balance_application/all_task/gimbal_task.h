/**
 * @file gimbal_task.h
 * @brief 云台控制任务头文件
 * @note 此文件定义了云台控制的所有宏定义、枚举类型和数据结构
 *       包含云台控制参数、电机参数、PID参数等
 */

#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

/*
                                                                                                                                                                    
                                                                                                                                                                                   
                         ||                  |||                                                                                              |||                   |||||||||      
                         ||           ||||||||||||||       ||||||||||||||||||        |||||||                             ||||||||      ||||||||||||||            |||||||||||||||   
                        ||||           ||||||||||||||       |||||||||||||||||      ||||||||||                          ||||||||||||     ||||||||||||||          |||||      |||||   
                        ||||             ||||   ||||||        ||||        |||     ||||   ||||                         |||||    ||||       ||||   ||||||        ||||          |||   
                        ||||             ||||    |||||        ||||         ||     |||      ||                        ||||        |||      ||||    |||||       ||||            ||   
                       ||||||            ||||     ||||        ||||         ||    ||||      ||                       ||||         |||      ||||     ||||      ||||             ||   
                       || |||            ||||     ||||        ||||               ||||      ||                      ||||           ||      ||||     ||||      ||||                  
                      ||| |||            ||||     ||||        ||||       ||       ||||                             ||||                   ||||     ||||     |||||                  
                      ||  ||||           ||||    ||||         ||||       ||       ||||||                           ||||                   ||||    ||||      ||||                   
                      ||   |||           ||||   ||||          ||||       ||        |||||||                        ||||                    ||||   ||||       ||||                   
                     |||   ||||          ||||||||||           |||||||||||||         ||||||||                      ||||                    ||||||||||        ||||                   
                     ||||||||||          ||||  ||||           |||||||||||||           |||||||                     ||||                    ||||  ||||        ||||        |||||||||||
                    |||||||||||          ||||   ||||          ||||       ||             ||||||                    ||||                    ||||   ||||       ||||        |||||||||||
                    |||     ||||         ||||   ||||          ||||       ||               ||||                    ||||                    ||||   ||||       |||||           ||||   
                    ||      ||||         ||||    ||||         ||||       ||      ||        |||                    ||||                    ||||    ||||       ||||           ||||   
                   |||       |||         ||||    |||||        ||||        |  ||  |||       |||                     ||||                   ||||    |||||      |||||          ||||   
                   ||        ||||        ||||     ||||        ||||          |||  |||       |||                     ||||           ||      ||||     ||||      |||||          ||||   
                   ||         |||        ||||     |||||       ||||          |||  ||||      |||                      ||||          ||      ||||     |||||      |||||         ||||   
                  |||         ||||       |||||     ||||||     ||||         |||   ||||||  ||||                       ||||         |||      |||||     ||||||     ||||||      |||||   
               |||||||||   |||||||||  ||||||||||   ||||||| |||||||||||||||||||    ||||||||||                         |||||     ||||    ||||||||||   |||||||      ||||||||||||||      
               |||||||||   |||||||||  ||||||||||    |||||| |||||||||||||||||||      ||||||                             ||||||||||||    ||||||||||    ||||||        |||||||||       
                                                                                                                         ||||||||                                                  
                                                                                                                                                                                   
                                                                                              ||||||||||||||||                                                                                                                                                                   
 
*/

#include "c_data_typedef.h"
#include "main.h"
#include "can_user.h"
#include "pid.h"
#include "usb_task.h"
#include "ins_task.h"
#include "dt7_remote_read.h"
#include "joint_motor_lqr.h"
#include "cmsis_os.h"

#include "gimbal_configuration.h"

// ============================ YAW轴控制参数（普通模式） ============================
// 编码器角度控制（相对角度）
#define YAW_ANGLE_KP_RELATIVE       60.0f      // 角度环比例系数
#define YAW_ANGLE_KI_RELATIVE       20.0f      // 角度环积分系数
#define YAW_ANGLE_KD_RELATIVE       100.0f     // 角度环微分系数
#define YAW_ANGLE_MAX_RELATIVE      2000.0f    // 角度环最大输出
#define YAW_ANGLE_I_MAX_RELATIVE    0.5f       // 角度环积分限幅

// 编码器速度控制（相对速度）
#define YAW_SPEED_KP_RELATIVE       6000.0f    // 速度环比例系数
#define YAW_SPEED_KI_RELATIVE       0.0f       // 速度环积分系数
#define YAW_SPEED_KD_RELATIVE       100.0f     // 速度环微分系数
#define YAW_SPEED_MAX_RELATIVE      30000.0f   // 速度环最大输出
#define YAW_SPEED_I_MAX_RELATIVE    200.0f     // 速度环积分限幅

// 编码器电流控制（相对模式）
#define YAW_CURRENT_KP_RELATIVE     0          // 电流环比例系数（未使用）
#define YAW_CURRENT_MAX_RELATIVE    30000      // 电流环最大输出

// ============================ YAW轴控制参数（陀螺仪模式） ============================
// 陀螺仪角度控制（绝对角度）
#define YAW_ANGLE_KP       20.0f               // 角度环比例系数
#define YAW_ANGLE_KI       0.0f                // 角度环积分系数
#define YAW_ANGLE_KD       5.0f                // 角度环微分系数
#define YAW_ANGLE_MAX      6.0f                // 角度环最大输出
#define YAW_ANGLE_I_MAX    0.5f                // 角度环积分限幅

// 陀螺仪速度控制（角速度）
#define YAW_SPEED_KP       1300.0f             // 速度环比例系数
#define YAW_SPEED_KI       0.0f                // 速度环积分系数
#define YAW_SPEED_KD       100.0f              // 速度环微分系数
#define YAW_SPEED_MAX      30000.0f            // 速度环最大输出
#define YAW_SPEED_I_MAX    200.0f              // 速度环积分限幅

// 陀螺仪电流控制
#define YAW_CURRENT_KP     0                   // 电流环比例系数（未使用）
#define YAW_CURRENT_MAX    30000               // 电流环最大输出

// ============================ YAW轴控制参数（自瞄模式） ============================
// 自瞄角度控制
#define YAW_ANGLE_KP_AC       20.0f            // 自瞄角度环比例系数
#define YAW_ANGLE_KI_AC       0.0f             // 自瞄角度环积分系数
#define YAW_ANGLE_KD_AC       5.0f             // 自瞄角度环微分系数
#define YAW_ANGLE_AC_MAX      6.0f             // 自瞄角度环最大输出
#define YAW_ANGLE_I_AC_MAX    0.5f             // 自瞄角度环积分限幅

// 自瞄速度控制
#define YAW_SPEED_KP_AC       1300.0f          // 自瞄速度环比例系数
#define YAW_SPEED_KI_AC       0.0f             // 自瞄速度环积分系数
#define YAW_SPEED_KD_AC       300.0f           // 自瞄速度环微分系数
#define YAW_SPEED_AC_MAX      30000.0f         // 自瞄速度环最大输出
#define YAW_SPEED_I_AC_MAX    200.0f           // 自瞄速度环积分限幅

// 自瞄电流控制
#define YAW_CURRENT_KP_AC     0                // 自瞄电流环比例系数（未使用）
#define YAW_CURRENT_AC_MAX    30000            // 自瞄电流环最大输出

// ============================ 数据结构定义 ============================

/**
 * @brief 云台控制模式枚举
 * @note 定义云台的不同工作模式
 */
typedef enum
{
	GIMBAL_OPERATING_MODE_ZERO_FORCE,  // 云台无力模式
	GIMBAL_OPERATING_MODE_ABSOLUTE_ANGLE,        // 云台绝对角度控制模式（基于陀螺仪）
	GIMBAL_OPERATING_MODE_ENCODER_ANGLE,     // 云台相对角度控制模式（基于编码器）
	GIMBAL_OPERATING_MODE_AUTOMATIC_AIM,        // 云台自动瞄准模式
} gimbal_operating_mode_t;

/**
 * @brief 电机控制模式枚举
 * @note 定义电机的不同控制模式
 */
typedef enum
{
	MOTOR_CONTROL_MODE_ZERO_FORCE,  // 电机无力控制
	MOTOR_CONTROL_MODE_ABSOLUTE_ANGLE,        // 电机陀螺仪控制（绝对角度）
	MOTOR_CONTROL_MODE_ENCODER_ANGLE,     // 电机编码器控制（相对角度）
} motor_control_mode_t;

/**
 * @brief 云台跟随状态枚举
 * @note 定义云台的朝向状态
 * @note【已停用】一键掉头至正位：机器人加装机械臂后不再支持该模式，
 *       GIMBAL_HEADING_REVERSED 仅为兼容底盘通信协议而保留，运行中不会再被使用。
 */
typedef enum
{
	GIMBAL_HEADING_FORWARD,   // 正常前向状态（掉头停用后恒为该值）
    GIMBAL_HEADING_REVERSED,  // 掉头前向状态（【已停用】加装机械臂后不再支持）
} gimbal_heading_state_t;

/**
 * @brief 云台电机控制数据结构体
 * @note 包含云台单个电机的所有控制参数和状态
 */
typedef struct
{  
    const motor_measurement_t *motor_measurement_pointer;      // 电机测量数据指针
    motor_control_mode_t motor_control_mode;            // 电机当前控制模式
    motor_control_mode_t previous_motor_control_mode;       // 电机上一次控制模式
    
    // 控制器
	lqr_type_def encoder_mode_controller;                                 // LQR控制器（PITCH用）
	lqr_type_def automatic_aim_encoder_mode_controller;                              // 自瞄LQR控制器
	pid_type_def speed_controller;                    // 速度环PID控制器
    pid_type_def screw_speed_controller;              /* screw pitch speed loop, rpm */
	pid_type_def angle_controller;                    // 角度环PID控制器
	pid_type_def relative_speed_controller;           // 相对速度环PID控制器
	pid_type_def relative_angle_controller;           // 相对角度环PID控制器
	pid_type_def automatic_aim_speed_controller;                 // 自瞄速度环PID控制器
	pid_type_def automatic_aim_angle_controller;                 // 自瞄角度环PID控制器
    
    // 角度相关
    fp32 relative_angle_radian;                              // 相对角度（弧度）- 编码器
    fp32 maximum_relative_angle_radian;                          // 最大相对角度限制
	fp32 minimum_relative_angle_radian;                          // 最小相对角度限制
	fp32 relative_angle_setpoint_radian;                          // 相对角度设定值
	fp32 relative_angular_velocity_radian_per_second;                              // 相对角速度（电机转速转换）
	fp32 relative_angular_velocity_setpoint_radian_per_second;                          // 相对角速度设定值（串级控制用）
    fp32 pitch_axis_screw_target_rotor_speed_revolution_per_minute;  // pitch 丝杆速度环目标（电机坐标系，不做关节换向）
    
    // 陀螺仪相关
    fp32 absolute_angle_radian;                              // 绝对角度（弧度）- 陀螺仪
	fp32 absolute_angular_velocity_radian_per_second;                               // 绝对角速度（陀螺仪测量）
	fp32 absolute_angular_velocity_setpoint_radian_per_second;                           // 绝对角速度设定值（串级控制用）
    fp32 maximum_absolute_angle_radian;                          // 最大绝对角度限制
	fp32 minimum_absolute_angle_radian;                          // 最小绝对角度限制
	fp32 absolute_angle_setpoint_radian;                          // 绝对角度设定值
    
    // 电流控制
	fp32 commanded_torque_current_count;                                // 计算得到的电流发送值
	fp32 measured_torque_current_count;                            // 反馈得到的扭矩电流
    
    // 编码器相关
	uint16_t zero_encoder_count;                       // 相对角度零点（编码器值）
	forward_feed angle_feedforward_calculation;                     // 角度前馈计算
} gimbal_axis_state_t;

/**
 * @brief 云台状态切换数据结构体
 * @note 用于管理云台的状态切换和掉头逻辑
 * @note【已停用】掉头逻辑：机器人加装机械臂后不再支持“掉头至正位”，
 *       gimbal_task.c 中相关解析与角度补偿已注释，本结构体仅保留字段与初始化，
 *       heading_state / previous_heading_state 恒为 GIMBAL_HEADING_FORWARD，
 *       各切换标志与计时器恒为 0。
 */
typedef struct
{
	gimbal_heading_state_t heading_state;       // 当前云台方向状态（【已停用】恒为前向）
	gimbal_heading_state_t previous_heading_state;  // 上一次云台方向状态（【已停用】恒为前向）
	uint8_t  heading_state_change_flag;                      // 云台方向切换标志（【已停用】恒为 0）
	uint32_t heading_state_change_time_millisecond;                      // 云台方向切换计时器（【已停用】不再更新）
	uint32_t heading_state_chassis_time_millisecond;              // 云台与底盘切换计时器（【已停用】不再更新）
	uint8_t  heading_state_chassis_change_flag;             // 云台与底盘切换标志（【已停用】恒为 0）
} gimbal_heading_transition_t;

/**
 * @brief 云台控制主结构体
 * @note 包含云台的所有控制参数、状态和控制器
 */
typedef struct
{
	gimbal_axis_state_t pitch_axis_state;                 // PITCH轴电机控制数据
    gimbal_axis_state_t yaw_axis_state;                   // YAW轴电机控制数据
    
    // 传感器数据指针
	const fp32 *gimbal_INS_angle_point;                // IMU角度数据指针
	const fp32 *gimbal_INS_gyro_point;                 // IMU角速度数据指针
	const RC_ctrl_t *gimbal_dt7_remote_control;        // 遥控器数据指针
	const autom_pc *hero_pc;                           // PC通信数据指针
    
    // 云台状态
	gimbal_operating_mode_t gimbal_mode;                         // 当前云台控制模式
	gimbal_operating_mode_t gimbal_mode_last;                    // 上一次云台控制模式
	gimbal_heading_transition_t heading_transition;               // 云台角度变化状态
} gimbal_control_t;

// ============================ 函数声明 ============================

/**
 * @brief 获取云台控制结构体指针
 * @return 云台控制结构体指针
 */
extern gimbal_control_t *GetGimbalControlStatePointer(void);

/**
 * @brief 自动模式标志（前哨站）
 * @note 用于模式切换的状态管理
 */
extern uint8_t auto_mode_flag_qian;

/**
 * @brief 自动模式标志
 * @note 当前自动模式状态
 */
extern uint8_t auto_mode_flag;

#endif /* GIMBAL_TASK_H */


