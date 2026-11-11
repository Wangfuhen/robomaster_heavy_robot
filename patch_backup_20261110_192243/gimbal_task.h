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

// ============================ 时间常量 ============================
#define HERO_GIMBAL_INIT_TIME 100              // 云台初始化延时时间（ms）
#define GIMBAL_LOOP_TIME      1                // 云台控制循环周期（ms）

// ============================ 数学常量 ============================
#define MOTOR_ECD_TO_RAD      0.000766990394f  // 6020电机编码器转弧度系数 (2*PI/8192)
#define PI	                  3.14159265358979f // 圆周率
#define ECD_TO_RAD            0.0000958738f    // 9025电机编码器转弧度系数 (2*PI/65536)

// ============================ 电机参数 ============================
#define MOTOR_T_k             0.741            // 6020电机带1:19减速箱的扭矩常数 (N·m/A)
#define MOTOR_MA_TO_A         5461.3333        // 6020电机电流转换系数（毫安转安培）
#define MOTOR_RPM_RADS        0.1047197533333  // 转速转角速度系数 (rad/s) - 所有电机通用
#define _MOTOR_T_k            0                // 9025电机带1:19减速箱的扭矩常数 (N·m/A)（待设置）
#define _MOTOR_MA_TO_A        0                // 9025电机电流转换系数（毫安转安培）（待设置）

// ============================ 编码器参数 ============================
#define HALF_ECD_RANGE        32768            // 半编码器范围
#define ECD_RANGE             65536            // 编码器总范围

// ============================ 云台零点设置 ============================
#define YAW_ZERO_ANGLE_SET    6533             // YAW轴零点位置（9025电机，待设置）
#define YAW_CHANGE_ANGLE_SET  0            // YAW轴掉头零点位置（9025电机，待设置）
#define YAW_CHANGE_WAIT_TIME  1600             // 掉头等待时间（控制周期数）
#define PITCH_ZERO_ANGLE_SET  1891             // PITCH轴零点位置

// ============================ 遥控器参数 ============================
// 鼠标灵敏度
#define YAW_MOUSE_SEN         0.000008f        // YAW轴鼠标灵敏度
#define PITCH_MOUSE_SEN       0.000008f        // PITCH轴鼠标灵敏度

// 遥控器灵敏度
#define YAW_RC_SEN           -0.000008f        // YAW轴遥控器灵敏度（负号表示方向）
#define PITCH_RC_SEN          0.000004f        // PITCH轴遥控器灵敏度

// 遥控器通道定义
#define YAW_CHANNEL           0                // YAW控制通道
#define PITCH_CHANNEL         1                // PITCH控制通道

// 遥控器死区
#define RC_DEADBAND           10               // 遥控器通道死区值

// 拨杆定义
#define LEFT                  1                // 左拨杆
#define RIGHT                 0                // 右拨杆

// ============================ 按键定义 ============================
#define CHANGE_GIMBAL_FORWARD KEY_PRESSED_OFFSET_E  // 一键掉头按键

// ============================ 云台角度限制 ============================
// PITCH轴相对角度限制（编码器角度）
#define GIMBAL_PITCH_RELATIVE_MAX_ANGLE   0.1296f     // PITCH相对角度上限（弧度）
#define GIMBAL_PITCH_RELATIVE_MIN_ANGLE  -0.368f      // PITCH相对角度下限（弧度）

// PITCH轴绝对角度限制（陀螺仪角度）
#define GIMBAL_PITCH_ABSOULATE_MAX_ANGLE  0.78f       // PITCH绝对角度上限（弧度）
#define GIMBAL_PITCH_ABSOULATE_MIN_ANGLE  0.00f       // PITCH绝对角度下限（弧度）

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

// ============================ PITCH轴控制参数（普通模式） ============================
// LQR控制参数 - 速度环
#define PITCH_SPEED_KP      2000               // 速度环比例系数
#define PITCH_SPEED_KI      0                  // 速度环积分系数
#define PITCH_SPEED_I_MAX   10000              // 速度环积分限幅
#define PITCH_SPEED_MAX     16384              // 速度环最大输出

// LQR控制参数 - 角度环
#define PITCH_ANGLE_KP      100                // 角度环比例系数
#define PITCH_ANGLE_KI      1                  // 角度环积分系数
#define PITCH_ANGLE_I_MAX   10000              // 角度环积分限幅
#define PITCH_ANGLE_MAX     16384              // 角度环最大输出

// LQR控制参数 - 电流环
#define PITCH_CURRENT_KP    0.3                // 电流环比例系数
#define PITCH_CURRENT_MAX   16384              // 电流环最大输出

// ============================ PITCH轴控制参数（自瞄模式） ============================
// LQR控制参数 - 自瞄速度环
#define PITCH_SPEED_KP_AC   28000              // 自瞄速度环比例系数
#define PITCH_SPEED_KI_AC   0                  // 自瞄速度环积分系数

// LQR控制参数 - 自瞄角度环
#define PITCH_ANGLE_KP_AC   250000             // 自瞄角度环比例系数
#define PITCH_ANGLE_KI_AC   500                // 自瞄角度环积分系数

// LQR控制参数 - 自瞄电流环
#define PITCH_CURRENT_KP_AC 0.3                // 自瞄电流环比例系数

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

// ============================ PITCH轴串级PID参数（非自瞄） ============================
// 角度环参数
#define PITCH_GYRO_ABSOLUTE_PID_KP       35.0f      // 陀螺仪角度环比例系数
#define PITCH_GYRO_ABSOLUTE_PID_KI        0.0f      // 陀螺仪角度环积分系数
#define PITCH_GYRO_ABSOLUTE_PID_KD        0.0f      // 陀螺仪角度环微分系数
#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT   300.0f    // 陀螺仪角度环最大输出
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT  2.1f      // 陀螺仪角度环积分限幅

// 速度环参数
#define PITCH_ABSOLUTE_SPEED_PID_KP        3000.0f   // 绝对速度环比例系数
#define PITCH_ABSOLUTE_SPEED_PID_KI        0.0f     // 绝对速度环积分系数
#define PITCH_ABSOLUTE_SPEED_PID_KD        200.0f     // 绝对速度环微分系数
#define PITCH_ABSOLUTE_SPEED_PID_MAX_OUT   16000.0f // 绝对速度环最大输出
#define PITCH_ABSOLUTE_SPEED_PID_MAX_IOUT  10000.0f // 绝对速度环积分限幅

// ============================ PITCH轴串级PID参数（自瞄） ============================
// 自瞄角度环参数
#define PITCH_GYRO_ABSOLUTE_PID_KP_AC        40.5f    // 自瞄陀螺仪角度环比例系数
#define PITCH_GYRO_ABSOLUTE_PID_KI_AC        0.0f     // 自瞄陀螺仪角度环积分系数
#define PITCH_GYRO_ABSOLUTE_PID_KD_AC        0.50f    // 自瞄陀螺仪角度环微分系数
#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT_AC   30.0f    // 自瞄陀螺仪角度环最大输出
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT_AC  2.1f     // 自瞄陀螺仪角度环积分限幅

// 自瞄速度环参数
#define PITCH_ABSOLUTE_SPEED_PID_KP_AC        3500.0f  // 自瞄绝对速度环比例系数
#define PITCH_ABSOLUTE_SPEED_PID_KI_AC        0.0f     // 自瞄绝对速度环积分系数
#define PITCH_ABSOLUTE_SPEED_PID_KD_AC        30.0f    // 自瞄绝对速度环微分系数
#define PITCH_ABSOLUTE_SPEED_PID_MAX_OUT_AC   30000.0f // 自瞄绝对速度环最大输出
#define PITCH_ABSOLUTE_SPEED_PID_MAX_IOUT_AC  1000.0f  // 自瞄绝对速度环积分限幅

// ============================ 数据结构定义 ============================

/**
 * @brief DM电机控制数据结构体
 * @note 用于直流电机的控制参数
 */
typedef struct
{
	int8_t id;               // 电机ID
	uint8_t start_flag;      // 启动标志
    int8_t mode;             // 控制模式
	float vel_set;           // 速度设定值
	float tor_set;           // 扭矩设定值
	float kp_set;            // 比例系数设定值
	float kd_set;            // 微分系数设定值
} dm_motor_control;

/**
 * @brief 云台控制模式枚举
 * @note 定义云台的不同工作模式
 */
typedef enum
{
	GIMBAL_ZERO_FORCE,  // 云台无力模式
	GIMBAL_GYRO,        // 云台绝对角度控制模式（基于陀螺仪）
	GIMBAL_ENCONDE,     // 云台相对角度控制模式（基于编码器）
	GIMBAL_AUTO,        // 云台自动瞄准模式
} gimbal_mode_e;

/**
 * @brief 电机控制模式枚举
 * @note 定义电机的不同控制模式
 */
typedef enum
{
	MOTOR_ZERO_FORCE,  // 电机无力控制
	MOTOR_GYRO,        // 电机陀螺仪控制（绝对角度）
	MOTOR_ENCONDE,     // 电机编码器控制（相对角度）
} gimbal_motor_mode_e;

/**
 * @brief 云台跟随状态枚举
 * @note 定义云台的朝向状态
 */
typedef enum
{
	GIMABL_NOMAL_FORWARD,   // 正常前向状态
    GIMABL_CHANGE_FORWARD,  // 掉头前向状态
} gimbal_follow_gimbal_state_e;

/**
 * @brief 云台电机控制数据结构体
 * @note 包含云台单个电机的所有控制参数和状态
 */
typedef struct
{  
    const motor_measure_t *gimbal_motor_measure;      // 电机测量数据指针
    gimbal_motor_mode_e gimbal_motor_mode;            // 电机当前控制模式
    gimbal_motor_mode_e gimbal_motor_mode_last;       // 电机上一次控制模式
    
    // 控制器
	lqr_type_def lqr;                                 // LQR控制器（PITCH用）
	lqr_type_def lqr_ac;                              // 自瞄LQR控制器
	pid_type_def pid_gimbal_speed;                    // 速度环PID控制器
	pid_type_def pid_gimbal_angle;                    // 角度环PID控制器
	pid_type_def pid_gimbal_speed_relative;           // 相对速度环PID控制器
	pid_type_def pid_gimbal_angle_relative;           // 相对角度环PID控制器
	pid_type_def pid_gimbal_speed_ac;                 // 自瞄速度环PID控制器
	pid_type_def pid_gimbal_angle_ac;                 // 自瞄角度环PID控制器
    
    // 角度相关
    fp32 relative_angle;                              // 相对角度（弧度）- 编码器
    fp32 max_relative_angle;                          // 最大相对角度限制
	fp32 min_relative_angle;                          // 最小相对角度限制
	fp32 relative_angle_set;                          // 相对角度设定值
	fp32 relative_omega;                              // 相对角速度（电机转速转换）
	fp32 relative_omega_set;                          // 相对角速度设定值（串级控制用）
    
    // 陀螺仪相关
    fp32 absolute_angle;                              // 绝对角度（弧度）- 陀螺仪
	fp32 absolute_gyro;                               // 绝对角速度（陀螺仪测量）
	fp32 absolute_gyro_set;                           // 绝对角速度设定值（串级控制用）
    fp32 max_absolute_angle;                          // 最大绝对角度限制
	fp32 min_absolute_angle;                          // 最小绝对角度限制
	fp32 absolute_angle_set;                          // 绝对角度设定值
    
    // 电流控制
	fp32 give_current;                                // 计算得到的电流发送值
	fp32 feedback_current;                            // 反馈得到的扭矩电流
    
    // 编码器相关
	uint16_t relative_ecd_aero;                       // 相对角度零点（编码器值）
	forward_feed feed_angle_cacl;                     // 角度前馈计算
    
    // 电机状态
	int8_t id;                                        // 电机ID
	uint8_t start_flag;                               // 启动标志
    int8_t mode;                                      // 电机模式
} gimbal_motor_t;

/**
 * @brief 云台状态切换数据结构体
 * @note 用于管理云台的状态切换和掉头逻辑
 */
typedef struct
{
	gimbal_follow_gimbal_state_e gimbal_change;       // 当前云台方向状态
	gimbal_follow_gimbal_state_e gimbal_change_last;  // 上一次云台方向状态
	uint8_t  gimbal_change_flag;                      // 云台方向切换标志
	uint32_t gimbal_change_time;                      // 云台方向切换计时器
	uint32_t gimbal_change_chassis_time;              // 云台与底盘切换计时器
	uint8_t  gimbal_chnamge_chassis_flag;             // 云台与底盘切换标志
} gimbal_change_t;

/**
 * @brief 云台控制主结构体
 * @note 包含云台的所有控制参数、状态和控制器
 */
typedef struct
{
	gimbal_motor_t pitch_gimbal_motor;                 // PITCH轴电机控制数据
    gimbal_motor_t yaw_gimbal_motor;                   // YAW轴电机控制数据
    
    // 传感器数据指针
	const fp32 *gimbal_INS_angle_point;                // IMU角度数据指针
	const fp32 *gimbal_INS_gyro_point;                 // IMU角速度数据指针
	const RC_ctrl_t *gimbal_dt7_remote_control;        // 遥控器数据指针
	const autom_pc *hero_pc;                           // PC通信数据指针
    
    // 云台状态
	gimbal_mode_e gimbal_mode;                         // 当前云台控制模式
	gimbal_mode_e gimbal_mode_last;                    // 上一次云台控制模式
	gimbal_change_t gimbal_angle_change;               // 云台角度变化状态
} gimbal_control_t;

// ============================ 函数声明 ============================

/**
 * @brief 获取云台控制结构体指针
 * @return 云台控制结构体指针
 */
extern gimbal_control_t *get_gimbal_control_point(void);

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


