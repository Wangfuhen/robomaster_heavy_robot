/*-------------------------------------------------------------------------------------------
 * 文件名称：shoot_task.h
 * 文件描述：英雄机器人发射机构控制任务头文件
 * 包含内容：宏定义、枚举类型、结构体定义、全局常量声明
 * 主要功能：定义发射机构控制相关的参数、数据结构和接口
 * 依赖文件：dt7_remote_read.h, pid.h, can_user.h, main.h, joint_motor_lqr.h, gimbal_task.h
 *-------------------------------------------------------------------------------------------*/

#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

/*-------------------------------------------------------------------------------------------
 *                                 艺术字：HERO ROBOTICS
 * 用于标识团队或项目，增加代码可读性和趣味性，不影响实际功能
 *-------------------------------------------------------------------------------------------*/
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
                                                                                                ||||||||||||||||                                                                                                                                                                   
 
*/

/*================================ 头文件包含 =================================*/
#include "dt7_remote_read.h"      // 遥控器数据处理
#include "pid.h"                  // PID控制器库
#include "can_user.h"             // CAN通信接口
#include "main.h"                 // 主头文件（系统配置）
#include "joint_motor_lqr.h"      // LQR控制器库
#include "gimbal_task.h"          // 云台控制任务
#include "stdio.h"                // 标准输入输出（调试用）

/*================================ 任务配置宏 =================================*/
#define SHOOT_LOOP_TIME             1    // 控制任务循环周期（单位：RTOS tick）
#define SHOOT_INIT_TIM              120  // 初始化延迟时间（单位：RTOS tick）

/*================================ 物理转换系数 =================================*/
#define MOTOR_RPM_TO_SPEED          0.1047197533333      // 3508电机：RPM转角速度（rad/s）！
#define MOTOR_ECD_TO_ANGLE          0.000021305288720633905968306772076277f  // 3508电机：编码器值转角度（rad）
#define MOTOR_A                     0.0001f                                  // 3508电机：CAN报文电流值转实际电流（A）！
#define MOTOR_T_K                   0.00018                                  // 3508电机：扭矩系数（Nm/A）
#define MOTOR_R                     0.05                                     // 拨弹盘半径（米）
#define MAX_OUT                     1000                                     // 3508电机最大输出电流（CAN报文值）




/*================================ 遥控器按键宏 =================================*/
#define FIRE_ON                     KEY_PRESSED_OFFSET_R  // 启动/停止摩擦轮按键（R键）
#define FIRE_ALL                    KEY_PRESSED_OFFSET_Q  // 连发模式按键（Q键）

/*================================ 堵转检测参数 =================================*/
#define BLOCK_ANGLE_ERROR           1.0f    // 堵转角度误差阈值（度）
#define BLOCK_CURRENT               0.8f    // 堵转电流阈值（A）
#define BLOCK_TIME                  500.0f  // 持续堵转判定时间（ms）
#define BLOCK_REVERSE_TIME          50.0f   // 堵转后反向旋转时间（ms）

/*================================ 射击参数 =================================*/
#define SHOOT_ANGLE_ERROR           72.0f   // 射击角度误差限制（度）

/*================================ 拨弹盘速度环PID参数（调试备用）=================================*/
#define TRIGGER_SPEED_PID_KP        0.0f  // 比例系数
#define TRIGGER_SPEED_PID_KI        0.0f    // 积分系数
#define TRIGGER_SPEED_PID_KD        0.0f    // 微分系数
#define TRIGGER_SPEED_PID_MAX_OUT   MAX_OUT // PID输出上限
#define TRIGGER_SPEED_PID_MAX_IOUT  7000.0f // 积分项上限

/*================================ 拨弹盘LQR控制器参数 =================================*/
// 速度环参数
#define TRIGGLE_SPEED_KP        10   // 速度环比例系数
#define TRIGGLE_SPEED_KI        0       // 速度环积分系数
#define TRIGGLE_SPEED_I_MAX     0       // 速度环积分上限
#define TRIGGLE_SPEED_OUT_MAX   100000  // 速度环输出上限

// 角度环参数
#define TRIGGLE_ANGLE_KP        200    // 角度环比例系数
#define TRIGGLE_ANGLE_KI        0       // 角度环积分系数
#define TRIGGLE_ANGLE_I_MAX     5       // 角度环积分上限
#define TRIGGLE_ANGLE_OUT_MAX   100000  // 角度环输出上限

// 电流环参数
#define TRIGGLE_CURRENT_KP      0.0    // 电流环比例系数
#define TRIGGLE_CURRENT_KI      0       // 电流环积分系数
#define TRIGGLE_CURRENT_I_MAX   0       // 电流环积分上限
#define TRIGGLE_CURRENT_OUT_MAX 10000   // 电流环输出上限  上限±16384

/*================================ 摩擦轮电机PID参数 =================================*/
// 右摩擦轮（电机1）参数
#define FIRE1_SPEED_PID_KP      30.0f   // 比例系数
#define FIRE1_SPEED_PID_KI      2.50f   // 积分系数
#define FIRE1_SPEED_PID_KD      20.0f   // 微分系数

// 左摩擦轮（电机2）参数
#define FIRE2_SPEED_PID_KP      30.0f   // 比例系数
#define FIRE2_SPEED_PID_KI      2.50f   // 积分系数
#define FIRE2_SPEED_PID_KD      20.0f   // 微分系数

// 下摩擦轮（电机3）参数
#define FIRE3_SPEED_PID_KP      30.0f   // 比例系数
#define FIRE3_SPEED_PID_KI      2.50f   // 积分系数
#define FIRE3_SPEED_PID_KD      20.0f   // 微分系数

// 摩擦轮PID公共参数
#define FIRE_FIRE_PID_MAX_OUT  4000.0f // 输出电流上限
#define FIRE_FIRE_PID_MAX_IOUT 3000.0f  // 积分项上限

/*================================ 枚举类型定义 =================================*/

/**
 * @brief 射击模式枚举
 * @details 控制摩擦轮的启停状态
 */
typedef enum
{
    SHOOT_STOP = 0,   // 停止射击：摩擦轮停转
    SHOOT_START,      // 开始射击：摩擦轮启动
} shoot_mode_e;

/**
 * @brief 拨弹盘模式枚举
 * @details 控制拨弹盘的运行状态
 */
typedef enum
{
    TRIGGER_STOP = 0,  // 停止拨弹
    TRIGGER_READY,     // 准备拨弹
} trigger_mode_e;

/*================================ 结构体定义 =================================*/

/**
 * @brief 堵转检测结构体
 * @details 用于检测和处理电机堵转情况
 */
typedef struct
{
    uint32_t block_time;      // 堵转持续时间计数（ms）
    uint32_t reverse_time;    // 反向旋转时间计数（ms）
} trigger_block_t;

/**
 * @brief 摩擦轮电机控制结构体
 * @details 管理单个摩擦轮电机的状态和控制参数
 */
typedef struct
{
    const motor_measurement_t *shoot_motor_measure;  // 电机测量数据指针
    pid_type_def pid_shoot_speed;                // 速度环PID控制器
    lqr_type_def lqr;                            // LQR控制器（预留）
    fp32 speed_set;                              // 目标转速（RPM）
    fp32 give_current;                           // 输出电流（CAN报文值）
    fp32 feedback_current;                       // 反馈电流（实际A）
    fp32 feedback_speed;                         // 反馈转速（RPM）
} shoot_motor_t;

/**
 * @brief 拨弹盘电机控制结构体
 * @details 管理拨弹盘电机的状态、控制和保护逻辑
 */
typedef struct
{
    const motor_measurement_t *trigger_motor_measure;  // 电机测量数据指针
    trigger_mode_e trigger_mode;                   // 拨弹盘工作模式
    trigger_block_t tigger_bolck;                  // 堵转检测状态
    
    lqr_type_def lqr;                              // LQR三环控制器
    pid_type_def speed_pid;                        // 速度环PID（备用）
    
    fp32 give_current;                             // 输出电流（CAN报文值）
    fp32 feedback_current;                         // 反馈电流（实际A）
    fp32 feedback_speed;                           // 反馈转速（rad/s）
    fp32 feedback_angle;                           // 反馈角度（rad）
    
    uint16_t shoot_ban_time_hz;                    // 射击频率计时器（状态机变量1）
    uint16_t shoot_ban_hz;                         // 射击频率设定值（状态机变量2）
    uint8_t fire_power_flag;                       // 射击热量限制标志：1=允许射击
    uint16_t fire_ban_hz;                          // 射击间隔频率控制
    
    fp32 dials_ecd_count;                          // 编码器增量计数
    fp32 add_angle;                                // 累计角度（度）
    
    fp32 trigger_motor_speed_set;               // 目标转速（备用）
    fp32 trigger_motor_angle_set;               // 目标角度（度）
} trigger_motor_t;

/**
 * @brief 发射机构总控制结构体
 * @details 整合所有发射机构相关的控制状态和数据
 */
typedef struct
{
    const RC_ctrl_t *shoot_dt7_remote_control;  // 遥控器数据指针
    const gimbal_control_t *gimbal;              // 云台控制数据指针
    
    shoot_motor_t shoot_motor_1;                 // 右摩擦轮电机
    shoot_motor_t shoot_motor_2;                 // 左摩擦轮电机
	shoot_motor_t shoot_motor_3;                 // 下摩擦轮电机
    trigger_motor_t trigger_motor;               // 拨弹盘电机
    
    shoot_mode_e shoot_mode;                     // 射击模式
    
    uint8_t press_r;                             // 鼠标右键当前状态
    uint8_t press_l;                             // 鼠标左键当前状态
    uint8_t last_press_r;                        // 鼠标右键上次状态
    uint8_t last_press_l;                        // 鼠标左键上次状态
    
    uint8_t fire_flag_change;                    // 射击标志变化位：0->1启动，1->0停止
    uint16_t fire_flag_time;                     // 射击标志计时器（防抖用）
} hero_shoot_t;

#endif /* SHOOT_TASK_H */

/*================================================================================
 * 文件结构说明：
 * 1. 宏定义部分：包含控制参数、物理转换系数、PID参数等
 * 2. 枚举类型：定义系统工作模式
 * 3. 结构体定义：分层定义电机控制和系统状态
 * 4. 物理单位说明：
 *    - 角度：度（deg）或弧度（rad）
 *    - 转速：RPM（转/分）或 rad/s
 *    - 电流：A（安培）或 CAN报文值
 *    - 时间：ms（毫秒）或 RTOS tick
 *================================================================================*/


