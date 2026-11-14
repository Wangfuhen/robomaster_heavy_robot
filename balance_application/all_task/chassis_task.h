#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
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

#include "main.h"
#include "c_data_typedef.h"
#include "can_user.h"
#include "pid.h"
#include "dt7_remote_read.h"
#include "crg_filter.h"
#include "usb_task.h"
#include "referee.h"
#include "chassis_configuration.h"


/*底盘速度低通滤波*/
typedef enum
{
    CHASSIS_OPERATING_MODE_ZERO_FORCE,      //零力模式
    CHASSIS_OPERATING_MODE_HOLD_POSITION,         //静止模式
    CHASSIS_OPERATING_MODE_INDEPENDENT,//不跟随云台模式
    CHASSIS_OPERATING_MODE_FOLLOW_GIMBAL,   //跟随云台模式
} chassis_operating_mode_e;


typedef enum
{
    SUPER_CAPACITOR_MODE_OFF,
    SUPER_CAPACITOR_MODE_ON,
} super_capacitor_mode_e;

/* 【已停用】小陀螺模式：机器人加装机械臂后不再支持，top_spin_mode 恒为 TOP_SPIN_MODE_OFF，
   TOP_SPIN_MODE_ON 仅为保留枚举与 CAN 协议位而存在，运行中不会再被使用。 */
typedef enum
{
	TOP_SPIN_MODE_OFF,   //小陀螺关闭（当前恒为该值）
	TOP_SPIN_MODE_ON,    //小陀螺开启（【已停用】加装机械臂后不再支持）
} top_spin_mode_e;

/* 【已停用】一键掉头至正位：机器人加装机械臂后不再支持该模式，
   CHASSIS_FOLLOW_GIMBAL_STATE_TURNAROUND 运行中不会再被使用，状态恒为 NORMAL。 */
typedef enum
{
    CHASSIS_FOLLOW_GIMBAL_STATE_NORMAL,  //普通跟随方向（当前恒为该值）
    CHASSIS_FOLLOW_GIMBAL_STATE_TURNAROUND,  //掉头（【已停用】加装机械臂后不再支持）
}chassis_follow_gimbal_state_e;

/* 【已停用】小陀螺旋转方向：随小陀螺模式一并停用，当前不再被使用（保留定义备查）。 */
typedef enum
{
    CHASSIS_TOP_SPIN_DIRECTION_COUNTERCLOCKWISE,            //逆时针小陀螺（【已停用】）
    CHASSIS_TOP_SPIN_DIRECTION_CLOCKWISE,                   //顺时针小陀螺（【已停用】）
}chassis_top_direction_e;

typedef struct
{
	const motor_measurement_t *motor_measurement_pointer;
    fp32 rotor_speed_revolution_per_minute;
    fp32 rotor_speed_setpoint_revolution_per_minute;
    fp32 rotor_acceleration_revolution_per_minute_per_second;                 
    int16_t commanded_torque_current_count;       //电机计算得电流
    fp32 mechanical_power_watt;           //电机当前总功率
	fp32 demanded_mechanical_power_watt;       //用户计算所需功率
	fp32 measured_torque_current_ampere;				//电机反馈电流换算
	fp32 allocated_torque_current_ampere;           //用户计算所需电流转换
	fp32 rotor_angular_velocity_radian_per_second;               //电机当前角速度
	fp32 mechanical_power_limit_watt;               //单个电机所能得到功率
}chassis_motor_state_t;

/* 【已停用】一键掉头至正位：机器人加装机械臂后不再支持该模式，
   state / previous_state 恒为 CHASSIS_FOLLOW_GIMBAL_STATE_NORMAL，
   掉头计时与等待标志不再使用（chassis_task.c 中相关逻辑已注释）。 */
typedef struct
{
	chassis_follow_gimbal_state_e state;
	chassis_follow_gimbal_state_e previous_state;
	uint8_t  state_changed;
	uint32_t state_change_time_millisecond;
	uint32_t gimbal_turnaround_elapsed_time_millisecond;
	uint8_t  chassis_response_pending;
}chassis_follow_gimbal_transition_t;


typedef struct
{
    const super_capacitor_measurement_t *super_capacitor_measurement_pointer;
    const RM_Referee_system_t *referee_system_data_pointer;
    const RC_ctrl_t *remote_control_data_pointer;
	const gimbal_command_data_t *gimbal_command_data_pointer;
    chassis_motor_state_t motor_states[4];
	const motor_measurement_t *gimbal_yaw_motor_measurement_pointer;
	const fp32 *inertial_measurement_angle_pointer;
	
	chassis_follow_gimbal_transition_t follow_gimbal_transition;
	
	slope_plan_t forward_speed_slope_plan;
	slope_plan_t leftward_speed_slope_plan;
	slope_plan_t rotation_speed_slope_plan;
	
    pid_type_def speed_loop_pid[4];
    pid_type_def follow_gimbal_angle_loop_pid;
	pid_type_def power_buffer_loop_pid;

    chassis_operating_mode_e operating_mode;
    chassis_operating_mode_e previous_operating_mode;
    super_capacitor_mode_e super_capacitor_mode;
    top_spin_mode_e top_spin_mode;    //小陀螺模式（【已停用】恒为 TOP_SPIN_MODE_OFF）
    chassis_follow_gimbal_state_e follow_gimbal_state;  //云台跟随状态（【已停用】未使用，恒为 NORMAL）
    chassis_top_direction_e top_spin_direction;  //小陀螺方向（【已停用】未使用）
    
    lpf_t forward_speed_command_low_pass_filter;  //底盘x速度低通
    lpf_t leftward_speed_command_low_pass_filter;  //底盘y速度低通
    lpf_t rotation_speed_command_low_pass_filter;  //底盘z速度低通

    fp32 forward_speed_revolution_per_minute;                      //底盘速度 前进方向 前为正，单位 rpm
	fp32 leftward_speed_revolution_per_minute;                      //底盘速度 左右方向 左为正  单位 rpm
	fp32 rotation_speed_radian_per_second;                      //底盘旋转角速度，逆时针为正 单位 rad/s
	fp32 forward_speed_setpoint_revolution_per_minute;                      //底盘设定速度 前进方向 前为正，单位 rpm
	fp32 leftward_speed_setpoint_revolution_per_minute;                      //底盘设定速度 左右方向 左为正，单位 rpm
	fp32 rotation_speed_setpoint_radian_per_second;                      //底盘设定旋转角速度，逆时针为正 单位 rpm
	fp32 gimbal_relative_angle_radian;              //底盘与云台的相对角度，单位 rad
	fp32 gimbal_relative_angle_setpoint_radian;          //设置相对云台控制角度，单位 rad

    fp32 forward_speed_normal_revolution_per_minute;  //正常速度
    fp32 leftward_speed_normal_revolution_per_minute;  //正常速度
    fp32 rotation_speed_normal_radian_per_second;  //正常速度 

    fp32 forward_speed_maximum_revolution_per_minute;  //前进方向最大速度 	
    fp32 leftward_speed_maximum_revolution_per_minute;  //左方向最大速度 
    fp32 rotation_speed_maximum_radian_per_second;  //旋转最大速度

	fp32 forward_speed_increment_revolution_per_minute;  //x方向速度增量 
	fp32 leftward_speed_increment_revolution_per_minute;  //y方向速度增量 
	fp32 rotation_speed_increment_radian_per_second;  //z旋转速度增量

    fp32 mechanical_power_watt;          //底盘当前消耗功率
	fp32 demanded_mechanical_power_watt;      //底盘所需消耗功率
    fp32 power_buffer_limit_watt;      //裁判系统底盘缓冲上限
    fp32 power_buffer_used_watt;    //用户使用底盘缓冲上限
    fp32 power_maximum_watt;      //规则下最大可用功率
    fp32 power_available_watt; //实际上用户可用功率

    fp32 forward_action_error;  //x方向动作误差
    fp32 leftward_action_error;  //y方向动作误差
    fp32 rotation_action_error;  //z方向动作误差
    fp32 forward_action_gain;  //x方向动作占比
    fp32 leftward_action_gain;  //y方向动作占比
    fp32 rotation_action_gain;  //z方向动作占比
	
	int16_t gimbal_turnaround_elapsed_time_millisecond;  //一键掉头时间（【已停用】掉头模式停用后不再使用）
	
	fp32 forward_speed_slope_acceleration;
	fp32 forward_speed_slope_rate;
	fp32 leftward_speed_slope_acceleration;
	fp32 leftward_speed_slope_rate;
	fp32 rotation_speed_slope_acceleration;
	fp32 rotation_speed_slope_rate;
	
	fp32 chassis_angular_velocity_radian_per_second;
} chassis_control_t;

extern void Chassis_Task(void *argument);
#endif
