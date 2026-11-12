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
#define CHASSIS_LOOP_TIME 2                        //底盘任务周期2MS
#define MINI_CHASSIS_INIT_TIME 100                 //底盘任务初始化100MS


#define MAX_CURRENT_3508 16384       //3508最大电流
#define RAD_TO_ANGLE 57.2957795f     //3508弧度转角度

/*REMOTE_DT7*/
#define CHASSIS_REMOTE_DEADLINE 10   //遥控器死区
#define CHASSIS_VX   3               //底盘x速度
#define CHASSIS_VY   2               //底盘y速度
#define CHASSIS_WZ   1               //底盘z速度
#define LEFT         1               //左拨杆
#define RIGHT        0               //右拨杆
#define ACTION_FLAG  80              //摇杆触发行为区间
#define RC_X_SENSE   48              //X灵敏度
#define RC_Y_SENSE   48              //Y灵敏度


/*MOUSE_DT7*/
#define FORWARED        ((uint8_t)1<<0)
#define BACK            ((uint8_t)1<<1)
#define RGHTWARD        ((uint8_t)1<<0)
#define LEFTWARD        ((uint8_t)1<<1)

#define CAP_ON_KEY       KEY_PRESSED_OFFSET_SHIFT
#define TOP_ON_KEY       KEY_PRESSED_OFFSET_C

/*麦轮正向解算*/
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f        //转换比例  待测
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f        //转换比例  
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f        //转换比例
#define MOTOR_DISTANCE_TO_CENTER        0.364f        //小麦 电机到中心距离
//底盘跟随角度
#define CHASSIS_FOLLOW_GIMBAL_ANGLE_NOMAL 0.0f       //普通底盘跟随云台角度
#define CHASSIS_FOLLOW_GIMBAL_ANGLE_CHANGE 0.0f      //一键掉头底盘跟随云台角度

/*chassis speed_pid*/
#define CHASSIS_SPEED_PID_P 20.0f
#define CHASSIS_SPEED_PID_I 0.15f                   //动态调整
#define CHASSIS_SPEED_PID_D 10.0f
#define CHASSIS_SPEED_PID_MAX_OUT MAX_CURRENT_3508
#define CHASSIS_SPEED_PID_MAX_I_OUT 2000.0f         //动态调整
/*chassis follow_gimbal_pid*/
#define CHASSIS_FOLLOW_GIMBAL_PID_P 28000.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_I 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_D 3000.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 30000.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_I_OUT 1500.0f

#define CHASSIS_SOLLOW_GIMBAL_FORWARD_KP  0.07f

/*chassis_power_buffer_control*/
#define BUFFER_PID_P            -2.5f
#define BUFFER_PID_I            -0.001f
#define BUFFER_PID_D            -0.000f
#define BUFFER_PID_MAX_OUT      60
#define BUFFER_PID_MAX_I_OUT    10
//dt7控制下的底盘速度 VX_VY
#define CHASSIS_SPEED_MAX_VX_DT7 3300.0f        //基础初始速度
#define CHASSIS_SPEED_MAX_VY_DT7 3300.0f
//比赛键盘控制下的底盘速度 VX_VY
#define CHASSIS_SPEED_MAX_VX_KEYBOARD 8000.0f   //初始速度
#define CHASSIS_SPEED_MAX_VY_KEYBOARD 6000.0f
//超电下小陀螺 WZ 
#define CHASSIS_SPEED_TOP_CAP_ON  26000.0f       //小陀螺速度 
#define CHASSIS_SPEED_TOP_CAP_OFF 26000.0f

//底盘功率测试开关
#define CHASSIS_POWER_TEST_ON 1
//功率拟合
#define K0 0.6641993412640775f
#define K1 0.006444284468539646f
#define K2 0.0001423857226262331f
#define K3 0.017644430204543864f
#define K4 0.1650143850678085f
#define K5 3.096721772539512e-05f
//功率控制
#define CHASSIS_POWER_BUFFER_USE_MAX_NO_GAIN 40;  //没有增益使用缓冲功率数值
#define CHASSIS_POWER_BUFFER_USE_MAX_GAIN   200;  //有增益使用缓冲功率数值
#define CHASSIS_POWER_BUFFER_ALl_NO_GAIN     60;  //没有增益缓冲功率数值上限
#define CHASSIS_POWER_BUFFER_ALL_GAIN       250;  //有增益缓冲功率数值上限
#define CHASSIS_POWER_RM_KP                 0.5;  //缓冲恢复P
//功率分配
#define CHASSIS_POWER_TO_ACTION_GAIN        1.0f;  //计算功率衰减比例
#define CHASSIS_POWER_VX_GAIN               0.4f;  //x方向功率增益
#define CHASSIS_POWER_VY_GAIN               0.3f;  //y方向功率增益
#define CHASSIS_POWER_WZ_GAIN               0.3f;  //z方向功率增益

#define RPM_RADS                            0.1047197533333   //2*pi/60
#define MA_A                                0.0012207031250   //20/16384

#define LPF_CHASSIS_X                        0.08f
#define LPF_CHASSIS_Y                        0.08f
#define LPF_CHASSIS_Z                        0.02f

#define SLOPE_CHASSIS_X_ACCELERATION                 8.5f
#define SLOPE_CHASSIS_X_DECELERATION                 19.5f
#define SLOPE_CHASSIS_Y_ACCELERATION                 500.5f
#define SLOPE_CHASSIS_Y_DECELERATION                 1000.5f
#define SLOPE_CHASSIS_W_ACCELERATION                 5.5f
#define SLOPE_CHASSIS_W_DECELERATION                 1.5f

#define HALF_ECD_RANGE        32768
#define ECD_RANGE             65536
#define YAW_CHANGE_WAIT_TIME    1600          //一键掉头等待时间
#define YAW_ZERO_ANGLE_SET      16190         //YAW 零点
#define YAW_CHANGE_ANGLE_SET    48929         //一键掉头零点
#define MOTOR_ECD_TO_RAD       0.0000958738f  // 2*PI/65536


/*底盘速度低通滤波*/
typedef enum
{
    CHASSIS_ZERO_FORCE,      //零力模式
    CHASSIS_NO_MOVE,         //静止模式
    CHASSIS_NO_FOLLOW_GIMBAL,//不跟随云台模式
    CHASSIS_FOLLOW_GIMBAL,   //跟随云台模式
} chassi_mode_e;


typedef enum
{
    CAP_OFF,
    CAP_ON,
} cap_mode_e;

typedef enum
{
	TOP_OFF,
	TOP_ON,
} top_mode_e;

typedef enum
{
    CHASSIS_FOLLOW_GIMBAL_NORMAL,  //普通跟随方向
    CHASSIS_FOLLOW_GIMBAL_CHANGE,
}chassis_follow_gimbal_state_e;

typedef enum
{
    CHASSIS_TOP_INVE,            //逆时针小陀螺
    CHASSIS_TOP_FWAD,
}chassis_top_direction_e;

typedef struct
{
	const motor_measure_t *chassis_motor_feedback;
    fp32 speed_rpm;
    fp32 speed_set;
    fp32 accel;                 
    int16_t give_current;       //电机计算得电流
    fp32 motor_power;           //电机当前总功率
	fp32 motor_demand_power;       //用户计算所需功率
	fp32 motor_current;				//电机反馈电流换算
	fp32 motor_use_current;           //用户计算所需电流转换
	fp32 motor_omega;               //电机当前角速度
	fp32 motor_power_limit;               //单个电机所能得到功率
}chassis_motor_t;

typedef struct
{
	chassis_follow_gimbal_state_e gimbal_change;
	chassis_follow_gimbal_state_e gimbal_change_last;
	uint8_t  gimbal_change_flag;
	uint32_t gimbal_change_time;
	uint32_t gimbal_change_chassis_time;
	uint8_t  gimbal_change_chassis_flag;
}chassis_follow_change_t;


typedef struct
{
    const super_cap_t *super_cap;
    const RM_Referee_system_t *chassis_rm_data;
    const RC_ctrl_t *chassis_dt7_remote_control;
	const chassis_c_get_data_t *chassis_gimbal_can_data;
    chassis_motor_t chassis_motor[4];
	const motor_measure_t *gimbal_yaw_motor;
	const fp32 *chassis_ins_point;
	
	chassis_follow_change_t chassis_follow;
	
	slope_plan_t chassis_vx;
	slope_plan_t chassis_vy;
	slope_plan_t chassis_w;
	
    pid_type_def chassis_speed_pid[4];
    pid_type_def chassis_follow_gimbal_pid;
	pid_type_def rm_buffer_control_pid;

    chassi_mode_e chassis_mode;
    chassi_mode_e chassis_mode_last;
    cap_mode_e chassis_cap_mode;
    top_mode_e chassis_top_mode;
    chassis_follow_gimbal_state_e chassis_follow_gimbal_state;
    chassis_top_direction_e chassis_top_direction;
    
    lpf_t chassis_cmd_vx_lpf;  //底盘x速度低通
    lpf_t chassis_cmd_vy_lpf;  //底盘y速度低通
    lpf_t chassis_cmd_wz_lpf;  //底盘z速度低通

    fp32 vx_now;                      //底盘速度 前进方向 前为正，单位 rpm
	fp32 vy_now;                      //底盘速度 左右方向 左为正  单位 rpm
	fp32 wz_now;                      //底盘旋转角速度，逆时针为正 单位 rad/s
	fp32 vx_set;                      //底盘设定速度 前进方向 前为正，单位 rpm
	fp32 vy_set;                      //底盘设定速度 左右方向 左为正，单位 rpm
	fp32 wz_set;                      //底盘设定旋转角速度，逆时针为正 单位 rpm
	fp32 relative_angle;              //底盘与云台的相对角度，单位 rad
	fp32 set_relative_angle;          //设置相对云台控制角度，单位 rad

    fp32 vx_normal_speed;  //正常速度
    fp32 vy_normal_speed;  //正常速度
    fp32 wz_normal_speed;  //正常速度 

    fp32 vx_max_speed;  //前进方向最大速度 	
    fp32 vy_max_speed;  //左方向最大速度 
    fp32 wz_max_speed;  //旋转最大速度

	fp32 vx_add_speed;  //x方向速度增量 
	fp32 vy_add_speed;  //y方向速度增量 
	fp32 wz_add_speed;  //z旋转速度增量

    fp32 chassis_power;          //底盘当前消耗功率
	fp32 chassis_power_demand;      //底盘所需消耗功率
    fp32 chassis_buffer_power_limit;      //裁判系统底盘缓冲上限
    fp32 chassis_buffer_power_used;    //用户使用底盘缓冲上限
    fp32 chassis_power_max;      //规则下最大可用功率
    fp32 chassis_power_user_can; //实际上用户可用功率

    fp32 chassis_action_vx_error;  //x方向动作误差
    fp32 chassis_action_vy_error;  //y方向动作误差
    fp32 chassis_action_wz_error;  //z方向动作误差
    fp32 chassis_action_vx_gain;  //x方向动作占比
    fp32 chassis_action_vy_gain;  //y方向动作占比
    fp32 chassis_action_wz_gain;  //z方向动作占比
	
	int16_t gimbal_change_chassis_time;  //一键掉头时间
	
	fp32 chassis_x_slope_accl;
	fp32 chassis_x_slope_rate;
	fp32 chassis_y_slope_accl;
	fp32 chassis_y_slope_rate;
	fp32 chassis_w_slope_accl;
	fp32 chassis_w_slope_rate;
	
	fp32 chassis_gyro;
} chassis_control_t;

extern void Chassis_Task(void *argument);
#endif
