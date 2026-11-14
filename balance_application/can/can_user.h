#ifndef CAN_USER_H
#define CAN_USER_H
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
#include "c_data_typedef.h"
#include "can_init.h"
#include "referee.h"
#include <stdbool.h>

typedef enum
{
    /*chassis_motor_get*/ 
    /****    hcan1  底盘电机  ****/   
    CAN_CHASSIS_MOTOR_1_IDENTIFIER = 0x201,   //底盘电机接收报文
    CAN_CHASSIS_MOTOR_2_IDENTIFIER = 0x202,
    CAN_CHASSIS_MOTOR_3_IDENTIFIER = 0x203,
    CAN_CHASSIS_MOTOR_4_IDENTIFIER = 0x204,

    /****  hcan2  底盘电机  ****/
    CAN_GIMBAL_YAW_MOTOR_IDENTIFIER=0x141,           //yaw轴电机接收报文 底盘和云台同时解算降低延迟

    /*chassis_motor_send_flag_id*/
    CAN_CHASSIS_COMMAND_IDENTIFIER   =  0x200,   //底盘电机发送报文
	
    /*other_msg_id*/
	CAN_CHASSIS_TO_GIMBAL_IDENTIFIER   =  0x308,    //底盘发送到云台 0x308
	CAN_GIMBAL_TO_CHASSIS_IDENTIFIER =  0x307,    //底盘接收云台 0x307
	CAN_CHASSIS_PITCH_TO_GIMBAL_IDENTIFIER =0X306,      //发送底盘pitch
	
	CAN_SUPER_CAPACITOR_TO_CHASSIS_IDENTIFIER = 0X304,        //接收来自超电的信息
	CAN_CHASSIS_TO_SUPER_CAPACITOR_IDENTIFIER   = 0X305,        //发送信息到底盘
	
}can_message_identifier_e;


typedef struct 
{
    uint16_t encoder_count;           //编码器机械数值
    int16_t rotor_speed_revolution_per_minute;      //转子转动速度 每分钟转
    int16_t commanded_torque_current_count;  //转子转矩电流
    uint8_t temperature_degree_celsius;      //转子温度
    int16_t previous_encoder_count;       //上一次编码器值
    int16_t encoder_count_delta;      //增量
    int16_t revolution_count;          //转的圈数
}motor_measurement_t;

typedef struct can_user
{
	fp32 voltage_volt;                         //电容电压
}super_capacitor_measurement_t;

typedef struct 
{
	int16_t forward_channel_value;        //16位 X_rc
	int16_t leftward_channel_value;        //16位 Y_rc
	uint8_t forward_key_value;       //两位 X_key
	uint8_t leftward_key_value;       //两位 Y_key
	uint8_t operating_mode;     //两位 底盘模式
	uint8_t super_capacitor_mode;         //一位 超电
	uint8_t top_spin_mode;         //一位 小陀螺（【已停用】加装机械臂后不再支持，恒为 0）
	uint8_t follow_gimbal_state;      //一位 一键掉头使用（【已停用】加装机械臂后不再支持，恒为 0）
	uint8_t previous_follow_gimbal_state; //一位 一键掉头使用（【已停用】加装机械臂后不再支持，恒为 0）
	uint8_t keyboard_control_enabled;      //使用键盘控制进入比赛模式
	
}gimbal_command_data_t;

/*CAN_send*/
extern void CAN_SendChassisCommand(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);  //chassis
extern void CAN_SendSuperCapacitorCommand(RM_Referee_system_t *rm_data);                                        //chassis_to_cap
extern void CAN_SendGimbalData(RM_Referee_system_t *rm_data);                            //
extern void CAN_SendChassisPitchToGimbal(fp32 pitch);
extern void CAN_SendChassisReadCommand(uint8_t id,int16_t current);
/*CAN_recive_point*/
extern const motor_measurement_t *get_motor_chassis_motor_measure_point(uint8_t i);
extern const motor_measurement_t *get_motor_gimbal_yaw_measure_point(void);
extern const super_capacitor_measurement_t *get_super_cap_measure_point(void);
extern const gimbal_command_data_t *get_c_gimbal_data_point(void);


#endif

