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
#define gimbal_can hcan2
#define chassis_can hcan1

typedef enum
{
    /*chassis_motor_get*/ 
    /****    hcan1  底盘电机  ****/   
    CAN1_3508_M1_ID = 0x201,   //底盘电机接收报文
    CAN1_3508_M2_ID = 0x202,
    CAN1_3508_M3_ID = 0x203,
    CAN1_3508_M4_ID = 0x204,

    /****  hcan2  底盘电机  ****/
    CAN2_yaw_motor_id=0x141,           //yaw轴电机接收报文 底盘和云台同时解算降低延迟

    /*chassis_motor_send_flag_id*/
    CAN1_chassis_flag_id   =  0x200,   //底盘电机发送报文
	
    /*other_msg_id*/
	CAN2_send_gimbal_msg   =  0x308,    //底盘发送到云台 0x308
	CAN2_recive_gimbal_msg =  0x307,    //底盘接收云台 0x307
	CAN2_send_gimbal_pitch =0X306,      //发送底盘pitch
	
	CAN1_recive_cap_msg = 0X304,        //接收来自超电的信息
	CAN1_send_cap_msg   = 0X305,        //发送信息到底盘
	
}can_msg_id;


typedef struct 
{
    uint16_t ecd;           //编码器机械数值
    int16_t speed_rpm;      //转子转动速度 每分钟转
    int16_t given_current;  //转子转矩电流
    uint8_t temperate;      //转子温度
    int16_t last_ecd;       //上一次编码器值
    int16_t delta_ecd;      //增量
    int16_t round;          //转的圈数
}motor_measure_t;

typedef struct can_user
{
	fp32 V_cap_value;                         //电容电压
}super_cap_t;

typedef struct 
{
	int16_t x_rc_data;        //16位 X_rc
	int16_t y_rc_data;        //16位 Y_rc
	uint8_t x_key_data;       //两位 X_key
	uint8_t y_key_data;       //两位 Y_key
	uint8_t chassis_mode;     //两位 底盘模式
	uint8_t cap_mode;         //一位 超电
	uint8_t top_mode;         //一位 小陀螺
	uint8_t follow_mode;      //一位 一键掉头使用
	uint8_t follow_mode_last; //一位 一键掉头使用
	uint8_t key_control;      //使用键盘控制进入比赛模式
	
}chassis_c_get_data_t;

/*CAN_send*/
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);  //chassis
extern void CAN_cmd_cap(RM_Referee_system_t *rm_data);                                        //chassis_to_cap
extern void CAN_chassis_send_gimbal(RM_Referee_system_t *rm_data);                            //
extern void CAN_chassis_ramp_pitch_to_gimbal(fp32 pitch);
extern void CAN_cmd_chassis_read(uint8_t id,int16_t current);
/*CAN_recive_point*/
extern const motor_measure_t *get_motor_chassis_motor_measure_point(uint8_t i);
extern const motor_measure_t *get_motor_gimbal_yaw_measure_point(void);
extern const super_cap_t *get_super_cap_measure_point(void);
extern const chassis_c_get_data_t *get_c_gimbal_data_point(void);


#endif

