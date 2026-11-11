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

#define gimbal_can hcan1
#define chassis_can hcan2
typedef enum
{
    /*gimbal_motor_get*/ 
    /****    hcan1  云台电机  ****/   
    CAN1_fire_l_motor_id=0x202,  // 两个摩擦轮0x200
    CAN1_fire_r_motor_id=0x201,
    CAN1_fire_d_motor_id=0x203,
	  CAN1_pitch_motor_id=0x205,   //pitch电机
       //pitch电机 0x1ff
	
    /****  hcan2  底盘电机  ****/
    
    CAN2_yaw_motor_id=0x141,      //yaw轴电机
    CAN1_trigger_motor_id=0x201,//拨弹电机

    /*gimbal_motor_send_flag_id*/
	//CAN1
    CAN1_shoot_flag_id=0x200,
    CAN1_pitch_flag_id=  0x1FF,
    CAN1_trigger_flag_id=0x200,
	
	//CAN2
    CAN2_yaw_flag_id=    0x141,

    /*other_msg_id*/
    CAN2_send_chassis_msg    =  0x307,    //CAN2发送底盘  0X307
	CAN2_recive_chassis_msg  =  0X308,    //底盘接收信息 0x308
	CAN2_SEND_REMOTE_TO_CHASSIS = 0x401,

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
    uint16_t rm_limit_power;                     //底盘功率上限
    uint16_t rm_buffer_energy;                   //缓冲能量
	uint16_t shooter_42mm_1_barrel_heat;         //42毫米枪管热量
	uint8_t  robot_id;                           //机器人ID
	uint16_t shooter_barrel_cooling_value;       //机器人枪口热量每秒冷却值 
	uint16_t shooter_barrel_heat_limit;          //机器人枪口热量上限
	uint8_t V_cap_value;                         //电容电压
}super_cap_t;

typedef struct 
{
	uint8_t rm_limit_power;
	uint8_t rm_buffer_energy;
	uint16_t shooter__heat_limit;
	uint16_t shooter_42mm_1_barrel_heat;
	uint8_t robot_id;
	uint8_t robot_level;
}gimbal_c_get_data_t;

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
}gimbal_c_send_data_t;


/*CAN_send*/
extern void CAN_cmd_shoot(int16_t fric_left, int16_t fric_right,int16_t fric_down);             //fire
extern void CAN_cmd_gimbal_yaw(int16_t yaw);                                  //yaw
extern void CAN_motor_send_data_pitch(int16_t pitch);		                  //pitch
extern void CAN_cmd_trigger(int16_t trigger,int16_t ecd);                                 //trigger
extern void CAN_gimbal_send_chassis(gimbal_c_send_data_t *gimbal_send_data);  //gimbal
/*CAN_recive_point*/
extern const motor_measure_t *get_motor_gimbal_fire_left_measure_point(void);
extern const motor_measure_t *get_motor_gimbal_fire_right_measure_point(void);
extern const motor_measure_t *get_motor_gimbal_fire_down_measure_point(void);
extern const motor_measure_t *get_motor_gimbal_pitch_measure_point(void);

extern const motor_measure_t *get_motor_gimbal_yaw_measure_point(void);
extern const motor_measure_t *get_motor_gimbal_trigger_measure_point(void);

extern const gimbal_c_get_data_t *get_c_rm_data_point(void);

extern void CAN_send_remote_to_chassis(uint8_t left_switch);
                                   
#endif

