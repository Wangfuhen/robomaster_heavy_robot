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
    /* gimbal motor feedback */
    /****    hcan1  云台电机  ****/   
    CAN1_fire_l_motor_id=0x202,  // 两个摩擦轮0x200
    CAN1_fire_r_motor_id=0x201,
    CAN1_fire_d_motor_id=0x203,
	  CAN_MESSAGE_IDENTIFIER_PITCH_AXIS_FEEDBACK=0x205,   //pitch电机
       //pitch电机 0x1ff
	
    /****  hcan2  底盘电机  ****/
    
    CAN2_yaw_motor_id=0x141,      //yaw轴电机
    CAN1_trigger_motor_id=0x201,//拨弹电机

    /*gimbal_motor_send_flag_id*/
	//CAN1
    CAN1_shoot_flag_id=0x200,
    CAN_MESSAGE_IDENTIFIER_PITCH_AXIS_COMMAND=  0x1FF,
    CAN1_trigger_flag_id=0x200,
	
	//CAN2
    CAN2_yaw_flag_id=    0x141,

    /*other_msg_id*/
    CAN2_send_chassis_msg    =  0x307,    //CAN2发送底盘  0X307
	CAN2_recive_chassis_msg  =  0X308,    //底盘接收信息 0x308
	CAN2_SEND_REMOTE_TO_CHASSIS = 0x401,

}can_message_identifier_t;

typedef struct 
{
    uint16_t encoder_count;           //编码器机械数值
    int16_t rotor_speed_revolution_per_minute;      //转子转动速度 每分钟转
    int16_t measured_torque_current_count;  //转子转矩电流
    uint8_t motor_temperature_degree_celsius;      //转子温度
    uint8_t controller_error_code;                  // C620 DATA[7] 错误码
    uint32_t last_update_time_millisecond;          // 最近一次收到反馈的时间
    int16_t previous_encoder_count;       //上一次编码器值
    int16_t encoder_count_difference;      //增量
    int16_t revolution_count;          //转的圈数
}motor_measurement_t;

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


/* CAN transmit */
extern void CAN_cmd_shoot(int16_t fric_left, int16_t fric_right,int16_t fric_down);             //fire
extern void CAN_cmd_gimbal_yaw(int16_t yaw);                                  //yaw
extern void CAN_SendPitchAxisTorqueCurrentCommand(int16_t pitch);		                  //pitch
extern void CAN_cmd_trigger(int16_t trigger,int16_t encoder_count);                                 //trigger
extern void CAN_gimbal_send_chassis(gimbal_c_send_data_t *gimbal_send_data);  //gimbal
/*CAN_recive_point*/
extern const motor_measurement_t *GetLeftFrictionWheelMotorMeasurementPointer(void);
extern const motor_measurement_t *GetRightFrictionWheelMotorMeasurementPointer(void);
extern const motor_measurement_t *GetDownFrictionWheelMotorMeasurementPointer(void);
extern const motor_measurement_t *GetPitchAxisMotorMeasurementPointer(void);

extern const motor_measurement_t *GetYawAxisMotorMeasurementPointer(void);
extern const motor_measurement_t *GetTriggerMotorMeasurementPointer(void);

extern const gimbal_c_get_data_t *get_c_rm_data_point(void);

extern void CAN_send_remote_to_chassis(uint8_t left_switch);
                                   
#endif

