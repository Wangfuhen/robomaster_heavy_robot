#include "can_user.h"
#include "main.h"
#include "string.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
#define ABS(x) (((x)>0) ? (x) : (-(x)))

#define get_motor_measure(ptr,data)                                     \
	{                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
	}

// #define get_9025_motor_measure(ptr, data)                                    \
//    {                                                                   \
//        (ptr)->last_ecd = (ptr)->ecd;                                   \
//        (ptr)->ecd = (uint16_t)((data)[7] << 8 | (data)[6]);            \
//        (ptr)->speed_rpm = (int16_t)((data)[5] << 8 | (data)[4]);      \
//        (ptr)->given_current = (int16_t)((data)[3] << 8 | (data)[2]);  \
//        (ptr)->temperate = (data)[1];                                   \
//    }

#define get_gimbal_measure(ptr,data)					        \
	{    											            \
		(ptr)->x_rc_data=((data)[0]<<8)|((data)[1]);  			\
		(ptr)->y_rc_data=((data)[2]<<8)|((data)[3]);  			\
		(ptr)->x_key_data=((data)[4]>>6)&3;  			        \
		(ptr)->y_key_data=((data)[4]>>4)&3;  			        \
		(ptr)->chassis_mode=((data)[5]>>6)&3;  			        \
		(ptr)->cap_mode=((data)[5]>>5)&1;  			            \
		(ptr)->top_mode=((data)[5]>>4)&1;  			            \
		(ptr)->follow_mode=((data)[5]>>3)&1;  			        \
		(ptr)->follow_mode_last=((data)[5]>>2)&1;  			    \
		(ptr)->key_control=((data)[5]>>1)&1;  			        \
	}                                                       \
                                                           
#define get_cap_measure(ptr,data)                                      \
	{                                                                  \
	    (ptr)->V_cap_value=(fp32)(((data)[2]<<8)|((data)[3]))*0.001;   \
	}
motor_measure_t motor_gimbal[1];
motor_measure_t motor_chassis[4];
chassis_c_get_data_t mini_chassis = 
{
		0,
		0,
		0,
		0,
		0,
		0,
		0,
	    0,
	    0,
	    0,
};
	
super_cap_t super_cap;

static uint8_t tx_data[8] = {0};

static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];

static CAN_TxHeaderTypeDef chassis_can_send_gimbal_msg;
static uint8_t chassis_send_gimbal_data[8];

static CAN_TxHeaderTypeDef cap_can_send_msg;
static uint8_t chassis_send_cap_data[8];

static CAN_TxHeaderTypeDef ramp_chassis_pitch;
static uint8_t chassis_send_ramp_pitch[4];


void get_9025_motor_ecd(motor_measure_t *ptr,uint8_t *data)                       
    {                                                                  
        (ptr)->ecd= ((data)[2]<<8)|((data)[3]);
      //(ptr)->ecdd= (data)[0];			
    }
/*can_recive*/		
		
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
   CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    if(hcan==&hcan1)  //不走滑环
    {
        switch (rx_header.StdId)
        {
        case CAN1_3508_M1_ID:  //201
        {
            get_motor_measure(&motor_chassis[0],rx_data);
            break;
        }
        case CAN1_3508_M2_ID:  //202
        {
            get_motor_measure(&motor_chassis[1],rx_data);
            break;
        }
		case CAN1_3508_M3_ID:  //203
        {
            get_motor_measure(&motor_chassis[2],rx_data);
            break;
        }
		case CAN1_3508_M4_ID:  //204
        {
					
            get_motor_measure(&motor_chassis[3],rx_data);
            break;
        }
		case CAN1_recive_cap_msg: //304
		{
			get_cap_measure(&super_cap,rx_data);
			break;
		}

        default:
        {
             break;
        }
  
        }
    }
    else if(hcan==&hcan2)  //走滑环
    {
        switch (rx_header.StdId)
        {
       
        case 0x200:
        {
            get_9025_motor_ecd(&motor_gimbal[0],rx_data);
            break;
        }
				
				case CAN2_recive_gimbal_msg:
				{
					get_gimbal_measure(&mini_chassis,rx_data);
					break;
				}
	
        default:
        {    
             break;
        }
           
        }
    }
}


 /*CAN_send*/

void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
     uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN1_chassis_flag_id;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&chassis_can, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_chassis_send_gimbal(RM_Referee_system_t *rm_data)
{
	uint32_t send_mail_box;
    chassis_can_send_gimbal_msg.StdId =  CAN2_send_gimbal_msg;
    chassis_can_send_gimbal_msg.IDE = CAN_ID_STD;  //标准ID(11位数)
    chassis_can_send_gimbal_msg.RTR = CAN_RTR_DATA;  //请求（RTR）为数据帧
    chassis_can_send_gimbal_msg.DLC = 0x0A;  //八位数据位
    chassis_send_gimbal_data[0] = (rm_data->robot_state.robot_id);
    chassis_send_gimbal_data[1] = (rm_data->robot_state.robot_level);
    chassis_send_gimbal_data[2] = (rm_data->robot_state.shooter_barrel_cooling_value);
    chassis_send_gimbal_data[3] = (uint8_t)(rm_data->shoot_data_t.bullet_speed*10);
    chassis_send_gimbal_data[4] = (rm_data->robot_state.shooter_barrel_heat_limit>>8);
    chassis_send_gimbal_data[5] = (rm_data->robot_state.shooter_barrel_heat_limit);
    chassis_send_gimbal_data[6] = (rm_data->power_heat_data_t.shooter_17mm_1_barrel_heat>>8);
    chassis_send_gimbal_data[7] = (rm_data->power_heat_data_t.shooter_17mm_1_barrel_heat);
    HAL_CAN_AddTxMessage(&gimbal_can, &chassis_can_send_gimbal_msg, chassis_send_gimbal_data, &send_mail_box);
}

void CAN_cmd_cap(RM_Referee_system_t *rm_data)
{
	uint32_t send_mail_box;
    cap_can_send_msg.StdId =  CAN1_send_cap_msg;
    cap_can_send_msg.IDE = CAN_ID_STD;    //标准ID(11位数)
    cap_can_send_msg.RTR = CAN_RTR_DATA;  //请求（RTR）为数据帧
    cap_can_send_msg.DLC = 0x08;          //八位数据位
    chassis_send_cap_data[0] = (rm_data->robot_state.chassis_power_limit>>8);
    chassis_send_cap_data[1] = (rm_data->robot_state.chassis_power_limit);
    chassis_send_cap_data[2] = (rm_data->power_heat_data_t.buffer_energy>>8);
    chassis_send_cap_data[3] = (rm_data->power_heat_data_t.buffer_energy);
    chassis_send_cap_data[4] = (0);
    chassis_send_cap_data[5] = (0);
    chassis_send_cap_data[6] = (0);
    chassis_send_cap_data[7] = (0);
	HAL_CAN_AddTxMessage(&chassis_can, &cap_can_send_msg, chassis_send_cap_data, &send_mail_box);
}
/**
  * @brief  Can总线发送控制参数
  * @param
  * @retval
  */
void CAN_cmd_chassis_read(uint8_t id,int16_t current)
{
    // CAN_TxHeaderTypeDef chassis_tx_message;             /**!< can通信发送协议头 */
    uint32_t canTxMailbox= CAN_TX_MAILBOX0;

    chassis_tx_message.StdId = 0x140+id;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0xA1;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = current;
    chassis_can_send_data[5] = (current>>8);
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    //找到空的发送邮箱 把数据发送出去
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0);	// 如果三个发送邮箱都阻塞了就等待直到其中某个邮箱空闲
    if ((hcan1.Instance->TSR & CAN_TSR_TME0) != RESET)
    {   // 检查发送邮箱0状态 如果邮箱0空闲就将待发送数据放入FIFO0
        canTxMailbox = CAN_TX_MAILBOX0;
    }
    else if ((hcan1.Instance->TSR & CAN_TSR_TME1) != RESET)
    {
        canTxMailbox = CAN_TX_MAILBOX1;
    }
    else if ((hcan1.Instance->TSR & CAN_TSR_TME2) != RESET)
    {
        canTxMailbox = CAN_TX_MAILBOX2;
    }


    if(HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, chassis_can_send_data, (uint32_t *)&canTxMailbox)==HAL_OK) {};
}
void CAN_chassis_ramp_pitch_to_gimbal(fp32 pitch)
{
	uint32_t send_mail_box;
    ramp_chassis_pitch.StdId =  CAN2_send_gimbal_pitch;
    ramp_chassis_pitch.IDE = CAN_ID_STD;    //标准ID(11位数)
    ramp_chassis_pitch.RTR = CAN_RTR_DATA;  //请求（RTR）为数据帧
    ramp_chassis_pitch.DLC = 0x04;          //八位数据位
	memcpy(chassis_send_ramp_pitch,&pitch,sizeof(pitch));
	HAL_CAN_AddTxMessage(&gimbal_can, &ramp_chassis_pitch, chassis_send_ramp_pitch, &send_mail_box);
}

/*CAN_point_get*/
//CAN1
const motor_measure_t *get_motor_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[i&3];
}
//CAN2

const motor_measure_t *get_motor_gimbal_yaw_measure_point(void)
{
    return &motor_gimbal[0];
}

const super_cap_t *get_super_cap_measure_point(void)
{
    return &super_cap;
}

const chassis_c_get_data_t *get_c_gimbal_data_point(void)
{
	return &mini_chassis;
}
