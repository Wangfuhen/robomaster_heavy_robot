#include "can_user.h"
#include "main.h"

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
	#define get_remote_status(ptr,data)                     \
    {                                                   \
        (ptr)->left_switch_state = (data)[0];          \         \
        (ptr)->robot_state = (data)[2];                \
        (ptr)->timestamp = (uint32_t)((data)[4] << 24 | (data)[5] << 16 | \
                                     (data)[6] << 8 | (data)[7]); \
    }
#define get_gimbal_measure(ptr,data)					\
	{    											    \
		(ptr)->rm_limit_power=((data)[0]);  			\
		(ptr)->rm_buffer_energy=((data)[1]);  							\
		(ptr)->shooter__heat_limit=(uint16_t)((data)[2]<<8 | (data)[3]);  \
		(ptr)->shooter_42mm_1_barrel_heat=(uint16_t)((data)[4]<<8 | (data)[5]);  \
		(ptr)->robot_id=((data)[6]);  			                                 \
		(ptr)->robot_level=((data)[7]);  			                             \
	}

 #define get_9025_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[7] << 8 | (data)[6]);            \
        (ptr)->speed_rpm = (int16_t)((data)[5] << 8 | (data)[4]);      \
        (ptr)->given_current = (int16_t)((data)[3] << 8 | (data)[2]);  \
        (ptr)->temperate = (data)[1];                                   \
    }


motor_measure_t motor_gimbal[6];//反馈接收数组
gimbal_c_get_data_t gimbal_can_get;
static CAN_TxHeaderTypeDef gimbal_pitch_tx_msg;
static uint8_t gimbal_pitch_can_send_data[8];

static CAN_TxHeaderTypeDef gimbal_shoot_tx_msg;
static uint8_t gimbal_shoot_can_send_data[8];

static CAN_TxHeaderTypeDef chassis_yaw_tx_msg;
static uint8_t gimbal_yaw_can_send_data[8];

static CAN_TxHeaderTypeDef gimbal_can_send_msg;
static uint8_t gimbal_send_chassis_data[8];

static CAN_TxHeaderTypeDef trigger_tx_msg;
static uint8_t trigger_can_send_data[8];
	
static CAN_TxHeaderTypeDef remote_to_chassis_tx_msg;
static uint8_t remote_to_chassis_can_send_data[8];


int cnt2 = 0;
int id2 = 0;


/*can_recive*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
   CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    if(hcan==&hcan1)  //不走滑环
    {
						cnt2++;
			id2 = rx_header.StdId;
        switch (rx_header.StdId)
        {
        case CAN1_fire_l_motor_id:  //202
        {
            get_motor_measure(&motor_gimbal[0],rx_data);
            break;
        }
        case CAN1_fire_r_motor_id:  //201
        {
            get_motor_measure(&motor_gimbal[1],rx_data);
            break;
        }
       
        case CAN1_fire_d_motor_id:  //203
        {
            get_motor_measure(&motor_gimbal[2],rx_data);
            break;
        }
        case CAN1_pitch_motor_id: //301
        {
            get_motor_measure(&motor_gimbal[3],rx_data);
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
       
        case CAN2_yaw_motor_id:  //143
        {
            get_9025_motor_measure(&motor_gimbal[5],rx_data);
            break;
        }
				 case CAN1_trigger_motor_id:  //201
        {
            get_motor_measure(&motor_gimbal[4],rx_data);
            break;
        }
        
		
		 case CAN2_recive_chassis_msg:
	    {
		  	get_gimbal_measure(&gimbal_can_get,rx_data);
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
void CAN_send_remote_to_chassis(uint8_t left_switch)
                           
{
    uint32_t send_mail_box;
    remote_to_chassis_tx_msg.StdId = CAN2_SEND_REMOTE_TO_CHASSIS;
    remote_to_chassis_tx_msg.IDE = CAN_ID_STD;
    remote_to_chassis_tx_msg.RTR = CAN_RTR_DATA;
    remote_to_chassis_tx_msg.DLC = 0x01;
    remote_to_chassis_can_send_data[0] = left_switch;               
    HAL_CAN_AddTxMessage(&chassis_can, &remote_to_chassis_tx_msg, remote_to_chassis_can_send_data, &send_mail_box);
}

// fire
void CAN_cmd_shoot(int16_t frie_right, int16_t frie_left,int16_t fric_down)
{
    uint32_t send_mail_box;
    gimbal_shoot_tx_msg.StdId = 0X200;
    gimbal_shoot_tx_msg.IDE = CAN_ID_STD;
    gimbal_shoot_tx_msg.RTR = CAN_RTR_DATA;
    gimbal_shoot_tx_msg.DLC = 0x08;
    gimbal_shoot_can_send_data[0] = frie_right >> 8;
    gimbal_shoot_can_send_data[1] = frie_right;
    gimbal_shoot_can_send_data[2] = frie_left >> 8;
    gimbal_shoot_can_send_data[3] = frie_left;
    gimbal_shoot_can_send_data[4] = fric_down >> 8;
    gimbal_shoot_can_send_data[5] = fric_down;
    gimbal_shoot_can_send_data[6] = 0 >> 8;
    gimbal_shoot_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&gimbal_can, &gimbal_shoot_tx_msg, gimbal_shoot_can_send_data, &send_mail_box);
}



void CAN_cmd_gimbal_yaw(int16_t yaw)
{
    uint32_t send_mail_box;
    chassis_yaw_tx_msg.StdId =   CAN2_yaw_flag_id;
    chassis_yaw_tx_msg.IDE = CAN_ID_STD;  //标准ID(11位数)
    chassis_yaw_tx_msg.RTR = CAN_RTR_DATA;  //请求（RTR）为数据帧
    chassis_yaw_tx_msg.DLC = 0x08;  //八位数据位
    gimbal_yaw_can_send_data[0] = 0xA1;
    gimbal_yaw_can_send_data[1] = 0;
    gimbal_yaw_can_send_data[2] = 0;
    gimbal_yaw_can_send_data[3] = 0;
    gimbal_yaw_can_send_data[4] =yaw;
    gimbal_yaw_can_send_data[5] = (yaw>>8);
    gimbal_yaw_can_send_data[6] = 0;
    gimbal_yaw_can_send_data[7] = 0;
    HAL_CAN_AddTxMessage(&chassis_can, &chassis_yaw_tx_msg, gimbal_yaw_can_send_data, &send_mail_box);
}

void CAN_cmd_trigger(int16_t trigger,int16_t ecd)//触发
{
    uint32_t send_mail_box;
    trigger_tx_msg.StdId =  CAN1_trigger_flag_id;
    trigger_tx_msg.IDE = CAN_ID_STD;  //标准ID(11位数)
    trigger_tx_msg.RTR = CAN_RTR_DATA;  //请求（RTR）为数据帧
    trigger_tx_msg.DLC = 0x08;  //八位数据位
    trigger_can_send_data[0] = (trigger >> 8);
    trigger_can_send_data[1] = trigger;
    trigger_can_send_data[2] = (ecd >> 8);
    trigger_can_send_data[3] =ecd;
    trigger_can_send_data[4] = (0 >> 8);
    trigger_can_send_data[5] = 0;
    trigger_can_send_data[6] = (0 >> 8);
    trigger_can_send_data[7] = 0;
    HAL_CAN_AddTxMessage(&chassis_can, &trigger_tx_msg, trigger_can_send_data, &send_mail_box);
}

void sen_flag_cap(uint8_t falg)
{
//	uint32_t send_mail_box;
//    gimbal_can_send_msg.StdId =  CAN2_recive_chassis_msg;
//    gimbal_can_send_msg.IDE = CAN_ID_STD;  //标准ID(11位数)
//    gimbal_can_send_msg.RTR = CAN_RTR_DATA;  //请求（RTR）为数据帧
//    gimbal_can_send_msg.DLC = 0x08;  //八位数据位
//    gimbal_send_chassis_data[0] = (falg);
//    gimbal_send_chassis_data[1] = (0);
//    gimbal_send_chassis_data[2] = (0);
//    gimbal_send_chassis_data[3] = (0);
//    gimbal_send_chassis_data[4] = (0);
//    gimbal_send_chassis_data[5] = (0);
//    gimbal_send_chassis_data[6] = (0);
//    gimbal_send_chassis_data[7] = (0);
//    HAL_CAN_AddTxMessage(&chassis_can, &gimbal_can_send_msg, gimbal_send_chassis_data, &send_mail_box);
}

void CAN_gimbal_send_chassis(gimbal_c_send_data_t *gimbal_send_data)
{
	uint32_t send_mail_box;
    gimbal_can_send_msg.StdId =  CAN2_send_chassis_msg;
    gimbal_can_send_msg.IDE = CAN_ID_STD;  //标准ID(11位数)
    gimbal_can_send_msg.RTR = CAN_RTR_DATA;  //请求（RTR）为数据帧
    gimbal_can_send_msg.DLC = 0x08;  //八位数据位
	gimbal_send_chassis_data[0] = (gimbal_send_data->x_rc_data>>8);
    gimbal_send_chassis_data[1] = (gimbal_send_data->x_rc_data);
    gimbal_send_chassis_data[2] = (gimbal_send_data->y_rc_data>>8);
    gimbal_send_chassis_data[3] = (gimbal_send_data->y_rc_data);
    gimbal_send_chassis_data[4] = ((gimbal_send_data->x_key_data<<6)&(3<<6))|((gimbal_send_data->y_key_data<<4)&(3<<4));  //剩余四位
    gimbal_send_chassis_data[5] = ((gimbal_send_data->chassis_mode<<6)&(3<<6))|((gimbal_send_data->cap_mode<<5)&(1<<5))|((gimbal_send_data->top_mode<<4)&(1<<4))|((gimbal_send_data->follow_mode<<3)&(1<<3))|((gimbal_send_data->follow_mode_last<<2)&(1<<2))|((gimbal_send_data->key_control<<1)&(1<<1));  //剩余一位
    gimbal_send_chassis_data[6] = (0);
    gimbal_send_chassis_data[7] = (0);
    HAL_CAN_AddTxMessage(&chassis_can, &gimbal_can_send_msg, gimbal_send_chassis_data, &send_mail_box);
}



void CAN_motor_send_data_pitch(int16_t pitch_current)
{
    uint32_t send_mail_box;

    /* 3508 + C620: 0x1FF covers ESC ID 5..8, big-endian, range +-16384.
       The clamp also protects against any stale 30000-valued LQR path. */
    if (pitch_current >  16000) { pitch_current =  16000; }
    if (pitch_current < -16000) { pitch_current = -16000; }

    gimbal_pitch_tx_msg.StdId = CAN1_pitch_flag_id;
    gimbal_pitch_tx_msg.IDE = CAN_ID_STD;
    gimbal_pitch_tx_msg.RTR = CAN_RTR_DATA;
    gimbal_pitch_tx_msg.DLC = 0x08;

    /* ESC ID = 5 -> bytes 0/1. ID 6/7/8 would move to 2-3 / 4-5 / 6-7. */
    gimbal_pitch_can_send_data[0] = (pitch_current >> 8) & 0xFF;
    gimbal_pitch_can_send_data[1] =  pitch_current       & 0xFF;
    gimbal_pitch_can_send_data[2] = 0;
    gimbal_pitch_can_send_data[3] = 0;
    gimbal_pitch_can_send_data[4] = 0;
    gimbal_pitch_can_send_data[5] = 0;
    gimbal_pitch_can_send_data[6] = 0;
    gimbal_pitch_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&gimbal_can, &gimbal_pitch_tx_msg, gimbal_pitch_can_send_data, &send_mail_box);
}


/*CAN_point_get*/
//CAN1
const motor_measure_t *get_motor_gimbal_fire_right_measure_point(void)
{
    return &motor_gimbal[1];
}
const motor_measure_t *get_motor_gimbal_fire_left_measure_point(void)
{
    return &motor_gimbal[0]; 
}
const motor_measure_t *get_motor_gimbal_fire_down_measure_point(void)
{
    return &motor_gimbal[2]; 
}
const motor_measure_t *get_motor_gimbal_pitch_measure_point(void)
{
    return &motor_gimbal[3];
}
const motor_measure_t *get_motor_gimbal_trigger_measure_point(void)
{
    return &motor_gimbal[4];
}
//CAN2

const motor_measure_t *get_motor_gimbal_yaw_measure_point(void)
{
    return &motor_gimbal[5];
}





const gimbal_c_get_data_t *get_c_rm_data_point(void)
{
	return &gimbal_can_get;
}




