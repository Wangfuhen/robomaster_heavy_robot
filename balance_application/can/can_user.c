#include "can_user.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static void MotorMeasurement_ParsePayload(motor_measurement_t *measurement, const uint8_t payload[8])
{
    measurement->previous_encoder_count = measurement->encoder_count;
    measurement->encoder_count = (uint16_t)(((uint16_t)payload[0] << 8) | payload[1]);
    measurement->rotor_speed_revolution_per_minute = (int16_t)(((uint16_t)payload[2] << 8) | payload[3]);
    measurement->measured_torque_current_count = (int16_t)(((uint16_t)payload[4] << 8) | payload[5]);
    measurement->motor_temperature_degree_celsius = payload[6];
    measurement->controller_error_code = payload[7];
    measurement->last_update_time_millisecond = HAL_GetTick();
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

static void MotorMeasurement_ParsePayloadYaw(motor_measurement_t *measurement, const uint8_t payload[8])
{
    measurement->previous_encoder_count = measurement->encoder_count;
    measurement->encoder_count = (uint16_t)(((uint16_t)payload[7] << 8) | payload[6]);
    measurement->rotor_speed_revolution_per_minute = (int16_t)(((uint16_t)payload[5] << 8) | payload[4]);
    measurement->measured_torque_current_count = (int16_t)(((uint16_t)payload[3] << 8) | payload[2]);
    measurement->motor_temperature_degree_celsius = payload[1];
    measurement->controller_error_code = 0U;
    measurement->last_update_time_millisecond = HAL_GetTick();
}


motor_measurement_t gimbal_motor_measurements[6];//反馈接收数组
gimbal_c_get_data_t gimbal_received_data;
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
            MotorMeasurement_ParsePayload(&gimbal_motor_measurements[0],rx_data);
            break;
        }
        case CAN1_fire_r_motor_id:  //201
        {
            MotorMeasurement_ParsePayload(&gimbal_motor_measurements[1],rx_data);
            break;
        }
       
        case CAN1_fire_d_motor_id:  //203
        {
            MotorMeasurement_ParsePayload(&gimbal_motor_measurements[2],rx_data);
            break;
        }
        case CAN_MESSAGE_IDENTIFIER_PITCH_AXIS_FEEDBACK: //301
        {
            MotorMeasurement_ParsePayload(&gimbal_motor_measurements[3],rx_data);
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
            MotorMeasurement_ParsePayloadYaw(&gimbal_motor_measurements[5],rx_data);
            break;
        }
				 case CAN2_trigger_motor_id:  //201
        {
            MotorMeasurement_ParsePayload(&gimbal_motor_measurements[4],rx_data);
            break;
        }
        
		
		 case CAN2_recive_chassis_msg:
	    {
		  	get_gimbal_measure(&gimbal_received_data,rx_data);
		    break;
		}
		
        default:
        {    
             break;
        }
           
        }
    }
}

 /* CAN transmit */
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

void CAN_cmd_trigger(int16_t trigger,int16_t encoder_count)//触发
{
    uint32_t send_mail_box;
    trigger_tx_msg.StdId =  CAN1_trigger_flag_id;
    trigger_tx_msg.IDE = CAN_ID_STD;  //标准ID(11位数)
    trigger_tx_msg.RTR = CAN_RTR_DATA;  //请求（RTR）为数据帧
    trigger_tx_msg.DLC = 0x08;  //八位数据位
    trigger_can_send_data[0] = (trigger >> 8);
    trigger_can_send_data[1] = trigger;
    trigger_can_send_data[2] = (encoder_count >> 8);
    trigger_can_send_data[3] =encoder_count;
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
    /* 说明：top_mode（小陀螺）/ follow_mode、follow_mode_last（一键掉头至正位）
       因机器人加装机械臂已停用，由 chassis_data_task.c 固定填 0，
       此处保留协议位（bit4~bit2）不变，底盘侧对应解析也已注释。 */
    gimbal_send_chassis_data[5] = ((gimbal_send_data->chassis_mode<<6)&(3<<6))|((gimbal_send_data->cap_mode<<5)&(1<<5))|((gimbal_send_data->top_mode<<4)&(1<<4))|((gimbal_send_data->follow_mode<<3)&(1<<3))|((gimbal_send_data->follow_mode_last<<2)&(1<<2))|((gimbal_send_data->key_control<<1)&(1<<1));  //剩余一位
    gimbal_send_chassis_data[6] = (0);
    gimbal_send_chassis_data[7] = (0);
    HAL_CAN_AddTxMessage(&chassis_can, &gimbal_can_send_msg, gimbal_send_chassis_data, &send_mail_box);
}



void CAN_SendPitchAxisTorqueCurrentCommand(int16_t pitch_current)
{
    uint32_t send_mail_box;

    /* 3508 + C620: 0x1FF covers ESC ID 5..8, big-endian, range +-16384.
       The clamp also protects against any stale 30000-valued LQR path. */
    if (pitch_current >  16000) { pitch_current =  16000; }
    if (pitch_current < -16000) { pitch_current = -16000; }

    gimbal_pitch_tx_msg.StdId = CAN_MESSAGE_IDENTIFIER_PITCH_AXIS_COMMAND;
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


/* CAN measurement pointers */
//CAN1
const motor_measurement_t *GetRightFrictionWheelMotorMeasurementPointer(void)
{
    return &gimbal_motor_measurements[1];
}
const motor_measurement_t *GetLeftFrictionWheelMotorMeasurementPointer(void)
{
    return &gimbal_motor_measurements[0]; 
}
const motor_measurement_t *GetDownFrictionWheelMotorMeasurementPointer(void)
{
    return &gimbal_motor_measurements[2]; 
}
const motor_measurement_t *GetPitchAxisMotorMeasurementPointer(void)
{
    return &gimbal_motor_measurements[3];
}
const motor_measurement_t *GetTriggerMotorMeasurementPointer(void)
{
    return &gimbal_motor_measurements[4];
}
//CAN2

const motor_measurement_t *GetYawAxisMotorMeasurementPointer(void)
{
    return &gimbal_motor_measurements[5];
}





const gimbal_c_get_data_t *get_c_rm_data_point(void)
{
	return &gimbal_received_data;
}




