#include "can_user.h"
#include "main.h"
#include "string.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static void CAN_ParseChassisMotorMeasurement(motor_measurement_t *measurement, const uint8_t *payload)
{
    measurement->previous_encoder_count = measurement->encoder_count;
    measurement->encoder_count = (uint16_t)(((uint16_t)payload[0] << 8) | payload[1]);
    measurement->rotor_speed_revolution_per_minute = (int16_t)(((uint16_t)payload[2] << 8) | payload[3]);
    measurement->commanded_torque_current_count = (int16_t)(((uint16_t)payload[4] << 8) | payload[5]);
    measurement->temperature_degree_celsius = payload[6];
}

static void CAN_ParseGimbalCommand(gimbal_command_data_t *command, const uint8_t *payload)
{
    command->forward_channel_value = (int16_t)(((uint16_t)payload[0] << 8) | payload[1]);
    command->leftward_channel_value = (int16_t)(((uint16_t)payload[2] << 8) | payload[3]);
    command->forward_key_value = (uint8_t)((payload[4] >> 6) & 0x03U);
    command->leftward_key_value = (uint8_t)((payload[4] >> 4) & 0x03U);
    command->operating_mode = (uint8_t)((payload[5] >> 6) & 0x03U);
    command->super_capacitor_mode = (uint8_t)((payload[5] >> 5) & 0x01U);
    command->top_spin_mode = (uint8_t)((payload[5] >> 4) & 0x01U);
    command->follow_gimbal_state = (uint8_t)((payload[5] >> 3) & 0x01U);
    command->previous_follow_gimbal_state = (uint8_t)((payload[5] >> 2) & 0x01U);
    command->keyboard_control_enabled = (uint8_t)((payload[5] >> 1) & 0x01U);
}

static void CAN_ParseSuperCapacitorMeasurement(super_capacitor_measurement_t *measurement, const uint8_t *payload)
{
    measurement->voltage_volt = (float)(((uint16_t)payload[2] << 8) | payload[3]) * 0.001f;
}

motor_measurement_t gimbal_motor_measurements[1];
motor_measurement_t chassis_motor_measurements[4];
gimbal_command_data_t gimbal_command = 
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
	
super_capacitor_measurement_t super_capacitor_measurement;

static uint8_t tx_data[8] = {0};

static CAN_TxHeaderTypeDef chassis_command_header;
static uint8_t chassis_command_payload[8];

static CAN_TxHeaderTypeDef gimbal_data_header;
static uint8_t gimbal_data_payload[8];

static CAN_TxHeaderTypeDef super_capacitor_command_header;
static uint8_t super_capacitor_command_payload[8];

static CAN_TxHeaderTypeDef chassis_pitch_header;
static uint8_t chassis_pitch_payload[4];


void CAN_ReadGimbalMotorEncoderCount(motor_measurement_t *measurement,uint8_t *payload)                       
    {                                                                  
        (measurement)->encoder_count= ((payload)[2]<<8)|((payload)[3]);
      //(measurement)->ecdd= (payload)[0];			
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
        case CAN_CHASSIS_MOTOR_1_IDENTIFIER:  //201
        {
            CAN_ParseChassisMotorMeasurement(&chassis_motor_measurements[0],rx_data);
            break;
        }
        case CAN_CHASSIS_MOTOR_2_IDENTIFIER:  //202
        {
            CAN_ParseChassisMotorMeasurement(&chassis_motor_measurements[1],rx_data);
            break;
        }
		case CAN_CHASSIS_MOTOR_3_IDENTIFIER:  //203
        {
            CAN_ParseChassisMotorMeasurement(&chassis_motor_measurements[2],rx_data);
            break;
        }
		case CAN_CHASSIS_MOTOR_4_IDENTIFIER:  //204
        {
					
            CAN_ParseChassisMotorMeasurement(&chassis_motor_measurements[3],rx_data);
            break;
        }
		case CAN_SUPER_CAPACITOR_TO_CHASSIS_IDENTIFIER: //304
		{
			CAN_ParseSuperCapacitorMeasurement(&super_capacitor_measurement,rx_data);
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
            CAN_ReadGimbalMotorEncoderCount(&gimbal_motor_measurements[0],rx_data);
            break;
        }
				
				case CAN_GIMBAL_TO_CHASSIS_IDENTIFIER:
				{
					CAN_ParseGimbalCommand(&gimbal_command,rx_data);
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

void CAN_SendChassisCommand(int16_t motor_1_command_current_count, int16_t motor_2_command_current_count, int16_t motor_3_command_current_count, int16_t motor_4_command_current_count)
{
     uint32_t send_mailbox;
    chassis_command_header.StdId = CAN_CHASSIS_COMMAND_IDENTIFIER;
    chassis_command_header.IDE = CAN_ID_STD;
    chassis_command_header.RTR = CAN_RTR_DATA;
    chassis_command_header.DLC = 0x08;
    chassis_command_payload[0] = motor_1_command_current_count >> 8;
    chassis_command_payload[1] = motor_1_command_current_count;
    chassis_command_payload[2] = motor_2_command_current_count >> 8;
    chassis_command_payload[3] = motor_2_command_current_count;
    chassis_command_payload[4] = motor_3_command_current_count >> 8;
    chassis_command_payload[5] = motor_3_command_current_count;
    chassis_command_payload[6] = motor_4_command_current_count >> 8;
    chassis_command_payload[7] = motor_4_command_current_count;

    HAL_CAN_AddTxMessage(&hcan1, &chassis_command_header, chassis_command_payload, &send_mailbox);
}

void CAN_SendGimbalData(RM_Referee_system_t *referee_data_pointer)
{
	uint32_t send_mailbox;
    gimbal_data_header.StdId =  CAN_CHASSIS_TO_GIMBAL_IDENTIFIER;
    gimbal_data_header.IDE = CAN_ID_STD;  //标准ID(11位数)
    gimbal_data_header.RTR = CAN_RTR_DATA;  //请求（RTR）为数据帧
    gimbal_data_header.DLC = 0x0A;  //八位数据位
    gimbal_data_payload[0] = (referee_data_pointer->robot_state.robot_id);
    gimbal_data_payload[1] = (referee_data_pointer->robot_state.robot_level);
    gimbal_data_payload[2] = (referee_data_pointer->robot_state.shooter_barrel_cooling_value);
    gimbal_data_payload[3] = (uint8_t)(referee_data_pointer->shoot_data_t.bullet_speed*10);
    gimbal_data_payload[4] = (referee_data_pointer->robot_state.shooter_barrel_heat_limit>>8);
    gimbal_data_payload[5] = (referee_data_pointer->robot_state.shooter_barrel_heat_limit);
    gimbal_data_payload[6] = (referee_data_pointer->power_heat_data_t.shooter_17mm_1_barrel_heat>>8);
    gimbal_data_payload[7] = (referee_data_pointer->power_heat_data_t.shooter_17mm_1_barrel_heat);
    HAL_CAN_AddTxMessage(&hcan2, &gimbal_data_header, gimbal_data_payload, &send_mailbox);
}

void CAN_SendSuperCapacitorCommand(RM_Referee_system_t *referee_data_pointer)
{
	uint32_t send_mailbox;
    super_capacitor_command_header.StdId =  CAN_CHASSIS_TO_SUPER_CAPACITOR_IDENTIFIER;
    super_capacitor_command_header.IDE = CAN_ID_STD;    //标准ID(11位数)
    super_capacitor_command_header.RTR = CAN_RTR_DATA;  //请求（RTR）为数据帧
    super_capacitor_command_header.DLC = 0x08;          //八位数据位
    super_capacitor_command_payload[0] = (referee_data_pointer->robot_state.chassis_power_limit>>8);
    super_capacitor_command_payload[1] = (referee_data_pointer->robot_state.chassis_power_limit);
    super_capacitor_command_payload[2] = (referee_data_pointer->power_heat_data_t.buffer_energy>>8);
    super_capacitor_command_payload[3] = (referee_data_pointer->power_heat_data_t.buffer_energy);
    super_capacitor_command_payload[4] = (0);
    super_capacitor_command_payload[5] = (0);
    super_capacitor_command_payload[6] = (0);
    super_capacitor_command_payload[7] = (0);
	HAL_CAN_AddTxMessage(&hcan1, &super_capacitor_command_header, super_capacitor_command_payload, &send_mailbox);
}
/**
  * @brief  Can总线发送控制参数
  * @param
  * @retval
  */
void CAN_SendChassisReadCommand(uint8_t id,int16_t command_current_count)
{
    // CAN_TxHeaderTypeDef chassis_command_header;             /**!< can通信发送协议头 */
    uint32_t send_mailbox= CAN_TX_MAILBOX0;

    chassis_command_header.StdId = 0x140+id;
    chassis_command_header.IDE = CAN_ID_STD;
    chassis_command_header.RTR = CAN_RTR_DATA;
    chassis_command_header.DLC = 0x08;
    chassis_command_payload[0] = 0xA1;
    chassis_command_payload[1] = 0;
    chassis_command_payload[2] = 0;
    chassis_command_payload[3] = 0;
    chassis_command_payload[4] = command_current_count;
    chassis_command_payload[5] = (command_current_count>>8);
    chassis_command_payload[6] = 0;
    chassis_command_payload[7] = 0;

    //找到空的发送邮箱 把数据发送出去
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0);	// 如果三个发送邮箱都阻塞了就等待直到其中某个邮箱空闲
    if ((hcan1.Instance->TSR & CAN_TSR_TME0) != RESET)
    {   // 检查发送邮箱0状态 如果邮箱0空闲就将待发送数据放入FIFO0
        send_mailbox = CAN_TX_MAILBOX0;
    }
    else if ((hcan1.Instance->TSR & CAN_TSR_TME1) != RESET)
    {
        send_mailbox = CAN_TX_MAILBOX1;
    }
    else if ((hcan1.Instance->TSR & CAN_TSR_TME2) != RESET)
    {
        send_mailbox = CAN_TX_MAILBOX2;
    }


    if(HAL_CAN_AddTxMessage(&hcan2, &chassis_command_header, chassis_command_payload, (uint32_t *)&send_mailbox)==HAL_OK) {};
}
void CAN_SendChassisPitchToGimbal(fp32 pitch_radian)
{
	uint32_t send_mailbox;
    chassis_pitch_header.StdId =  CAN_CHASSIS_PITCH_TO_GIMBAL_IDENTIFIER;
    chassis_pitch_header.IDE = CAN_ID_STD;    //标准ID(11位数)
    chassis_pitch_header.RTR = CAN_RTR_DATA;  //请求（RTR）为数据帧
    chassis_pitch_header.DLC = 0x04;          //八位数据位
	memcpy(chassis_pitch_payload,&pitch_radian,sizeof(pitch_radian));
	HAL_CAN_AddTxMessage(&hcan2, &chassis_pitch_header, chassis_pitch_payload, &send_mailbox);
}

/*CAN_point_get*/
//CAN1
const motor_measurement_t *get_motor_chassis_motor_measure_point(uint8_t i)
{
    return &chassis_motor_measurements[i&3];
}
//CAN2

const motor_measurement_t *get_motor_gimbal_yaw_measure_point(void)
{
    return &gimbal_motor_measurements[0];
}

const super_capacitor_measurement_t *get_super_cap_measure_point(void)
{
    return &super_capacitor_measurement;
}

const gimbal_command_data_t *get_c_gimbal_data_point(void)
{
	return &gimbal_command;
}
