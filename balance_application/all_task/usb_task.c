#include "usb_task.h"
#include "cmsis_os.h"
#include "usbd_cdc_if.h"
#include "ins_task.h"
#include "gimbal_task.h"
#include "usb_device.h"
#include "can_user.h"
extern gimbal_c_get_data_t gimbal_received_data;
autom_pc hero_pc={0};
static const fp32 *gimbal_INT_gyro_angle_point;
static const fp32 *gimbal_INT_q_point;
static const fp32 *gimbal_INT_gyro_point;
fp32 shoot_speed=10.40f;
static uint8_t send_data[36];
static void Usb_Send_Data(void);
uint8_t aim_auto=0;
 
void Usb_Task(void const * argument)
 {
	MX_USB_DEVICE_Init();  //usb初始化
	vTaskDelay(500);
	while(1)
	{
		hero_pc.pc_enable_flag--;
		if(hero_pc.pc_enable_flag<-20)
		{
			hero_pc.pc_enable_flag=-20;
		}
		if(hero_pc.pc_enable_flag<0)
		{
			hero_pc.pc_distance=-1;
			hero_pc.pc_fire_mode=0;
		}
		if(auto_mode_flag)
		{
			aim_auto=1;
		}
		else if(auto_mode_flag_qian)
		{
				aim_auto = 2;
		}
		else {
		   aim_auto = 0;
		}

		
		
		Usb_Send_Data();
		vTaskDelay(1);
	}
 }
 

  const autom_pc *get_crg_pc_point(void)
{
	return (&hero_pc);
 }

static void Usb_Send_Data(void)
{ 
  shoot_speed=16.0f;
  gimbal_INT_gyro_angle_point=get_INS_angle_point();
	gimbal_INT_q_point=get_INS_quat_point();
	gimbal_INT_gyro_point=get_gyro_data_point();
	send_data[0]=0xff;  //֡ͷ
	send_data[1]=aim_auto; 

	memcpy(&send_data[2],&gimbal_INT_gyro_point[2],sizeof(gimbal_INT_gyro_point[2]));  
	memcpy(&send_data[6],&gimbal_INT_gyro_point[1],sizeof(gimbal_INT_gyro_point[1]));
	memcpy(&send_data[10],&gimbal_INT_gyro_point[0],sizeof(gimbal_INT_gyro_point[0]));    
	memcpy(&send_data[14],&gimbal_INT_q_point[0],sizeof(gimbal_INT_q_point[0]));  
	memcpy(&send_data[18],&gimbal_INT_q_point[1],sizeof(gimbal_INT_q_point[1]));  
	memcpy(&send_data[22],&gimbal_INT_q_point[2],sizeof(gimbal_INT_q_point[2])); 
	memcpy(&send_data[26],&gimbal_INT_q_point[3],sizeof(gimbal_INT_q_point[3]));
	memcpy(&send_data[30],&(shoot_speed),sizeof(shoot_speed));
	send_data[34]=0x22;  
	send_data[35]=0x0d;
	CDC_Transmit_FS(send_data,sizeof(send_data));
}
