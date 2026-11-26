#include "usb_task.h"
#include "cmsis_os.h"
#include "usbd_cdc_if.h"
#include "ins_task.h"
#include "gimbal_task.h"
#include "usb_device.h"
#include "can_user.h"
extern gimbal_c_get_data_t gimbal_can_get;
autom_pc hero_pc;
static const fp32 *gimbal_INT_gyro_angle_point;
static uint8_t send_data[16];

static void Usb_Send_Data(void);
uint8_t aim_auto=0;
 
 void Usb_Task(void const * argument)
 {
	MX_USB_DEVICE_Init();  //usb初始化
	vTaskDelay(500);
	while(1)
	{
		if(gimbal_can_get.robot_id==3)
		{
			aim_auto=1;
		}
		else if(gimbal_can_get.robot_id==103)
		{
			aim_auto=0;
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
    gimbal_INT_gyro_angle_point=get_INS_angle_point();
	send_data[0]=0xff;  //֡ͷ
	send_data[1]=aim_auto; 
	memcpy(&send_data[2],&gimbal_INT_gyro_angle_point[2],sizeof(gimbal_INT_gyro_angle_point[2]));  //roll
	memcpy(&send_data[6],&gimbal_INT_gyro_angle_point[1],sizeof(gimbal_INT_gyro_angle_point[1]));  //pitch
	memcpy(&send_data[10],&gimbal_INT_gyro_angle_point[0],sizeof(gimbal_INT_gyro_angle_point[0]));  //yaw
	send_data[14]='c';
	send_data[15]=0x0d;  //֡β
	CDC_Transmit_FS(send_data,sizeof(send_data));
}
