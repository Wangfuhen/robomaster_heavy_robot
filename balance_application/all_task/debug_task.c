#include "debug_task.h"
#include "ins_task.h"
#include "cmsis_os2.h"
#include <string.h>
extern void Chassis_Task_Report_Update(void);
/**
 * @brief   debug_task.c —— 调试数据上传任务（VOFA+ 上位机示波器）
 *
 * 职责：把 yaw/pitch/roll 等状态量打包，通过 USART 发送到 VOFA+ 上位机实时曲线显示，
 *       便于联调观察姿态/控制量。
 */
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern fp32 yaw;
extern fp32  pitch;
extern fp32 roll;
const fp32 *send_data[CH_COUNT];
fp32  a=0;
/**
 * @brief   调试上传任务（CMSIS-RTOS2 线程入口）
 * @param   argument  CMSIS-RTOS2 线程参数（未使用）
 * @note    每 10ms 调用一次 debug_to_vofa，把 yaw/pitch/roll/计数 a 四路数据上传，
 *          等效采样率 100Hz（见 osDelay(10) 与注释“频率100hz”）。
 */
void Crg_test_Task(void *argument)
{
	const fp32 *send_data[CH_COUNT] =
	{
		&yaw,
		&pitch,
		&roll,
		&a
	};
	while(1)
	{
		Chassis_Task_Report_Update();
		debug_to_vofa(send_data,CH_COUNT);
		a+=1.0f;
		osDelay(10); //发送频率100hz
	}
}
