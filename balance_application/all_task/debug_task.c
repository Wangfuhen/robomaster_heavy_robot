#include "debug_task.h"
#include "ins_task.h"
#include "dt7_remote_read.h"
#include "can_user.h"
#include "usb_task.h"

extern gimbal_c_send_data_t gimbal_send_chassis_data;
extern volatile uint16_t wheel_raw_dbg;
// extern UART_HandleTypeDef huart1;
// extern DMA_HandleTypeDef hdma_usart6_tx;

void Crg_test_Task(void const * argument)
{
    int b=0;
static	fp32 channels[CH_COUNT];
	const RC_ctrl_t *remote = get_dt7_remote_control_point();
    const motor_measurement_t *yaw_motor = GetYawAxisMotorMeasurementPointer();
    const motor_measurement_t *pitch_motor = GetPitchAxisMotorMeasurementPointer();
    const fp32 *ins_angle = get_INS_angle_point();
    const autom_pc *pc = get_crg_pc_point();
    const motor_measurement_t *trigger_motor = GetTriggerMotorMeasurementPointer();
	while(1)
	{
		ist8310_data[0]=b++;
		channels[0] = (fp32)yaw_motor->encoder_count;
        channels[1] = (fp32)pitch_motor->encoder_count;          // pitch encoder count
        channels[2] = ins_angle[INS_YAW_ADDRESS_OFFSET] * 57.295779513f;   // yaw IMU angle (deg)
        channels[3] = ins_angle[INS_PITCH_ADDRESS_OFFSET] * 57.295779513f; // pitch IMU angle (deg)
	    	channels[4] = ins_angle[INS_ROLL_ADDRESS_OFFSET] * 57.295779513f;      // roll (deg)
	     	channels[5] = (fp32)pitch_motor->rotor_speed_revolution_per_minute;
	      channels[6] = 
        channels[7] = 
        channels[8] = 0;
        channels[9] =0;
        channels[10] = 0;
        channels[11] = 0;
        channels[12] =(fp32)gimbal_send_chassis_data.top_mode;

		debug_to_vofa(channels, CH_COUNT);
		vTaskDelay(10); //发送频率100hz
	}
}



#include "debug_task.h"
#include <math.h>

/* 可以在 Keil Watch 中观察 */
volatile uint32_t vofa_send_ok = 0;
volatile uint32_t vofa_send_fail = 0;
int a=0;

// void Crg_test_Task(void const *argument)
// {
//     fp32 phase = 0.0f;
//     fp32 channels[CH_COUNT];

//     (void)argument;

//     while (1)
//     {
//         a++;
//         channels[0] = sinf(phase);  /* 正弦波 */
//         channels[1] = cosf(phase);  /* 余弦波 */
//         channels[2] = phase;        /* 锯齿波 */
//         channels[3] = 1.0f;         /* 水平线 */

//         if (debug_to_vofa(channels, CH_COUNT))
//         {
//             vofa_send_ok++;
//         }
//         else
//         {
//             vofa_send_fail++;
//         }

//         phase += 0.06283185f;
//         if (phase >= 6.2831853f)
//         {
//             phase -= 6.2831853f;
//         }

//         osDelay(10);  /* 约每 10 ms 发送一次 */
//     }
// }
