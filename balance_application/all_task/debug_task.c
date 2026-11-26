#include "debug_task.h"
#include "ins_task.h"
#include "can_user.h"
#include "vofa.h"
#include "chassis_task.h"


extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern fp32 yaw;
extern fp32 pitch;
extern fp32 roll;
extern fp32 motor_chassis[4];
extern chassis_control_t hero_chassis;

const fp32 *send_data[CH_COUNT];
fp32  a=0;


void Crg_test_Task(void const * argument)
{
    (void)argument;

    // VOFA 四个通道依次对应：
    // CH0：云台相对底盘的实际角度
    // CH1：云台相对底盘的目标角度
    // CH2：底盘最终旋转速度设定值
    // CH3：底盘 Z 轴陀螺仪角速度

    const fp32 *send_data[CH_COUNT] =
    {
        &hero_chassis.relative_angle,
        &hero_chassis.set_relative_angle,
        &hero_chassis.wz_set,
        &hero_chassis.chassis_gyro
    };

    while (1)
    {
        debug_to_vofa(send_data, CH_COUNT);

        vTaskDelay(10);
    }
}