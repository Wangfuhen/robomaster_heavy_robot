#include "vofa.h"

fp32 send_data_msg[4]={1.1,2,4,5};
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
vofa_msg_justfloat gimbal_msg=
{
{0},
{0x00,0x00,0x80,0x7f},
}; //初始化

int debug_to_vofa(const fp32 *send_data_msg[],uint16_t data_len)
{
    for(uint16_t i =0;i<data_len;i++)
    {
        gimbal_msg.data[i]=*(send_data_msg[i]);  //为一块打包迁移
    }   
    if(HAL_UART_Transmit(&huart1,(uint8_t *)&gimbal_msg,sizeof(gimbal_msg),100) != HAL_OK ) //发送失败
    {  
        return(0); //发送失败
		
    }
	return (1);  //发送成功
}

