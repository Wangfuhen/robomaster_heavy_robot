#include "vofa.h"

//fp32 send_data_msg[4]={1.1,2,4,5};
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart6_tx;
vofa_msg_justfloat gimbal_msg=
{
{0},
{0x00,0x00,0x80,0x7f},
}; //初始化

// int debug_to_vofa(const fp32 *send_data_msg,uint16_t data_len)
// {
//     for(uint16_t i =0;i<data_len;i++)
//     {
//         gimbal_msg.data[i]=send_data_msg[i];  //为一块打包迁移
//     }
//     while(HAL_DMA_GetState(&hdma_usart6_tx) == HAL_DMA_STATE_BUSY)
//     {
//        vTaskDelay(1);
//     }
//     //__HAL_DMA_DISABLE(hdma_usart_tx);  //暂时关闭dma通道混乱
// 	__HAL_DMA_ENABLE(&hdma_usart6_tx);
//     if(HAL_UART_Transmit(&huart6,(uint8_t *)&gimbal_msg,sizeof(gimbal_msg),1000) != HAL_OK ) //发送失败
//     {  
//         return(0); //发送失败
		
//     }
// 	return (1);  //发送成功
// }



int debug_to_vofa(const fp32 *data, uint16_t data_len)
{
    uint16_t i;

    if (data == NULL || data_len == 0 || data_len > CH_COUNT)
    {
        return 0;
    }

    /* 上一帧尚未发送完，不能覆盖 DMA 正在读取的数据 */
    if (huart1.gState != HAL_UART_STATE_READY)
    {
        return 0;
    }

    for (i = 0; i < CH_COUNT; i++)
    {
        gimbal_msg.data[i] = (i < data_len) ? data[i] : 0.0f;
    }

    return HAL_UART_Transmit_DMA(
        &huart1,
        (uint8_t *)&gimbal_msg,
        sizeof(gimbal_msg)
    ) == HAL_OK;
}


