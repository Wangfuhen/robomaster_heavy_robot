#include "double_dma_init.h"
#include "main.h"

void Double_dma_init(UART_HandleTypeDef *huart,DMA_HandleTypeDef *hdma_usart_rx,uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_len)
{
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);//DMA Enable Receiver  使能开启串口的dma接收

    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);//开启空闲中断

    do{
    __HAL_DMA_DISABLE(hdma_usart_rx);  //先暂时关闭该通道的dma避免出现错误
    }while(hdma_usart_rx->Instance->CR & DMA_SxCR_EN);

    /*关闭后先初始化栓缓冲区的设定*/
    hdma_usart_rx->Instance->PAR=(uint32_t)&(huart->Instance->DR); //设置dma的数据源来自uart
    /*设置数据目标地址 缓存区1*/
    hdma_usart_rx->Instance->M0AR=(uint32_t)(rx1_buf);
    /*设置数据目标地址 缓存区2*/
    hdma_usart_rx->Instance->M1AR=(uint32_t)(rx2_buf);
    hdma_usart_rx->Instance->NDTR=dma_buf_len;
    SET_BIT(hdma_usart_rx->Instance->CR, DMA_SxCR_DBM);//enable doubule memory buffer

    __HAL_DMA_ENABLE(hdma_usart_rx);//开始正式启动dma双缓冲接收
}
