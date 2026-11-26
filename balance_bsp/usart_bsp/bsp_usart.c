#include "bsp_usart.h"
#include "main.h"
#include "string.h"
#include "cmsis_os.h"


extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
HAL_StatusTypeDef Bsp_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
  uint32_t *tmp;
  uint32_t tmp1 = 0;
  
  tmp1 = huart->gState;
  if((tmp1 == HAL_UART_STATE_READY) || (tmp1 == HAL_UART_STATE_BUSY_TX))
  {
    if((pData == NULL ) || (Size == 0)) 
    {
      return HAL_ERROR;
    }
		
    
    /* Process Locked */
    __HAL_LOCK(huart);
    
    huart->pRxBuffPtr = pData;
    huart->RxXferSize = Size;
    
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    /* Check if a transmit process is ongoing or not */
    if(huart->gState == HAL_UART_STATE_BUSY_TX)
    {
      huart->gState = HAL_UART_STATE_BUSY_TX_RX;
    }
    else
    {
      huart->gState = HAL_UART_STATE_BUSY_RX;
    }
    
    /* Enable the DMA Stream */
    tmp = (uint32_t*)&pData;
    HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, *(uint32_t*)tmp, Size);
    
    /* Enable the DMA transfer for the receiver request by setting the DMAR bit 
    in the UART CR3 register */
    huart->Instance->CR3 |= USART_CR3_DMAR;
    
    /* Process Unlocked */
    __HAL_UNLOCK(huart);
    
    return HAL_OK;
  }
  else
  {
    return HAL_BUSY; 
  }
}
HAL_StatusTypeDef Bsp_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
	//__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);//开启串口空闲中断
	Bsp_UART_Receive_DMA(huart,pData,Size);//利用DMA接受数据
	
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);//开启串口空闲中断
	return HAL_OK;
}
void Bsp_UsartIdleHanlder(UART_HandleTypeDef *huart,uint16_t Size)
{
	uint32_t DMA_FLAGS;//根据串口的不同来选择清除不同的DMA标志位
//  uint32_t tmp;
	
	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
  {
				//关闭dma
	  __HAL_DMA_DISABLE(huart->hdmarx);
		
		UART_IdleRxCallback(huart);
		
		


		
		DMA_FLAGS = __HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmarx);	
		__HAL_DMA_CLEAR_FLAG(huart->hdmarx,DMA_FLAGS);
		

	 
		__HAL_DMA_SET_COUNTER(huart->hdmarx,Size);
		
		__HAL_DMA_ENABLE(huart->hdmarx);

		
		/*清除IDLE标志位*/
    __HAL_UART_CLEAR_IDLEFLAG(huart);
		
	}
}
//CRC16打包
void ISO14443AAppendCRCA(void* Buffer, uint16_t ByteCount) {
    uint16_t Checksum = 0x6363;
    uint8_t* DataPtr = (uint8_t*) Buffer;

    while(ByteCount--) {
        uint8_t Byte = *DataPtr++;

        Byte ^= (uint8_t) (Checksum & 0x00FF);
        Byte ^= Byte << 4;

        Checksum = (Checksum >> 8) ^ ( (uint16_t) Byte << 8 ) ^
                ( (uint16_t) Byte << 3 ) ^ ( (uint16_t) Byte >> 4 );
    }

    *DataPtr++ = (Checksum >> 0) & 0x00FF;
    *DataPtr = (Checksum >> 8) & 0x00FF;
}
//CRC16解包
uint8_t ISO14443ACheckCRCA(void* Buffer, uint16_t ByteCount)
{
    uint16_t Checksum = 0x6363;
    uint8_t* DataPtr = (uint8_t*) Buffer;

    while(ByteCount--) {
        uint8_t Byte = *DataPtr++;

        Byte ^= (uint8_t) (Checksum & 0x00FF);
        Byte ^= Byte << 4;

        Checksum = (Checksum >> 8) ^ ( (uint16_t) Byte << 8 ) ^
                ( (uint16_t) Byte << 3 ) ^ ( (uint16_t) Byte >> 4 );
    }

    return (DataPtr[0] == ((Checksum >> 0) & 0xFF)) && (DataPtr[1] == ((Checksum >> 8) & 0xFF));
}

uint8_t ISO14443ACheckLen(uint8_t* Buffer)
{
  if((Buffer[0]+Buffer[1])==0xff&&Buffer[0]<PC_BUF_LEN-2)
		return 1;
	else
		return 0;
}
fp32 laser_distance=0;
void UART_IdleRxCallback(UART_HandleTypeDef *huart)	
{
	
}


uint8_t usart_state = 0;
//void UART_Send_Buff(UART_HandleTypeDef *huart,Command command,uint8_t *data_input, uint8_t data_len)
//{
//  uint8_t  Buffer[PC_BUF_LEN];
//	Buffer[0] = 0x55;
//	Buffer[1] = command;
//	Buffer[2] = data_len;
//	Buffer[3] = 0xff - data_len;
//	memcpy(Buffer + HEAD_LEN, data_input, data_len);
//	ISO14443AAppendCRCA(Buffer, data_len + HEAD_LEN);
//	usart_state = HAL_UART_Transmit(huart, Buffer, data_len + HEAD_LEN + 2,10);
//	//HAL_UART_Transmit_DMA(huart, Buffer, data_len + HEAD_LEN + 2);
//}
void usart1_tx_dma_init(uint8_t *data, uint16_t len)
{
    //enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);
    hdma_usart1_tx.Instance->M0AR = (uint32_t)(data);
    hdma_usart1_tx.Instance->NDTR = len;
}

void usart6_tx_dma_init(uint8_t *data,uint16_t len)
{

    //enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);

      //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }

    hdma_usart6_tx.Instance->PAR = (uint32_t) & (USART6->DR);
    hdma_usart6_tx.Instance->M0AR = (uint32_t)(data);
    hdma_usart6_tx.Instance->NDTR = len;

}
void usart1_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);

    hdma_usart1_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart1_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart1_tx);
}


void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);



    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    
    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_LISR_TCIF1);

    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    __HAL_DMA_SET_COUNTER(&hdma_usart6_rx, dma_buf_num);

    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);


    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }

    hdma_usart6_tx.Instance->PAR = (uint32_t) & (USART6->DR);

}



void usart6_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx, DMA_HISR_TCIF6);

    hdma_usart6_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart6_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart6_tx);
}


