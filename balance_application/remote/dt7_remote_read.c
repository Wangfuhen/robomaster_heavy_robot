#include "dt7_remote_read.h"
#include "main.h"

//临时变量，测试
volatile uint16_t wheel_raw_dbg = 0;
/*使用uart3——dma*/

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

RC_ctrl_t dt7;

static void solve_dt7_data(volatile const uint8_t *sbus_buf, RC_ctrl_t *dt7); //进行解码

static uint8_t sbus_rx_buffer[2][SBUS_RX_BUF_NUM];  //缓冲数组

void dt7_remote_read_init(void)
{
    Double_dma_init(&huart3,&hdma_usart3_rx,sbus_rx_buffer[0],sbus_rx_buffer[1],RC_FRAME_LENGTH);  //双缓冲加载
}


const RC_ctrl_t *get_dt7_remote_control_point(void)
{
    return &dt7;
}

//串口中断
void USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */
    
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)  //接收到一帧数据后解码
            {
                solve_dt7_data(sbus_rx_buffer[0], &dt7);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR; //不定长数据接收

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                solve_dt7_data(sbus_rx_buffer[1], &dt7);
            }
			
        }
    }
}

static void  solve_dt7_data(volatile const uint8_t *sbus, RC_ctrl_t *dt7)
{
    //114  第0
    if(sbus==NULL||dt7==NULL)
    {
        return;
    }
    else
    {
        dt7->rc.ch[0]=((sbus[0])|(sbus[1]<<8)) & 0x7ff;      //左边   ——>0
        dt7->rc.ch[1]=((sbus[1]>>3)|(sbus[2]<<5)) &0x7ff;    

        dt7->rc.ch[2]=((sbus[2]>>6)|(sbus[3]<<2)|(sbus[4]<<10)) &0x7ff;   //右边   ——>2
        dt7->rc.ch[3]=((sbus[4]>>1)|(sbus[5]<<7)) &0x7ff;

        dt7->rc.s[0]= ((sbus[5]>>4)) &0x0003;  //左边
        dt7->rc.s[1]=((sbus[5]>>6))  &0x0003;  //右边

        dt7->mouse.x=((sbus[6])|(sbus[7]<<8));
        dt7->mouse.y=((sbus[8])|(sbus[9]<<8));
        dt7->mouse.z=((sbus[10])|(sbus[11]<<8));
        dt7->mouse.press_l_last=dt7->mouse.press_l;
        dt7->mouse.press_r_last = dt7->mouse.press_r;
        dt7->mouse.press_l=sbus[12];
        dt7->mouse.press_r=sbus[13];
        dt7->key.v = sbus[14] | (sbus[15] << 8);                    //键盘通道
        dt7->rc.ch[4] = sbus[16] | (sbus[17] << 8); 
        wheel_raw_dbg = (uint16_t)sbus[16]
              | ((uint16_t)sbus[17] << 8);                //波轮
        //dt7->rc.ch[4]= dt7->rc.ch[4]&0x07ff;

        dt7->rc.ch[0] -= RC_CH_VALUE_OFFSET;
        dt7->rc.ch[1] -= RC_CH_VALUE_OFFSET;
        dt7->rc.ch[2] -= RC_CH_VALUE_OFFSET;
        dt7->rc.ch[3] -= RC_CH_VALUE_OFFSET;
        dt7->rc.ch[4] -= RC_CH_VALUE_OFFSET;

    }
}
