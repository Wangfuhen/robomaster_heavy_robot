#include "dt7_remote_read.h"
#include "main.h"
/*Ê¹ÓÃuart3¡ª¡ªdma*/

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

RC_ctrl_t dt7;

static void solve_dt7_data(volatile const uint8_t *sbus_buf, RC_ctrl_t *dt7); //½øÐÐ½âÂë

static uint8_t sbus_rx_buffer[2][SBUS_RX_BUF_NUM];  //»º³åÊý×é

void dt7_remote_read_init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, sbus_rx_buffer[0], SBUS_RX_BUF_NUM);  //Ë«»º³å¼ÓÔØ
}


const RC_ctrl_t *get_dt7_remote_control_point(void)
{
    return &dt7;
}

void dt7_remote_on_rx_event(uint16_t Size)
{
    if(Size == RC_FRAME_LENGTH)
    {
        solve_dt7_data(sbus_rx_buffer[0], &dt7);
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, sbus_rx_buffer[0], SBUS_RX_BUF_NUM);
}

static void  solve_dt7_data(volatile const uint8_t *sbus, RC_ctrl_t *dt7)
{
    //114  µÚ0
    if(sbus==NULL||dt7==NULL)
    {
        return;
    }
    else
    {
        dt7->rc.ch[0]=((sbus[0])|(sbus[1]<<8)) & 0x7ff;      //×ó±ß   ¡ª¡ª>0
        dt7->rc.ch[1]=((sbus[1]>>3)|(sbus[2]<<5)) &0x7ff;    

        dt7->rc.ch[2]=((sbus[2]>>6)|(sbus[3]<<2)|(sbus[4]<<10)) &0x7ff;   //ÓÒ±ß   ¡ª¡ª>2
        dt7->rc.ch[3]=((sbus[4]>>1)|(sbus[5]<<7)) &0x7ff;

        dt7->rc.s[0]= ((sbus[5]>>4)) &0x0003;  //×ó±ß
        dt7->rc.s[1]=((sbus[5]>>6))  &0x0003;  //ÓÒ±ß

        dt7->mouse.x=((sbus[6])|(sbus[7]<<8));
        dt7->mouse.y=((sbus[8])|(sbus[9]<<8));
        dt7->mouse.z=((sbus[10])|(sbus[11]<<8));
        dt7->mouse.press_l_last=dt7->mouse.press_l;
        dt7->mouse.press_r_last = dt7->mouse.press_r;
        dt7->mouse.press_l=sbus[12];
        dt7->mouse.press_r=sbus[13];
        dt7->key.v = sbus[14] | (sbus[15] << 8);                    //¼üÅÌÍ¨µÀ
        dt7->rc.ch[4] = sbus[16] | (sbus[17] << 8);                 //²¨ÂÖ

        dt7->rc.ch[0] -= RC_CH_VALUE_OFFSET;
        dt7->rc.ch[1] -= RC_CH_VALUE_OFFSET;
        dt7->rc.ch[2] -= RC_CH_VALUE_OFFSET;
        dt7->rc.ch[3] -= RC_CH_VALUE_OFFSET;
        dt7->rc.ch[4] -= RC_CH_VALUE_OFFSET;

    }
}
