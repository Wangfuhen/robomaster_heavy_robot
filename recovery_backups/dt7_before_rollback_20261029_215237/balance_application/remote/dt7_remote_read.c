#include "dt7_remote_read.h"
#include "main.h"
/*使用uart3——dma*/

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

RC_ctrl_t dt7;
volatile uint32_t dt7_rx_frame_count = 0;
volatile uint32_t dt7_rx_bad_length_count = 0;
volatile uint32_t dt7_rx_invalid_data_count = 0;
volatile uint32_t dt7_rx_uart_error_count = 0;
volatile uint16_t dt7_rx_last_length = 0;

static uint8_t solve_dt7_data(volatile const uint8_t *sbus_buf, RC_ctrl_t *dt7); //进行解码

static uint8_t sbus_rx_buffer[2][SBUS_RX_BUF_NUM];  //缓冲数组

void dt7_remote_read_init(void)
{
    Double_dma_init(&huart3,&hdma_usart3_rx,sbus_rx_buffer[0],sbus_rx_buffer[1],SBUS_RX_BUF_NUM);  //双缓冲加载
}


const RC_ctrl_t *get_dt7_remote_control_point(void)
{
    return &dt7;
}

//串口中断
void USART3_IRQHandler(void)
{
    uint32_t status = huart3.Instance->SR;
    uint32_t error_flags = status & (UART_FLAG_PE | UART_FLAG_FE |
                                     UART_FLAG_NE | UART_FLAG_ORE);
    uint16_t received_length;
    uint8_t completed_buffer;

    if ((status & UART_FLAG_IDLE) == 0U)
    {
        if (error_flags != 0U)
        {
            __HAL_UART_CLEAR_PEFLAG(&huart3);
            dt7_rx_uart_error_count++;
        }
        return;
    }

    /* Clearing IDLE also clears the UART error flags (SR then DR). */
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while ((hdma_usart3_rx.Instance->CR & DMA_SxCR_EN) != 0U)
    {
    }

    completed_buffer = ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) != 0U) ? 1U : 0U;
    received_length = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;
    dt7_rx_last_length = received_length;

    /* Use the same DMA capacity at startup and after every IDLE event. */
    hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;
    hdma_usart3_rx.Instance->CR ^= DMA_SxCR_CT;
    __HAL_DMA_ENABLE(&hdma_usart3_rx);

    if (error_flags != 0U)
    {
        dt7_rx_uart_error_count++;
        return;
    }
    if (received_length != RC_FRAME_LENGTH)
    {
        dt7_rx_bad_length_count++;
        return;
    }
    if (solve_dt7_data(sbus_rx_buffer[completed_buffer], &dt7) != 0U)
    {
        dt7_rx_frame_count++;
    }
    else
    {
        dt7_rx_invalid_data_count++;
    }
}

static uint8_t solve_dt7_data(volatile const uint8_t *sbus, RC_ctrl_t *remote)
{
    RC_ctrl_t decoded = {0};
    uint16_t wheel_raw;
    uint8_t i;

    if (sbus == NULL || remote == NULL)
    {
        return 0U;
    }

    decoded.rc.ch[0] = ((sbus[0]) | (sbus[1] << 8)) & 0x07ff;
    decoded.rc.ch[1] = ((sbus[1] >> 3) | (sbus[2] << 5)) & 0x07ff;
    decoded.rc.ch[2] = ((sbus[2] >> 6) | (sbus[3] << 2) | (sbus[4] << 10)) & 0x07ff;
    decoded.rc.ch[3] = ((sbus[4] >> 1) | (sbus[5] << 7)) & 0x07ff;
    decoded.rc.s[0] = (sbus[5] >> 4) & 0x03;
    decoded.rc.s[1] = (sbus[5] >> 6) & 0x03;
    wheel_raw = sbus[16] | (sbus[17] << 8);
    if (wheel_raw > 0x07ffU)
    {
        return 0U;
    }
    
    decoded.rc.ch[4] = wheel_raw;

    for (i = 0; i < 5U; i++)
    {
        decoded.rc.ch[i] -= RC_CH_VALUE_OFFSET;
        if (decoded.rc.ch[i] < -700 || decoded.rc.ch[i] > 700)
        {
            return 0U;
        }
    }
    if (decoded.rc.s[0] < RC_SW_UP || decoded.rc.s[0] > RC_SW_MID ||
        decoded.rc.s[1] < RC_SW_UP || decoded.rc.s[1] > RC_SW_MID ||
        sbus[12] > 1U || sbus[13] > 1U)
    {
        return 0U;
    }

    decoded.mouse.x = sbus[6] | (sbus[7] << 8);
    decoded.mouse.y = sbus[8] | (sbus[9] << 8);
    decoded.mouse.z = sbus[10] | (sbus[11] << 8);
    decoded.mouse.press_l_last = remote->mouse.press_l;
    decoded.mouse.press_r_last = remote->mouse.press_r;
    decoded.mouse.press_l = sbus[12];
    decoded.mouse.press_r = sbus[13];
    decoded.key.v = sbus[14] | (sbus[15] << 8);

    /* Publish only a complete, plausible frame; keep the previous frame on error. */
    *remote = decoded;
    return 1U;
}
