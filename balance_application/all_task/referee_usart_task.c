/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee_usart_task.c/h
  * @brief      RM referee system data solve. RM裁判系统数据处理
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "referee_usart_task.h"
#include "main.h"
#include "cmsis_os2.h"
#include "bsp_usart.h"
#include "CRC8_CRC16.h"
#include "fifo.h"
#include "protocol.h"
#include "referee.h"
#include "can_user.h"
#include "ins_task.h"
/**
 * @brief   referee_usart_task.c —— RoboMaster 裁判系统数据处理任务
 *
 * 职责：通过 USART6（双缓冲 DMA）接收裁判系统的串口数据帧，用状态机逐字节解包
 *       出完整协议帧，CRC 校验通过后交给 referee_data_solve() 解析成结构体，
 *       并把需要的控制量（电容功率、云台/底盘指令、pitch 斜坡）经 CAN 发给对应节点。
 *
 * 接收链路：
 *   USART6 + DMA 双缓冲接收 -> 空闲中断（IDLE）触发 -> 把已到数据写入环形 FIFO
 *   -> Referee_Usart_Task 每 5ms 从 FIFO 取字节 -> 状态机解包 -> CRC 校验 -> 解析
 *
 * 协议帧格式（RoboMaster 裁判系统串口协议）：
 *   帧头(SOF) + 数据长度(2) + 序列号 + 命令码(2) + 数据 + 帧尾 CRC(2)
 *   先做帧头 CRC8 校验，再做整帧 CRC16 校验，任一失败则丢弃残帧回到“找帧头”。
 */
/**
  * @brief          single byte upacked 
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */
static /**
 * @brief   从环形 FIFO 逐字节解包裁判系统协议帧（状态机）
 * @param   None
 * @note    状态机按帧结构推进：
 *            找帧头 -> 解析数据长度(2) -> 序列号 -> 帧头 CRC8 -> 收数据体 -> 帧尾 CRC16
 *          任一步校验失败即重置回“找帧头”并丢弃残帧；
 *          CRC16 校验通过后调用 referee_data_solve() 把原始帧解析到裁判系统结构体。
 */
void referee_unpack_fifo_data(void);

 
extern UART_HandleTypeDef huart6;
extern void dt7_remote_on_rx_event(uint16_t Size);


uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];
fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;
float limit_to_C=0;
float buffer_to_C=0;

extern fp32 pitch;
	
int16_t send_pitch=0;
/**
  * @brief          referee task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          裁判系统任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
 * @brief   裁判系统处理任务主循环（CMSIS-RTOS2 线程入口）
 * @param   argument  CMSIS-RTOS2 线程参数（本工程未使用，恒为 NULL）
 * @note    初始化裁判数据结构、环形 FIFO 与 USART6 双缓冲 DMA；
 *          启动后先 osDelay(500ms) 等串口稳定，之后每 5ms 循环：
 *          解包 FIFO 数据，并把电容功率/云台底盘指令/pitch 斜坡量经 CAN 下发。
 */
void Referee_Usart_Task(void *argument)
{
    init_referee_struct_data();
    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
    usart6_init(usart6_buf[0], usart6_buf[1], USART_RX_BUF_LENGHT);
	osDelay(500);
    while(1)
    {
        referee_unpack_fifo_data();
		CAN_cmd_cap(&RM_Referee_system);
		CAN_chassis_send_gimbal(&RM_Referee_system);
		CAN_chassis_ramp_pitch_to_gimbal(pitch);
        osDelay(5);
    }
}


/**
  * @brief          single byte upacked 
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */
/**
 * @brief   从环形 FIFO 逐字节解包裁判系统协议帧（状态机）
 * @param   None
 * @note    状态机按帧结构推进：
 *            找帧头 -> 解析数据长度(2) -> 序列号 -> 帧头 CRC8 -> 收数据体 -> 帧尾 CRC16
 *          任一步校验失败即重置回“找帧头”并丢弃残帧；
 *          CRC16 校验通过后调用 referee_data_solve() 把原始帧解析到裁判系统结构体。
 */
void referee_unpack_fifo_data(void)
{
  uint8_t byte = 0;
  uint8_t sof = HEADER_SOF;
  unpack_data_t *p_obj = &referee_unpack_obj;

  while ( fifo_s_used(&referee_fifo) )
  {
    byte = fifo_s_get(&referee_fifo);
    switch(p_obj->unpack_step)
    {
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          p_obj->unpack_step = STEP_LENGTH_LOW;
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else
        {
          p_obj->index = 0;
        }
      }break;
      
      case STEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);
        p_obj->protocol_packet[p_obj->index++] = byte;

        if(p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
        {
          p_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }break;
      case STEP_FRAME_SEQ:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_HEADER_CRC8;
      }break;

      case STEP_HEADER_CRC8:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;

        if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
        {
          if ( verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE) )
          {
            p_obj->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
      }break;  
      
      case STEP_DATA_CRC16:
      {
        if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
           p_obj->protocol_packet[p_obj->index++] = byte;  
        }
        if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;

          if ( verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len) )
          {
            referee_data_solve(p_obj->protocol_packet);
          }
        }
      }break;

      default:
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }break;
    }
  }
}



/**
 * @brief   USART6/USART3 串口 IDLE 接收事件回调(HAL_UARTEx_ReceiveToIdle_DMA 触发)
 * @param   huart  触发回调的串口句柄
 * @param   Size   本次 IDLE 前收到的字节数
 * @note    USART6: 将本次收到的数据写入裁判系统环形 FIFO, 并重新启动下一帧接收;
 *          USART3: 分派给 DT7 遥控接收处理.
 * @warning 本回调为 HAL 全局弱回调, 只能定义一份, 按 Instance 分派到各外设.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance == USART6)
    {
        fifo_s_puts(&referee_fifo, (char*)usart6_buf[0], Size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, usart6_buf[0], USART_RX_BUF_LENGHT);
    }
    else if(huart->Instance == USART3)
    {
        dt7_remote_on_rx_event(Size);
    }
}
