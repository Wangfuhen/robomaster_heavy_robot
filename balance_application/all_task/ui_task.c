#include "ui_task.h"
#include "referee_usart_task.h"
#include "main.h"
#include "cmsis_os2.h"
#include "bsp_usart.h"
#include "CRC8_CRC16.h"
#include "protocol.h"
#include "referee.h"
#include "stm32f4xx.h"
#include "bsp_usart.h"
#include "stm32f4xx_hal.h"
#include "can_user.h"
#include "arm_math.h"
#include "chassis_task.h"
#include "ui_interface.h"
/**
 * @brief   ui_task.c —— 裁判系统 UI（客户端交互图形）上传任务
 *
 * 职责：通过裁判系统串口协议，把超级电容电量等状态绘制成图形（线段/矩形/字符/浮点）
 *       上传到客户端界面显示。本质是往 USART6 发一串符合裁判系统“交互数据”协议的帧。
 *
 * 绘制内容：超级电容电压条（按剩余量分段变色：绿/粉/橙/品红，低电量警示），
 *           以及超级电容数值框（FLOAT 浮点显示 voltage_volt，单位 V）。
 *
 * 任务节奏（Ui_Task，每周期 1ms）：
 *   - i%100==0  ：每 100ms 刷新一次“框架”图（RECTANGLE 静态框）；
 *   - i%20==0   ：每 20ms 刷新一次“数值条”图（LINE 动态条）；
 *   - 其余时间只做标志更新，避免占用过多串口带宽。
 *
 * 关键协议点：
 *   - 帧 = 帧头(含长度) + CRC8 + 命令码 + 数据段头 + 图形数据 + CRC16；
 *   - 发送 ID = 本机 robot_id，接收 ID 由 mini_ui_flag_updata 按 robot_id 查表映射。
 */
extern super_capacitor_measurement_t super_capacitor_measurement;

/*功能函数区*/
//画字符
void Char_Graphic(ext_client_string_t* graphic,//发送char型字符 ，要发送的数据段内容
                  const char* name,       //图像名称
                  uint32_t operate_tpye,  //图层操作
                  uint32_t layer,         //在哪个图层
                  uint32_t color,         //绘制内容颜色
                  uint32_t size,          //尺寸 粗细
                  uint32_t length,        //长度
                  uint32_t width,         //宽度
                  uint32_t start_x,       //起始坐标x
                  uint32_t start_y,       //终止坐标y
                  const char *character);//外部放入字符数组内容
				  
//画图像
void Figure_Graphic(graphic_data_struct_t* graphic,//最终要发出去的数组的数据段内容
                    const char* name,
                    uint32_t operate_tpye,
                    uint32_t graphic_tpye,//绘制什么图像
                    uint32_t layer,
                    uint32_t color,
                    uint32_t start_angle,
                    uint32_t end_angle,
                    uint32_t width,
                    uint32_t start_x,
                    uint32_t start_y,
                    uint32_t radius,
                    uint32_t end_x,
                    uint32_t end_y); 
					
void Float_Graphic(Float_data_struct_t* graphic,//最终要发出去的数组中的数据段内容
                   const char* name,//图像名称
                   uint32_t operate_tpye,//图层操作
                   uint32_t graphic_tpye,//绘制什么图像
                   uint32_t layer,//图层选择
                   uint32_t color,//颜色选择
                   uint32_t start_angle,//开始角度
                   uint32_t end_angle,//结束角度
                   uint32_t width,//宽度
                   uint32_t start_x,//起始位置x坐标
                   uint32_t start_y,//起始位置y坐标
                   float number);
void Client_Sent_String(uint8_t *string, uint16_t length);
/*功能函数截至区*/

/*中间功能数据*/
				   
uint16_t Recieve_id;	   
fp32 global_supercapacitor_remain = 0;//[0,100]
uint32_t global_supercapacitor_point = 50;
/*用户使用区*/
/**
 * @brief   mini_ui 初始化（预留接口，当前为空实现）
 * @param   None
 */
static void mini_ui_init(void);
/**
 * @brief   更新 UI 需要的映射与状态（运行在任务上下文）
 * @note    按裁判系统 robot_id 查表得到“接收端 ID”Recieve_id（用于把图形发给对应机器人），
 *          并同步 ui_self_id 与全局超级电容剩余量 global_supercapacitor_remain（单位 V）。
 */
static void mini_ui_flag_updata(void);
/**
 * @brief   绘制超级电容电量条（LINE 线段，按剩余量分段变色）
 * @param   remain_energy  超级电容电压，单位 V（经 25.2V 满量程折算成 0..100 比例）
 * @param   turning_point  低电量警示阈值（百分比）
 * @note    剩余越高越绿，降到不同阈值区间依次变为 粉/橙/品红，提示电量不足；
 *          update_supercapacitor_flag==2 时画动态线段，==1 时画静态矩形框。
 */
static void supercapacitor_figure(fp32 remain_energy,uint32_t turning_point);
static /**
 * @brief   组装并发送“超级电容数值”交互图形帧（FLOAT 浮点显示电压）
 * @note    用 FLOAT 图形类型把 super_capacitor_measurement.voltage_volt 数值直接绘制在客户端界面。
 */
void Float_supercapacitor_update();//超级电容条绘制
/**
 * @brief   组装并发送“超级电容电量条”交互图形帧
 * @note    依次填充：帧头 -> CRC8 -> 命令码 -> 数据段头(send/receiver id) ->
 *          图形数据(supercapacitor_figure) -> CRC16，最后经 USART6 发出。
 */
void Client_supercapacitor_update();//超级电容框架绘制
ext_graphic_seven_data_t super_data;    //绘制超级电容电压
//变量 
uint8_t update_supercapacitor_flag=1;
uint8_t V_cap_flag=1;
uint8_t CliendTxBuffer[200];//真正发送数组
//结构体
ext_graphic_one_data_t tx_supercapacitor_figure;//绘制一个图形   超级电容条静态
/*用户使用区*/
/**
 * @brief   UI 上传任务主循环（CMSIS-RTOS2 线程入口）
 * @param   argument  CMSIS-RTOS2 线程参数（未使用）
 * @note    以 i 为节拍计数：每 100ms 刷新一次静态框架（ui_init_first_Ungroup + 电容框），
 *          每 20ms 刷新一次动态数值条；中间仅做标志更新，控制串口带宽。
 */
void Ui_Task(void *argument)
{
	osDelay(10);
	osDelay(1);
	while(1)
	{
		static uint32_t i=0;
		if(i%100==0) //静态
		{
			ui_init_first_Ungroup() ;	
			update_supercapacitor_flag=1;	
		    Client_supercapacitor_update();
			V_cap_flag=2;
		}
		if(i%100==3)
		{
			
		}
		mini_ui_flag_updata();
		if(i%20==0)
		{			
			update_supercapacitor_flag=2;
		    Client_supercapacitor_update();

			V_cap_flag=1;
		}
		osDelay(1);
		i++;
		
		if(i>1001)
		{
			i=0;
		}
	}
}


/*功能函数定义区*/
void Char_Graphic(ext_client_string_t* graphic,//发送char型字符 ，要发送的数据段内容
                  const char* name,       //图像名称
                  uint32_t operate_tpye,  //图层操作
                  uint32_t layer,         //在哪个图层
                  uint32_t color,         //绘制内容颜色
                  uint32_t size,          //尺寸 粗细
                  uint32_t length,        //长度
                  uint32_t width,         //宽度
                  uint32_t start_x,       //起始坐标x
                  uint32_t start_y,       //终止坐标y
				  const char *character)//外部放入的数组，内容是要显示的内容
 {
    graphic_data_struct_t *data_struct = &graphic->grapic_data_struct;
    for(char i=0; i<3; i++)
        data_struct->graphic_name[i] = name[i];	//字符索引
    data_struct->operate_tpye = operate_tpye; //图层操作
    data_struct->graphic_tpye = CHAR;         //Char型
    data_struct->layer = layer;//都在第零层
    data_struct->color = color;//都是白色
    data_struct->start_angle = size;
    data_struct->end_angle = length;
    data_struct->width = width;
    data_struct->start_x = start_x;
    data_struct->start_y = start_y;

    data_struct->radius = 0;
    data_struct->end_x = 0;
    data_struct->end_y = 0;

    memcpy(graphic->data,character,length);
}
 
void Figure_Graphic(graphic_data_struct_t* graphic,//最终要发出去的数组的数据段内容
                    const char* name,
                    uint32_t operate_tpye,
                    uint32_t graphic_tpye,//绘制什么图像
                    uint32_t layer,
                    uint32_t color,
                    uint32_t start_angle,
                    uint32_t end_angle,
                    uint32_t width,
                    uint32_t start_x,
                    uint32_t start_y,
                    uint32_t radius,
                    uint32_t end_x,
                    uint32_t end_y)
{
	for(char i=0; i<3; i++)
    graphic->graphic_name[i] = name[i];	//字符索引
    graphic->operate_tpye = operate_tpye; //图层操作
    graphic->graphic_tpye = graphic_tpye;         //Char型
    graphic->layer        = layer;//都在第一层
    graphic->color        = color;//变色
    graphic->start_angle  = start_angle;
    graphic->end_angle    = end_angle;
    graphic->width        = width;
    graphic->start_x      = start_x;
    graphic->start_y      = start_y;
    graphic->radius = radius;
    graphic->end_x  = end_x;
    graphic->end_y  = end_y;
}

void Float_Graphic(Float_data_struct_t* graphic,//最终要发出去的数组中的数据段内容
                   const char* name,//图像名称
                   uint32_t operate_tpye,//图层操作
                   uint32_t graphic_tpye,//绘制什么图像
                   uint32_t layer,//图层选择
                   uint32_t color,//颜色选择
                   uint32_t start_angle,//开始角度
                   uint32_t end_angle,//结束角度
                   uint32_t width,//宽度
                   uint32_t start_x,//起始位置x坐标
                   uint32_t start_y,//起始位置y坐标
                   float number)
{
    for(char i=0; i<3; i++)
    graphic->graphic_name[i] = name[i];	//字符索引
    graphic->operate_tpye = operate_tpye; //图层操作
    graphic->graphic_tpye = FLOAT;         //Float型
    graphic->layer        = layer;//都在第一层
    graphic->color        = color;//变色
    graphic->start_angle  = start_angle;
    graphic->end_angle    = end_angle;
    graphic->width        = width;
    graphic->start_x      = start_x;
    graphic->start_y      = start_y;
    graphic->number       =number;
}

void Client_Sent_String(uint8_t *string, uint16_t length)
{
    HAL_UART_Transmit(&huart6,string,length,100);
}

/*用户使用区*/
/**
 * @brief   mini_ui 初始化（预留接口，当前为空实现）
 * @param   None
 */
static void mini_ui_init(void)
{

}

/**
 * @brief   更新 UI 需要的映射与状态（运行在任务上下文）
 * @note    按裁判系统 robot_id 查表得到“接收端 ID”Recieve_id（用于把图形发给对应机器人），
 *          并同步 ui_self_id 与全局超级电容剩余量 global_supercapacitor_remain（单位 V）。
 */
static void mini_ui_flag_updata(void)
{
    if(RM_Referee_system.robot_state.robot_id==3)
    {
        Recieve_id = 0x103;
    }
    else if(RM_Referee_system.robot_state.robot_id==4)
    {
        Recieve_id = 0x104;
    }
    else if(RM_Referee_system.robot_state.robot_id==103)
    {
        Recieve_id = 0x167;
    }
    else if(RM_Referee_system.robot_state.robot_id==104)
    {
        Recieve_id = 0x168;
    }
	ui_self_id=RM_Referee_system.robot_state.robot_id;
	global_supercapacitor_remain = (super_capacitor_measurement.voltage_volt);
}
/**
 * @brief   绘制超级电容电量条（LINE 线段，按剩余量分段变色）
 * @param   remain_energy  超级电容电压，单位 V（经 25.2V 满量程折算成 0..100 比例）
 * @param   turning_point  低电量警示阈值（百分比）
 * @note    剩余越高越绿，降到不同阈值区间依次变为 粉/橙/品红，提示电量不足；
 *          update_supercapacitor_flag==2 时画动态线段，==1 时画静态矩形框。
 */
static void supercapacitor_figure(fp32 remain_energy,uint32_t turning_point)//剩余超级电容（单位百分比），低于某百分比变红色
{
	 uint32_t remaining = 0,remaining1=0;//强制转换
    remaining = (uint32_t)(remain_energy*100)/25.2;
    remaining1 = (uint32_t)(remain_energy*600)/25.2;
	if(update_supercapacitor_flag==2)
	{
    if(remaining >= turning_point+25)//直线长度为3
        Figure_Graphic(&tx_supercapacitor_figure.clientData,"SR1",update_supercapacitor_flag,LINE,5,GREEN,  0,0,30,(640),30,0,(640+remaining1),30);
	else if(remaining < turning_point+25 && remaining>=turning_point)
	{
		Figure_Graphic(&tx_supercapacitor_figure.clientData,"SR1",update_supercapacitor_flag,LINE,5,PINK,  0,0,30,(640),30,0,  (640+remaining1),30);
	}
	else if(remaining < turning_point && remaining>=turning_point-15)
	{
		Figure_Graphic(&tx_supercapacitor_figure.clientData,"SR1",update_supercapacitor_flag,LINE,5,ORANGE,  0,0,30,(640),30,0, (640+remaining1),30);
	}
    else if(remaining < turning_point-15)
	{
        Figure_Graphic(&tx_supercapacitor_figure.clientData,"SR1",update_supercapacitor_flag,LINE,5,FUCHSIA,0,0,30,(640),30,0,  (640+remaining1),30);
	}
    }
	else if(update_supercapacitor_flag==1)
	Figure_Graphic(&tx_supercapacitor_figure.clientData,"SR1",update_supercapacitor_flag,RECTANGLE,5,CYAN_BLUE,0,0,5,(920-280),95,0, (920+280),125);
}

/**
 * @brief   组装并发送“超级电容电量条”交互图形帧
 * @note    依次填充：帧头 -> CRC8 -> 命令码 -> 数据段头(send/receiver id) ->
 *          图形数据(supercapacitor_figure) -> CRC16，最后经 USART6 发出。
 */
void Client_supercapacitor_update()//超级电容条绘制
{
    //帧头
    tx_supercapacitor_figure.txFrameHeader.sof = JUDGE_FRAME_HEADER;
    tx_supercapacitor_figure.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t);
    tx_supercapacitor_figure.txFrameHeader.seq = 0;//包序号
    memcpy(CliendTxBuffer,&tx_supercapacitor_figure.txFrameHeader,sizeof(std_frame_header_t));
    append_CRC8_check_sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验
    //命令码
    tx_supercapacitor_figure.CmdID = ID_robot_interactive_header_data;
    //数据段头结构
    tx_supercapacitor_figure.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_five_graphic;
    tx_supercapacitor_figure.dataFrameHeader.send_ID     =  RM_Referee_system.robot_state.robot_id;
    tx_supercapacitor_figure.dataFrameHeader.receiver_ID = Recieve_id;
    //数据段
    supercapacitor_figure(global_supercapacitor_remain,global_supercapacitor_point);
    memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_supercapacitor_figure.CmdID, LEN_CMD_ID+tx_supercapacitor_figure.txFrameHeader.data_length);//加上命令码长度2

    //帧尾
    append_CRC16_check_sum(CliendTxBuffer,sizeof(tx_supercapacitor_figure));

    Client_Sent_String(CliendTxBuffer, sizeof(tx_supercapacitor_figure));
}

/**
 * @brief   组装并发送“超级电容数值”交互图形帧（FLOAT 浮点显示电压）
 * @note    用 FLOAT 图形类型把 super_capacitor_measurement.voltage_volt 数值直接绘制在客户端界面。
 */
void Float_supercapacitor_update()//超级电容框绘制
{
    //帧头
    super_data.txFrameHeader.sof = JUDGE_FRAME_HEADER;
    super_data.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t);
    super_data.txFrameHeader.seq = 0;//包序号
    memcpy(CliendTxBuffer,&super_data.txFrameHeader,sizeof(std_frame_header_t));
    append_CRC8_check_sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验
    //命令码
    super_data.CmdID = ID_robot_interactive_header_data;
    //数据段头结构
    super_data.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_one_graphic;
    super_data.dataFrameHeader.send_ID     =  RM_Referee_system.robot_state.robot_id;
    super_data.dataFrameHeader.receiver_ID = Recieve_id;
    //数据段
	Figure_Graphic(super_data.clientData,"SR1",1,FLOAT,5,CYAN_BLUE,0,0,5,(680),95,0, (680+600),super_capacitor_measurement.voltage_volt);
    memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&super_data.CmdID, LEN_CMD_ID+super_data.txFrameHeader.data_length);//加上命令码长度2

    //帧尾
    append_CRC16_check_sum(CliendTxBuffer,sizeof(super_data));

    Client_Sent_String(CliendTxBuffer, sizeof(super_data));
}