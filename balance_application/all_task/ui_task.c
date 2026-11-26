#include "ui_task.h"
#include "referee_usart_task.h"
#include "main.h"
#include "cmsis_os.h"
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
extern super_cap_t super_cap;

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
static void mini_ui_init(void);
static void mini_ui_flag_updata(void);
static void supercapacitor_figure(fp32 remain_energy,uint32_t turning_point);
static void Float_supercapacitor_update();//超级电容条绘制
void Client_supercapacitor_update();//超级电容框架绘制
ext_graphic_seven_data_t super_data;    //绘制超级电容电压
//变量 
uint8_t update_supercapacitor_flag=1;
uint8_t V_cap_flag=1;
uint8_t CliendTxBuffer[200];//真正发送数组
//结构体
ext_graphic_one_data_t tx_supercapacitor_figure;//绘制一个图形   超级电容条静态
/*用户使用区*/
void Ui_Task(void const * argument)
{
	vTaskDelay(10);
	vTaskDelay(1);
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
		vTaskDelay(1);
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
static void mini_ui_init(void)
{

}

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
	global_supercapacitor_remain = (super_cap.V_cap_value);
}
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
	Figure_Graphic(super_data.clientData,"SR1",1,FLOAT,5,CYAN_BLUE,0,0,5,(680),95,0, (680+600),super_cap.V_cap_value);
    memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&super_data.CmdID, LEN_CMD_ID+super_data.txFrameHeader.data_length);//加上命令码长度2

    //帧尾
    append_CRC16_check_sum(CliendTxBuffer,sizeof(super_data));

    Client_Sent_String(CliendTxBuffer, sizeof(super_data));
}