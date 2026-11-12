#include "chassis_task.h"
#include "main.h"
#include "can_user.h"
#include "cmsis_os2.h"
#include "pid.h"
#include "arm_math.h"
#include "ins_task.h"
#include "math.h"
#include <stdlib.h>
/**
 * @brief   chassis_task.c —— 麦轮（Mecanum）底盘控制任务
 *
 * 职责：周期性读取遥控器/键盘/云台指令，解析出底盘模式，解算四个麦轮电机的
 *       目标速度并交给 PID 速度环，再输出电流指令，最后经 CAN 发给四个底盘电机。
 *
 * 控制链路（每周期 1ms，由 Chassis_Task 的 osDelay(1) 决定）：
 *   模式解析 -> 反馈更新 -> 目标速度解算 -> PID 速度环 -> 功率限制 -> CAN 输出
 *
 * 底盘模式（chassis_mode）：
 *   CHASSIS_ZERO_FORCE       : 零力矩（完全无输出，电机自由）
 *   CHASSIS_NO_MOVE          : 原地不动（目标速度置 0）
 *   CHASSIS_NO_FOLLOW_GIMBAL : 不随云台（平移，不自旋）
 *   CHASSIS_FOLLOW_GIMBAL    : 云台跟随（平移 + 跟随云台 yaw 自旋）
 *
 * 速度约定（单位统一）：
 *   - 平移速度 vx / vy        : 底盘坐标系，单位 rpm（麦轮转速折算，见 MOTOR_SPEED_TO_CHASSIS_SPEED_*）
 *   - 自旋角速度 wz           : 单位 rpm（已按电机编码折算）
 *   - 电流 give_current       : 单位 A（安培）
 *   - 时间                   : tickCountMs 由 osKernelGetTickCount() 获取，单位 ms
 *
 * 控制说明：
 *   - 遥控器通道经死区（Remote_Deadband_Limit）与灵敏度（RC_X/Y_SENSE）映射为速度；
 *   - 键盘模式按裁判系统功率上限（chassis_power_limit）查表得到最大速度；
 *   - 云台跟随模式下用 PID（chassis_follow_gimbal_pid）把底盘 yaw 对齐云台 yaw；
 *   - 功率限制用“二次方程反解电流”（chassis_power_limit）在功率预算内限制电流。
 */
#define Remote_Deadband_Limit(mag_input,msg_output,dead_control) \
{                                                                \
    if(mag_input>dead_control||mag_input<-dead_control)          \
    {                                                            \
        msg_output=mag_input;                                    \
    }                                                            \
    else                                                         \
    {                                                            \
        msg_output=0;                                            \
    }                                                            \
}  //遥控器死区控制
#define Limit_Max(input,max) \
{                           \
    if(input>max)           \
    {                       \
        input=max;          \
    }                       \
    else if(input<-max)     \
    {                       \
        input=-max;         \
    }                       \
}  //最大区间控制
#define Abs(input) ((input) < 0 ? -(input) : (input))
#define Sqrt(input)         \
{                           \
    input=input*input;      \
}  //开平方

/**
 * @brief   底盘控制结构体初始化（仅在任务启动时调用一次）
 * @param   mini_chassis_init  底盘控制结构体指针
 * @note    完成：默认模式（零力矩）、速度/功率清零、四路电机反馈指针绑定、
 *          PID 参数初始化、云台/INS/遥控器/裁判系统/超电数据指针绑定、
 *          三个轴的低通滤波（First_Lpf）与斜坡规划（slope_init）初始化。
 *          所有指针均来自各模块的 get_*_point() 观测点（共享内存/外设数据）。
 */
static void chassis_init(chassis_control_t *mini_chassis_init);                                               //底盘初始化
/**
 * @brief   从云台 CAN 数据解析本次控制模式（底盘/超电/陀螺/掉头）
 * @param   mini_chassis_set_mode  底盘控制结构体指针
 * @note    把来自云台（chassis_gimbal_can_data）的 mode/cap_mode/top_mode/follow_mode
 *          整数编码翻译为对应的底盘枚举模式，供后续控制逻辑分支使用。
 */
static void chassis_parse_mode(chassis_control_t *mini_chassis_set_mode);                                       //底盘模式设置
/**
 * @brief   检测模式是否发生切换，并记录上一次模式
 * @param   mini_chassis_mode_change_feedback_change  底盘控制结构体指针
 * @note    仅当 chassis_mode != chassis_mode_last 时更新 last；可用于检测模式跳变沿。
 */
static void chassis_detect_mode_change(chassis_control_t *mini_chassis_mode_change_feedback_change); //底盘模式切换反馈
/**
 * @brief   更新底盘反馈量（电机速度/电流、功率、当前底盘速度、云台相对角）
 * @param   mini_chassis_feedback_updata  底盘控制结构体指针
 * @note    逐电机：读取 CAN 编码器反馈 speed_rpm 与 given_current；
 *          由四个轮速按麦轮运动学逆解得到底盘当前 vx_now / vy_now / wz_now；
 *          用 K0..K5 拟合电机功率 P = f(I,ω)（电流 A、角速度 rad/s）；
 *          更新云台相对角 relative_angle（随/不随云台模式取不同零点）。
 */
static void chassis_update_feedback(chassis_control_t *mini_chassis_feedback_updata);                         //底盘反馈更新
/**
 * @brief   根据底盘模式决定目标速度来源
 * @param   mini_chassis_mode_set_control  底盘控制结构体指针
 * @note    CHASSIS_NO_MOVE：目标速度强制为 0；
 *          NO_FOLLOW / FOLLOW：调用 chassis_calc_target_speed 由遥控器/键盘解算；
 *          ZERO_FORCE：无需设置目标（控制阶段直接清零输出）。
 */
static void chassis_set_target_speed(chassis_control_t *mini_chassis_mode_set_control);                       //底盘模式切换控制
/**
 * @brief   底盘速度环 PID 控制计算（含功率限制与零力矩处理）
 * @param   mini_chassis_control_cacl  底盘控制结构体指针
 * @note    1) 根据四轮平均速度动态调整 PID 的 ki 与积分限幅 max_iout（变增益抗饱和）；
 *          2) 对每轮做速度 PID：输入 = 目标转速 speed_set（叠加云台跟随前馈项），
 *             反馈 = 实际转速 speed_rpm，输出 = 给电机电流 give_current（单位 A）；
 *          3) 由解算电流预估实际功率，做功率限制（chassis_power_limit）；
 *          4) ZERO_FORCE 模式下最终把所有电机输出电流清零。
 */
static void chassis_run_speed_pid(chassis_control_t *mini_chassis_control_cacl);                               //底盘模式控制计算
/*子函数*/
/**
 * @brief   由遥控器/键盘原始输入解算底盘目标速度（含跟随、斜坡、滤波）
 * @param   mini_chassis_action_to_control_cacl  底盘控制结构体指针
 * @note    流程：
 *          1. 按 key_control 选择键盘或遥控器作为 vx/vy 平移来源；
 *          2. 若陀螺开启（TOP_ON）按方向设定自旋 wz；陀螺关闭且跟随云台时用
 *             跟随 PID 把 wz 对齐云台（含掉头切换的等待/平滑处理）；
 *          3. 得到 vx_set/vy_set/wz_set（速度单位：rpm）；
 *          4. 云台跟随模式下把平移速度旋转到云台系（坐标系旋转变换）；
 *          5. wz 经一阶低通（First_Lpf_Calc），vx/vy 经斜坡规划（slope_plan）限速；
 *          6. 最后反解四个麦轮目标转速（chassis_vector_to_wheel_speed）。
 */
static void chassis_calc_target_speed(chassis_control_t *mini_chassis_action_to_control_cacl);           //行为控制量计算
/**
 * @brief   遥控器平移控制：把摇杆通道映射为 vx/vy 目标速度
 * @param   mini_chassis_rc_to_control_cacl  底盘控制结构体指针
 * @note    vx = x_rc_data * RC_X_SENSE，vy = y_rc_data * RC_Y_SENSE
 *          （RC_X_SENSE / RC_Y_SENSE 为灵敏度标定系数，输出量纲为 rpm）。
 */
static void chassis_rc_to_speed_xy(chassis_control_t *mini_chassis_rc_to_control_cacl);                     //遥控器控制XY计算
/**
 * @brief   键盘平移控制：按裁判系统功率上限查表得到最大速度，再按按键给出 vx/vy
 * @param   mini_chassis_keyboard_to_control_cacl  底盘控制结构体指针
 * @note    键盘仅在比赛（key_control==1）模式启用；WASD 映射前后/左右平移，
 *          开启超电（CAP_ON）时最大速度提升到 9000rpm。
 */
static void chassis_keyboard_to_speed_xy(chassis_control_t *mini_chassis_keyboard_to_control_cacl);         //键盘控制XY计算
/**
 * @brief   底盘功率限制：在功率预算内反解每轮允许的最大电流
 * @param   hero_chassis_power_speed_control  底盘控制结构体指针
 * @note    核心思路：电机功率 P = K0+K1·I+K2·ω+K3·ω·I+K4·I2+K5·ω2（拟合式），
 *          其中 I 为待求电流（A）、ω 为当前角速度（rad/s）。给定“本轮可用功率
 *          上限”，即解一元二次方程 a·I2+b·I+c=0 求 I，再换算回给电机的
 *          give_current = I / MA_A（MA_A 为电流比例系数）。
 *          可用功率 = 裁判系统功率上限 + 超电增益 + 缓冲能量 PID（缓冲闭环）；
 *          若实际需求功率 <= 上限则不加限制。
 */
static void chassis_power_limit(chassis_control_t *hero_chassis_power_speed_control);                 //功率速度限制控制
/**
 * @brief   更新功率缓冲能量的可用上限档位
 * @param   chassis_power_feeback_updata  底盘控制结构体指针
 * @note    根据裁判系统 buffer_energy：>70 时按“全增益”档给缓冲功率，
 *          <=60 时按“无增益”档；60~70 之间保持上一档（滞回，防抖）。
 */
static void chassis_update_power_buffer(chassis_control_t *chassis_power_feeback_updata);                    //功率反馈更新
/**
 * @brief   麦轮运动学逆解：把底盘 vx/vy/wz 目标反解为四个麦轮的目标转速
 * @param   chassis_vector_to_mecaum_speed  底盘控制结构体指针
 * @note    麦轮布置：四个轮呈 X 型，正负号组合体现各轮对平移/自旋的贡献；
 *          wz 需乘电机到底盘中心的距离系数 MOTOR_DISTANCE_TO_CENTER 统一量纲。
 *          轮0 = -vx+vy+wz；轮1 = +vx+vy+wz；轮2 = +vx-vy+wz；轮3 = -vx-vy+wz。
 */
static void chassis_vector_to_wheel_speed(chassis_control_t *chassis_vector_to_mecaum_speed);                //底盘向量转麦轮各电机速度
/**
 * @brief   电机编码器原始值 -> 相对角度（弧度）
 * @param   ecd                  当前编码器原始值（0..ECD_RANGE）
 * @param   ecd_zero_angle_set   零位编码值（安装零点）
 * @return  相对零位的角度，单位 rad（弧度）
 * @note    处理编码器回绕：差值超过半圈则加减一整圈，保证角度连续无跳变。
 */
static fp32 motor_encoder_to_angle(uint16_t ecd,uint16_t ecd_zero_angle_set);                                     //电机角度转换
fp32 sin_yaw =0.0f;
fp32 cos_yaw =0.0f; 
chassis_control_t hero_chassis;
/**
 * @brief   底盘控制任务主循环（CMSIS-RTOS2 线程入口）
 * @param   argument  CMSIS-RTOS2 线程参数（本工程未使用，恒为 NULL）
 * @note    周期 = 1ms（while 内 osDelay(1)）。
 *          每周期执行：模式解析 -> 反馈更新 -> 目标解算 -> 控制计算 -> CAN 发送。
 *          tickCountMs 记录本次周期系统 tick，供时间相关逻辑使用（单位 ms）。
 */
void Chassis_Task(void *argument)
{   osDelay(MINI_CHASSIS_INIT_TIME);
    uint32_t tickCountMs;
    chassis_init(&hero_chassis);
	osDelay(MINI_CHASSIS_INIT_TIME);
    while (1) 
    {   
       tickCountMs = osKernelGetTickCount();
        chassis_parse_mode(&hero_chassis);
       chassis_detect_mode_change(&hero_chassis);
       chassis_update_feedback(&hero_chassis);
        chassis_set_target_speed(&hero_chassis);
       chassis_run_speed_pid(&hero_chassis);
    	CAN_cmd_chassis(hero_chassis.chassis_motor[0].give_current,hero_chassis.chassis_motor[1].give_current,hero_chassis.chassis_motor[2].give_current,hero_chassis.chassis_motor[3].give_current);
		
//    #if INCLUDE_uxTaskGetStackHighWaterMark
//        chassis_high_water = uxTaskGetStackHighWaterMark(NULL); //获取任务栈使用情况
//    #endif
    osDelay(1);
	}
}

/**
 * @brief   底盘控制结构体初始化（仅在任务启动时调用一次）
 * @param   mini_chassis_init  底盘控制结构体指针
 * @note    完成：默认模式（零力矩）、速度/功率清零、四路电机反馈指针绑定、
 *          PID 参数初始化、云台/INS/遥控器/裁判系统/超电数据指针绑定、
 *          三个轴的低通滤波（First_Lpf）与斜坡规划（slope_init）初始化。
 *          所有指针均来自各模块的 get_*_point() 观测点（共享内存/外设数据）。
 */
static void chassis_init(chassis_control_t *mini_chassis_init)
{
    if(mini_chassis_init == NULL)
    {
        return;
    }
    mini_chassis_init->chassis_mode = CHASSIS_ZERO_FORCE;
    mini_chassis_init->chassis_cap_mode = CAP_OFF;
    mini_chassis_init->chassis_top_mode = TOP_OFF;
    mini_chassis_init->vx_now = 0;
    mini_chassis_init->vy_now = 0;
    mini_chassis_init->wz_now = 0;
    mini_chassis_init->vx_set = 0;
    mini_chassis_init->vy_set = 0;
    mini_chassis_init->wz_set = 0;
    mini_chassis_init->vx_normal_speed=0;
    mini_chassis_init->vy_normal_speed=0;
    mini_chassis_init->wz_normal_speed=0;
    mini_chassis_init->vx_add_speed=0;
    mini_chassis_init->vy_add_speed=0;
    mini_chassis_init->wz_add_speed=0;
    mini_chassis_init->chassis_power = 0;
    mini_chassis_init->chassis_power_user_can = 0;
    mini_chassis_init->chassis_power_max = 0;
    mini_chassis_init->chassis_power = 0;
	mini_chassis_init->chassis_follow.gimbal_change = CHASSIS_FOLLOW_GIMBAL_NORMAL;
	mini_chassis_init->chassis_follow.gimbal_change_chassis_time=0;
	mini_chassis_init->chassis_follow.gimbal_change_chassis_flag=0;
	mini_chassis_init->chassis_x_slope_accl=SLOPE_CHASSIS_X_ACCELERATION;
	mini_chassis_init->chassis_x_slope_rate=SLOPE_CHASSIS_X_DECELERATION;
	mini_chassis_init->chassis_y_slope_accl=SLOPE_CHASSIS_Y_ACCELERATION;
	mini_chassis_init->chassis_y_slope_rate=SLOPE_CHASSIS_Y_DECELERATION;
	mini_chassis_init->chassis_w_slope_accl=SLOPE_CHASSIS_W_ACCELERATION;
	mini_chassis_init->chassis_w_slope_rate=SLOPE_CHASSIS_W_DECELERATION;
    const fp32 chassis_motor_speed_pid[3]={CHASSIS_SPEED_PID_P,CHASSIS_SPEED_PID_I,CHASSIS_SPEED_PID_D};
    const fp32 chassis_follow_gimbal_pid[3]={CHASSIS_FOLLOW_GIMBAL_PID_P,CHASSIS_FOLLOW_GIMBAL_PID_I,CHASSIS_FOLLOW_GIMBAL_PID_D};
	const fp32 chassis_buffer_pid[3]={BUFFER_PID_P,BUFFER_PID_I,BUFFER_PID_D};
    /*电机指针初始化*/
    for(int i=0;i<4;i++)
    {
       mini_chassis_init->chassis_motor[i].chassis_motor_feedback = get_motor_chassis_motor_measure_point(i);
    }
	mini_chassis_init->gimbal_yaw_motor=get_motor_gimbal_yaw_measure_point();
    /*云台数据指针初始化*/
    mini_chassis_init->chassis_gimbal_can_data = get_c_gimbal_data_point();
    /*PID初始化*/
    for(int i=0;i<4;i++)
    {
         pid_init(&mini_chassis_init->chassis_speed_pid[i],PID_DELTA,chassis_motor_speed_pid,CHASSIS_SPEED_PID_MAX_OUT,CHASSIS_SPEED_PID_MAX_I_OUT);
    }
    pid_init(&mini_chassis_init->chassis_follow_gimbal_pid,PID_POSITION,chassis_follow_gimbal_pid,CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT,CHASSIS_FOLLOW_GIMBAL_PID_MAX_I_OUT);
    pid_init(&mini_chassis_init->rm_buffer_control_pid,PID_POSITION,chassis_buffer_pid,BUFFER_PID_MAX_OUT,BUFFER_PID_MAX_I_OUT);
    /*INS初始化*/
		mini_chassis_init->chassis_ins_point=get_gyro_data_point();
    /*遥控器指针初始化*/
    mini_chassis_init->chassis_dt7_remote_control = get_dt7_remote_control_point();
    /*裁判系统指针初始化*/ 
    mini_chassis_init->chassis_rm_data=get_Referee_point();
    /*超电数据指针初始化*/
    mini_chassis_init->super_cap=get_super_cap_measure_point();
		/*低通输入初始化*/
		First_Lpf_Init(&mini_chassis_init->chassis_cmd_vx_lpf,LPF_CHASSIS_X,0.001);
		First_Lpf_Init(&mini_chassis_init->chassis_cmd_vy_lpf,LPF_CHASSIS_Y,0.001);
		First_Lpf_Init(&mini_chassis_init->chassis_cmd_wz_lpf,LPF_CHASSIS_Z,0.001);	
		slope_init(&mini_chassis_init->chassis_vx,mini_chassis_init->chassis_x_slope_accl,mini_chassis_init->chassis_x_slope_rate,&mini_chassis_init->vx_now,&mini_chassis_init->vx_set);
		slope_init(&mini_chassis_init->chassis_vy,mini_chassis_init->chassis_y_slope_accl,mini_chassis_init->chassis_y_slope_rate,&mini_chassis_init->vy_now,&mini_chassis_init->vy_set);
		slope_init(&mini_chassis_init->chassis_w,mini_chassis_init->chassis_w_slope_accl,mini_chassis_init->chassis_w_slope_rate,&mini_chassis_init->wz_now,&mini_chassis_init->wz_set);
    chassis_update_feedback(mini_chassis_init);
}

/**
 * @brief   从云台 CAN 数据解析本次控制模式（底盘/超电/陀螺/掉头）
 * @param   mini_chassis_set_mode  底盘控制结构体指针
 * @note    把来自云台（chassis_gimbal_can_data）的 mode/cap_mode/top_mode/follow_mode
 *          整数编码翻译为对应的底盘枚举模式，供后续控制逻辑分支使用。
 */
static void chassis_parse_mode(chassis_control_t *mini_chassis_set_mode)
{
    if(mini_chassis_set_mode == NULL) 
    {
        return; 
    }
    /*底盘模式*/
        if(mini_chassis_set_mode->chassis_gimbal_can_data->chassis_mode==0)
		{
				mini_chassis_set_mode->chassis_mode=CHASSIS_ZERO_FORCE;
		}
		else if(mini_chassis_set_mode->chassis_gimbal_can_data->chassis_mode==1)
		{
				mini_chassis_set_mode->chassis_mode=CHASSIS_NO_MOVE;
		}
		else if(mini_chassis_set_mode->chassis_gimbal_can_data->chassis_mode==2)
		{
				mini_chassis_set_mode->chassis_mode=CHASSIS_NO_FOLLOW_GIMBAL;
		}
		else if(mini_chassis_set_mode->chassis_gimbal_can_data->chassis_mode==3)
		{
				mini_chassis_set_mode->chassis_mode=CHASSIS_FOLLOW_GIMBAL;
		}
		/*超电模式*/
		if(mini_chassis_set_mode->chassis_gimbal_can_data->cap_mode==0)
		{
				mini_chassis_set_mode->chassis_cap_mode=CAP_OFF;
		}
		else if(mini_chassis_set_mode->chassis_gimbal_can_data->cap_mode==1)
		{
				mini_chassis_set_mode->chassis_cap_mode=CAP_ON;
		}
		/*陀螺模式*/
		if(mini_chassis_set_mode->chassis_gimbal_can_data->top_mode==0)
		{
				mini_chassis_set_mode->chassis_top_mode=TOP_OFF;
		}
		else if(mini_chassis_set_mode->chassis_gimbal_can_data->top_mode)
		{
				mini_chassis_set_mode->chassis_top_mode=TOP_ON;
		}
		/*掉头模式*/
		mini_chassis_set_mode->chassis_follow.gimbal_change_last=mini_chassis_set_mode->chassis_follow.gimbal_change;
		if(mini_chassis_set_mode->chassis_gimbal_can_data->follow_mode==0)
		{
			mini_chassis_set_mode->chassis_follow.gimbal_change=CHASSIS_FOLLOW_GIMBAL_NORMAL;
		}
		else if(mini_chassis_set_mode->chassis_gimbal_can_data->follow_mode==1)
		{
			mini_chassis_set_mode->chassis_follow.gimbal_change=CHASSIS_FOLLOW_GIMBAL_CHANGE;
		}
}

/**
 * @brief   检测模式是否发生切换，并记录上一次模式
 * @param   mini_chassis_mode_change_feedback_change  底盘控制结构体指针
 * @note    仅当 chassis_mode != chassis_mode_last 时更新 last；可用于检测模式跳变沿。
 */
static void chassis_detect_mode_change(chassis_control_t *mini_chassis_mode_change_feedback_change)
{
    if(mini_chassis_mode_change_feedback_change == NULL)
    {
        return;
    }
    if(mini_chassis_mode_change_feedback_change->chassis_mode == mini_chassis_mode_change_feedback_change->chassis_mode_last)
    {
        return; 
    }
    mini_chassis_mode_change_feedback_change->chassis_mode_last = mini_chassis_mode_change_feedback_change->chassis_mode;
}

/**
 * @brief   更新底盘反馈量（电机速度/电流、功率、当前底盘速度、云台相对角）
 * @param   mini_chassis_feedback_updata  底盘控制结构体指针
 * @note    逐电机：读取 CAN 编码器反馈 speed_rpm 与 given_current；
 *          由四个轮速按麦轮运动学逆解得到底盘当前 vx_now / vy_now / wz_now；
 *          用 K0..K5 拟合电机功率 P = f(I,ω)（电流 A、角速度 rad/s）；
 *          更新云台相对角 relative_angle（随/不随云台模式取不同零点）。
 */
static void chassis_update_feedback(chassis_control_t *mini_chassis_feedback_updata)
{
    if(mini_chassis_feedback_updata == NULL)
    {
        return;
    }
	if(mini_chassis_feedback_updata->chassis_follow.gimbal_change==CHASSIS_FOLLOW_GIMBAL_NORMAL)
	{   
		if(mini_chassis_feedback_updata->gimbal_yaw_motor->ecd==0)
		{
			
			mini_chassis_feedback_updata->relative_angle=motor_encoder_to_angle(mini_chassis_feedback_updata->gimbal_yaw_motor->last_ecd,YAW_ZERO_ANGLE_SET);
		}
		mini_chassis_feedback_updata->relative_angle=motor_encoder_to_angle(mini_chassis_feedback_updata->gimbal_yaw_motor->ecd,YAW_ZERO_ANGLE_SET);
	}
	else if(mini_chassis_feedback_updata->chassis_follow.gimbal_change==CHASSIS_FOLLOW_GIMBAL_CHANGE)
	{
	    mini_chassis_feedback_updata->relative_angle=motor_encoder_to_angle(mini_chassis_feedback_updata->gimbal_yaw_motor->ecd,YAW_CHANGE_ANGLE_SET);
	}
    for(int i=0;i<4;i++)
    {	
        mini_chassis_feedback_updata->chassis_motor[i].speed_rpm = mini_chassis_feedback_updata->chassis_motor[i].chassis_motor_feedback->speed_rpm;
        mini_chassis_feedback_updata->chassis_motor[i].give_current = mini_chassis_feedback_updata->chassis_motor[i].chassis_motor_feedback->given_current;
				mini_chassis_feedback_updata->chassis_motor[i].motor_current=mini_chassis_feedback_updata->chassis_motor[i].chassis_motor_feedback->given_current*MA_A;
				mini_chassis_feedback_updata->chassis_motor[i].motor_omega = mini_chassis_feedback_updata->chassis_motor[i].chassis_motor_feedback->speed_rpm*RPM_RADS;
        mini_chassis_feedback_updata->chassis_motor[i].motor_power = (K0+(K1*mini_chassis_feedback_updata->chassis_motor[i].motor_current)+
					(K2*mini_chassis_feedback_updata->chassis_motor[i].motor_omega)+
						(K3*mini_chassis_feedback_updata->chassis_motor[i].motor_omega*mini_chassis_feedback_updata->chassis_motor[i].motor_current)+
							(K4*mini_chassis_feedback_updata->chassis_motor[i].motor_current*mini_chassis_feedback_updata->chassis_motor[i].motor_current)+
								(K5*mini_chassis_feedback_updata->chassis_motor[i].motor_omega*mini_chassis_feedback_updata->chassis_motor[i].motor_omega)); 
        mini_chassis_feedback_updata->vx_now = (-mini_chassis_feedback_updata->chassis_motor[0].speed_rpm + mini_chassis_feedback_updata->chassis_motor[1].speed_rpm + mini_chassis_feedback_updata->chassis_motor[2].speed_rpm - mini_chassis_feedback_updata->chassis_motor[3].speed_rpm)*MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
        mini_chassis_feedback_updata->vy_now = (-mini_chassis_feedback_updata->chassis_motor[0].speed_rpm - mini_chassis_feedback_updata->chassis_motor[1].speed_rpm + mini_chassis_feedback_updata->chassis_motor[2].speed_rpm + mini_chassis_feedback_updata->chassis_motor[3].speed_rpm)*MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
        mini_chassis_feedback_updata->wz_now = (-mini_chassis_feedback_updata->chassis_motor[0].speed_rpm - mini_chassis_feedback_updata->chassis_motor[1].speed_rpm - mini_chassis_feedback_updata->chassis_motor[2].speed_rpm - mini_chassis_feedback_updata->chassis_motor[3].speed_rpm)*MOTOR_SPEED_TO_CHASSIS_SPEED_WZ/MOTOR_DISTANCE_TO_CENTER;
    }
    mini_chassis_feedback_updata->chassis_power=mini_chassis_feedback_updata->chassis_motor[0].motor_power+mini_chassis_feedback_updata->chassis_motor[1].motor_power+mini_chassis_feedback_updata->chassis_motor[2].motor_power+mini_chassis_feedback_updata->chassis_motor[3].motor_power;
    mini_chassis_feedback_updata->chassis_gyro=*(mini_chassis_feedback_updata->chassis_ins_point+INS_GYRO_Z_ADDRESS_OFFSET);
    chassis_update_power_buffer(mini_chassis_feedback_updata);
}

/**
 * @brief   根据底盘模式决定目标速度来源
 * @param   mini_chassis_mode_set_control  底盘控制结构体指针
 * @note    CHASSIS_NO_MOVE：目标速度强制为 0；
 *          NO_FOLLOW / FOLLOW：调用 chassis_calc_target_speed 由遥控器/键盘解算；
 *          ZERO_FORCE：无需设置目标（控制阶段直接清零输出）。
 */
static void chassis_set_target_speed(chassis_control_t *mini_chassis_mode_set_control)
{
    if(mini_chassis_mode_set_control == NULL)
    {
        return;
   
    }

    if(mini_chassis_mode_set_control->chassis_mode == CHASSIS_NO_MOVE)
    {
        mini_chassis_mode_set_control->vx_set = 0;
        mini_chassis_mode_set_control->vy_set = 0;
        mini_chassis_mode_set_control->wz_set = 0;
    }
    else if(mini_chassis_mode_set_control->chassis_mode == CHASSIS_NO_FOLLOW_GIMBAL||mini_chassis_mode_set_control->chassis_mode == CHASSIS_FOLLOW_GIMBAL)
    {
        chassis_calc_target_speed(mini_chassis_mode_set_control);
    }

}

/**
 * @brief   底盘速度环 PID 控制计算（含功率限制与零力矩处理）
 * @param   mini_chassis_control_cacl  底盘控制结构体指针
 * @note    1) 根据四轮平均速度动态调整 PID 的 ki 与积分限幅 max_iout（变增益抗饱和）；
 *          2) 对每轮做速度 PID：输入 = 目标转速 speed_set（叠加云台跟随前馈项），
 *             反馈 = 实际转速 speed_rpm，输出 = 给电机电流 give_current（单位 A）；
 *          3) 由解算电流预估实际功率，做功率限制（chassis_power_limit）；
 *          4) ZERO_FORCE 模式下最终把所有电机输出电流清零。
 */
static void chassis_run_speed_pid(chassis_control_t *mini_chassis_control_cacl)
{
    if(mini_chassis_control_cacl == NULL)
    {
        return;
    }
    fp32 speed_ave=fabs(mini_chassis_control_cacl->chassis_motor[0].speed_rpm+
			mini_chassis_control_cacl->chassis_motor[1].speed_rpm+
				mini_chassis_control_cacl->chassis_motor[2].speed_rpm+
					mini_chassis_control_cacl->chassis_motor[3].speed_rpm)/4;
    for (uint8_t i = 0; i < 4; i++)
    {
        mini_chassis_control_cacl->chassis_speed_pid[i].ki=0.20+1.0*exp(-0.001*fabs(speed_ave)-1.8);
        mini_chassis_control_cacl->chassis_speed_pid[i].max_iout=fmin((440 + 6 * fabs(speed_ave / 100)),1000);
        mini_chassis_control_cacl->chassis_motor[i].give_current=pid_calc(&mini_chassis_control_cacl->chassis_speed_pid[i],
					mini_chassis_control_cacl->chassis_motor[i].speed_rpm,
						mini_chassis_control_cacl->chassis_motor[i].speed_set+mini_chassis_control_cacl->chassis_gyro*CHASSIS_SOLLOW_GIMBAL_FORWARD_KP);
    }
    for(uint8_t i=0;i<4;i++)
    {
			mini_chassis_control_cacl->chassis_motor[i].motor_use_current=mini_chassis_control_cacl->chassis_motor[i].give_current*MA_A;
			mini_chassis_control_cacl->chassis_motor[i].motor_omega = mini_chassis_control_cacl->chassis_motor[i].chassis_motor_feedback->speed_rpm*RPM_RADS;
			mini_chassis_control_cacl->chassis_motor[i].motor_demand_power=(K0+
				(K1*mini_chassis_control_cacl->chassis_motor[i].motor_use_current)+
					(K2*mini_chassis_control_cacl->chassis_motor[i].motor_omega)+
						(K3*mini_chassis_control_cacl->chassis_motor[i].motor_omega*mini_chassis_control_cacl->chassis_motor[i].motor_use_current)+
							(K4*mini_chassis_control_cacl->chassis_motor[i].motor_use_current*mini_chassis_control_cacl->chassis_motor[i].motor_use_current)+
								(K5*mini_chassis_control_cacl->chassis_motor[i].motor_omega*mini_chassis_control_cacl->chassis_motor[i].motor_omega));
		}
	mini_chassis_control_cacl->chassis_power_demand=mini_chassis_control_cacl->chassis_motor[0].motor_demand_power+
		mini_chassis_control_cacl->chassis_motor[1].motor_demand_power+
			mini_chassis_control_cacl->chassis_motor[2].motor_demand_power+
				mini_chassis_control_cacl->chassis_motor[3].motor_demand_power;
	chassis_power_limit(mini_chassis_control_cacl);
	if(mini_chassis_control_cacl->chassis_mode == CHASSIS_ZERO_FORCE)
    {
       for(uint8_t i=0;i<4;i++)
       {
           mini_chassis_control_cacl->chassis_motor[i].give_current = 0;
		   
       }	   
    }
}

/**
 * @brief   由遥控器/键盘原始输入解算底盘目标速度（含跟随、斜坡、滤波）
 * @param   mini_chassis_action_to_control_cacl  底盘控制结构体指针
 * @note    流程：
 *          1. 按 key_control 选择键盘或遥控器作为 vx/vy 平移来源；
 *          2. 若陀螺开启（TOP_ON）按方向设定自旋 wz；陀螺关闭且跟随云台时用
 *             跟随 PID 把 wz 对齐云台（含掉头切换的等待/平滑处理）；
 *          3. 得到 vx_set/vy_set/wz_set（速度单位：rpm）；
 *          4. 云台跟随模式下把平移速度旋转到云台系（坐标系旋转变换）；
 *          5. wz 经一阶低通（First_Lpf_Calc），vx/vy 经斜坡规划（slope_plan）限速；
 *          6. 最后反解四个麦轮目标转速（chassis_vector_to_wheel_speed）。
 */
static void chassis_calc_target_speed(chassis_control_t *mini_chassis_action_to_control_cacl)
{
    fp32 vx=0;
	fp32 vy=0;
    if(mini_chassis_action_to_control_cacl == NULL)
    {
        return;
    }
	 if(mini_chassis_action_to_control_cacl->chassis_gimbal_can_data->key_control==1)//进入比赛模式
     {
	      chassis_keyboard_to_speed_xy(mini_chassis_action_to_control_cacl);  //键盘控制
	 }
	 else
	 {
	 	  chassis_rc_to_speed_xy(mini_chassis_action_to_control_cacl);  //遥控器通道计算
	 }
   
    if(mini_chassis_action_to_control_cacl->chassis_top_mode == TOP_ON)
    {
        if(mini_chassis_action_to_control_cacl->chassis_cap_mode == CAP_ON)
        {
            mini_chassis_action_to_control_cacl->wz_normal_speed=CHASSIS_SPEED_TOP_CAP_ON;
        }
        else if(mini_chassis_action_to_control_cacl->chassis_cap_mode == CAP_OFF)
        {
            mini_chassis_action_to_control_cacl->wz_normal_speed=CHASSIS_SPEED_TOP_CAP_OFF;
        }
        if(mini_chassis_action_to_control_cacl->chassis_top_direction == CHASSIS_TOP_INVE)
        {
            mini_chassis_action_to_control_cacl->wz_normal_speed = mini_chassis_action_to_control_cacl->wz_normal_speed;
        }
        else if(mini_chassis_action_to_control_cacl->chassis_top_direction == CHASSIS_TOP_FWAD)
        {
            mini_chassis_action_to_control_cacl->wz_normal_speed = -mini_chassis_action_to_control_cacl->wz_normal_speed;
        }
    }
    else if(mini_chassis_action_to_control_cacl->chassis_top_mode == TOP_OFF)
    {   
		if(mini_chassis_action_to_control_cacl->chassis_mode==CHASSIS_FOLLOW_GIMBAL)
        {
			if(mini_chassis_action_to_control_cacl->chassis_follow.gimbal_change!=mini_chassis_action_to_control_cacl->chassis_follow.gimbal_change_last)
			{
				mini_chassis_action_to_control_cacl->chassis_follow.gimbal_change_chassis_flag=1;
			}
			if(mini_chassis_action_to_control_cacl->chassis_follow.gimbal_change_chassis_flag==1)
			{   mini_chassis_action_to_control_cacl->wz_normal_speed = 0;
				if(mini_chassis_action_to_control_cacl->chassis_follow.gimbal_change_chassis_time<=YAW_CHANGE_WAIT_TIME/2)
				{
					mini_chassis_action_to_control_cacl->chassis_follow.gimbal_change_chassis_time+=1;
					
				}
				else
				{
					mini_chassis_action_to_control_cacl->chassis_follow.gimbal_change_chassis_flag=0;
					mini_chassis_action_to_control_cacl->chassis_follow.gimbal_change_chassis_time=0;
				}
			}
			else
			{
				mini_chassis_action_to_control_cacl->wz_normal_speed=pid_calc(&mini_chassis_action_to_control_cacl->chassis_follow_gimbal_pid,mini_chassis_action_to_control_cacl->relative_angle,mini_chassis_action_to_control_cacl->set_relative_angle);
			}	
			
        }
        else if(CHASSIS_NO_FOLLOW_GIMBAL)
        {
            mini_chassis_action_to_control_cacl->wz_normal_speed = 0;
        } 
    }
    mini_chassis_action_to_control_cacl->vx_set=mini_chassis_action_to_control_cacl->vx_normal_speed+mini_chassis_action_to_control_cacl->vx_add_speed;
    mini_chassis_action_to_control_cacl->vy_set=mini_chassis_action_to_control_cacl->vy_normal_speed+mini_chassis_action_to_control_cacl->vy_add_speed;
    mini_chassis_action_to_control_cacl->wz_set=mini_chassis_action_to_control_cacl->wz_normal_speed+mini_chassis_action_to_control_cacl->wz_add_speed;
	if(mini_chassis_action_to_control_cacl->chassis_follow.gimbal_change==CHASSIS_FOLLOW_GIMBAL_CHANGE)
	{
		mini_chassis_action_to_control_cacl->vx_set=-mini_chassis_action_to_control_cacl->vx_set;
		mini_chassis_action_to_control_cacl->vy_set=-mini_chassis_action_to_control_cacl->vy_set;
	}
    if(mini_chassis_action_to_control_cacl->chassis_mode==CHASSIS_FOLLOW_GIMBAL)
    {  
		    cos_yaw=cos(mini_chassis_action_to_control_cacl->relative_angle);
			sin_yaw=sin(mini_chassis_action_to_control_cacl->relative_angle);
			vx=mini_chassis_action_to_control_cacl->vx_set;
		    vy=mini_chassis_action_to_control_cacl->vy_set;
			mini_chassis_action_to_control_cacl->vx_set = vx*cos_yaw + vy*sin_yaw;//跟随相对平滑处理
			mini_chassis_action_to_control_cacl->vy_set = -vx*sin_yaw + vy*cos_yaw;
	}
	if(mini_chassis_action_to_control_cacl->chassis_top_mode==TOP_OFF)
	{
		if(__fabs(mini_chassis_action_to_control_cacl->wz_set)>800)
		{
			mini_chassis_action_to_control_cacl->vx_set= mini_chassis_action_to_control_cacl->vx_set*0.8;
		}
	}
	mini_chassis_action_to_control_cacl->wz_set=First_Lpf_Calc(&mini_chassis_action_to_control_cacl->chassis_cmd_wz_lpf,mini_chassis_action_to_control_cacl->wz_set);
    mini_chassis_action_to_control_cacl->vx_set=slope_plan(&mini_chassis_action_to_control_cacl->chassis_vx);
    mini_chassis_action_to_control_cacl->vy_set=slope_plan(&mini_chassis_action_to_control_cacl->chassis_vy);
    //mini_chassis_action_to_control_cacl->wz_set=slope_plan(&mini_chassis_action_to_control_cacl->chassis_w);
    chassis_vector_to_wheel_speed(mini_chassis_action_to_control_cacl);
}

/**
 * @brief   遥控器平移控制：把摇杆通道映射为 vx/vy 目标速度
 * @param   mini_chassis_rc_to_control_cacl  底盘控制结构体指针
 * @note    vx = x_rc_data * RC_X_SENSE，vy = y_rc_data * RC_Y_SENSE
 *          （RC_X_SENSE / RC_Y_SENSE 为灵敏度标定系数，输出量纲为 rpm）。
 */
static void chassis_rc_to_speed_xy(chassis_control_t *mini_chassis_rc_to_control_cacl)
{
	  
		mini_chassis_rc_to_control_cacl->vx_normal_speed=mini_chassis_rc_to_control_cacl->chassis_gimbal_can_data->x_rc_data*RC_X_SENSE;
		mini_chassis_rc_to_control_cacl->vy_normal_speed=mini_chassis_rc_to_control_cacl->chassis_gimbal_can_data->y_rc_data*RC_Y_SENSE;
}

/**
 * @brief   键盘平移控制：按裁判系统功率上限查表得到最大速度，再按按键给出 vx/vy
 * @param   mini_chassis_keyboard_to_control_cacl  底盘控制结构体指针
 * @note    键盘仅在比赛（key_control==1）模式启用；WASD 映射前后/左右平移，
 *          开启超电（CAP_ON）时最大速度提升到 9000rpm。
 */
static void chassis_keyboard_to_speed_xy(chassis_control_t *mini_chassis_keyboard_to_control_cacl)
{
    if(mini_chassis_keyboard_to_control_cacl == NULL) 
    {
        return; 
    }
    if(mini_chassis_keyboard_to_control_cacl->chassis_rm_data->robot_state.chassis_power_limit==50)
	{
		mini_chassis_keyboard_to_control_cacl->vx_max_speed=4000;
	}
	else if(mini_chassis_keyboard_to_control_cacl->chassis_rm_data->robot_state.chassis_power_limit==60)
	{
		mini_chassis_keyboard_to_control_cacl->vx_max_speed=5400;
	}
	else if(mini_chassis_keyboard_to_control_cacl->chassis_rm_data->robot_state.chassis_power_limit==70)
	{
		mini_chassis_keyboard_to_control_cacl->vx_max_speed=6000;
	}
	else if(mini_chassis_keyboard_to_control_cacl->chassis_rm_data->robot_state.chassis_power_limit==80)
	{
		mini_chassis_keyboard_to_control_cacl->vx_max_speed=6200;
	}
	else if(mini_chassis_keyboard_to_control_cacl->chassis_rm_data->robot_state.chassis_power_limit==90)
	{
		mini_chassis_keyboard_to_control_cacl->vx_max_speed=6800;
	}
	else if(mini_chassis_keyboard_to_control_cacl->chassis_rm_data->robot_state.chassis_power_limit==100)
	{
		mini_chassis_keyboard_to_control_cacl->vx_max_speed=7000;
	}
	else if(mini_chassis_keyboard_to_control_cacl->chassis_rm_data->robot_state.chassis_power_limit==110)
	{
		mini_chassis_keyboard_to_control_cacl->vx_max_speed=8500;
	}
	else if(mini_chassis_keyboard_to_control_cacl->chassis_rm_data->robot_state.chassis_power_limit==120)
	{
		mini_chassis_keyboard_to_control_cacl->vx_max_speed=8000;
	}
	else
	{
		mini_chassis_keyboard_to_control_cacl->vx_max_speed=4200;
	}
	if(mini_chassis_keyboard_to_control_cacl->chassis_cap_mode==CAP_ON)
	{
		mini_chassis_keyboard_to_control_cacl->vx_max_speed=9000;
	}
    if(mini_chassis_keyboard_to_control_cacl->chassis_gimbal_can_data->x_key_data&FORWARED)
    {
        mini_chassis_keyboard_to_control_cacl->vx_normal_speed=-mini_chassis_keyboard_to_control_cacl->vx_max_speed;
    }
    else if(mini_chassis_keyboard_to_control_cacl->chassis_gimbal_can_data->x_key_data&BACK)
    {
        mini_chassis_keyboard_to_control_cacl->vx_normal_speed=mini_chassis_keyboard_to_control_cacl->vx_max_speed;
    }
    else
    {
        mini_chassis_keyboard_to_control_cacl->vx_normal_speed=0;  
    }
    if(mini_chassis_keyboard_to_control_cacl->chassis_gimbal_can_data->y_key_data&RGHTWARD)
    {
        mini_chassis_keyboard_to_control_cacl->vy_normal_speed=CHASSIS_SPEED_MAX_VY_KEYBOARD;
    }
    else if(mini_chassis_keyboard_to_control_cacl->chassis_gimbal_can_data->y_key_data&LEFTWARD)
    {
        mini_chassis_keyboard_to_control_cacl->vy_normal_speed=-CHASSIS_SPEED_MAX_VY_KEYBOARD;
    }
    else
    {
        mini_chassis_keyboard_to_control_cacl->vy_normal_speed=0;
    }
}
/**
 * @brief   底盘功率限制：在功率预算内反解每轮允许的最大电流
 * @param   hero_chassis_power_speed_control  底盘控制结构体指针
 * @note    核心思路：电机功率 P = K0+K1·I+K2·ω+K3·ω·I+K4·I2+K5·ω2（拟合式），
 *          其中 I 为待求电流（A）、ω 为当前角速度（rad/s）。给定“本轮可用功率
 *          上限”，即解一元二次方程 a·I2+b·I+c=0 求 I，再换算回给电机的
 *          give_current = I / MA_A（MA_A 为电流比例系数）。
 *          可用功率 = 裁判系统功率上限 + 超电增益 + 缓冲能量 PID（缓冲闭环）；
 *          若实际需求功率 <= 上限则不加限制。
 */
static void chassis_power_limit(chassis_control_t *hero_chassis_power_speed_control)
{
    if(hero_chassis_power_speed_control == NULL)
    {
        return;
    }
	static fp32 rm_chassis_power_limit =55.0f;  //p_max  来自RM
	static fp32 rm_chassis_power_buffer = 60.0f; //p_buff 来自RM
	static fp32 user_chassis_power = 0.0f;      //当前电机所需消耗
	static fp32 chassis_power_max = 0.0f;       //最大可放功率
	static fp32 chassis_cap_gain=0.0f;         //开启超电功率
	static fp32 chassis_buffer_rm_max=0.0f;    //缓冲上限
	static fp32 chassos_buffer_rm_use=0.0f;    //用户使用缓冲功率
	static fp32 chassis_motor_i1=0.0f;         //电机的电流解1
	static fp32 chassis_motor_i2=0.0f;              //电机的电流解2
	static fp32 a=0.0f;							   //方程a项
	static fp32 b=0.0f;							   //方程b项	
	static fp32 c=0.0f;							   //方程c项	
	static fp32 result=0.0f;                        //中间平方根结果
	static fp32 numerator=0.0f;                     //衰减分子
	static fp32 denominator=0.0f;                   //衰减分母
	if(hero_chassis_power_speed_control->chassis_cap_mode==CAP_ON&&hero_chassis_power_speed_control->super_cap->V_cap_value>=19)
	{
		chassis_cap_gain=200;
	}
	else
	{
		chassis_cap_gain=0;
	}
    rm_chassis_power_limit=hero_chassis_power_speed_control->chassis_rm_data->robot_state.chassis_power_limit;
    rm_chassis_power_buffer=hero_chassis_power_speed_control->chassis_rm_data->power_heat_data_t.buffer_energy;
	chassis_buffer_rm_max=hero_chassis_power_speed_control->chassis_buffer_power_limit;
    user_chassis_power=hero_chassis_power_speed_control->chassis_power_demand;
	chassos_buffer_rm_use=rm_chassis_power_buffer-hero_chassis_power_speed_control->chassis_buffer_power_used;
	chassis_power_max=rm_chassis_power_limit+chassis_cap_gain+pid_calc(&hero_chassis_power_speed_control->rm_buffer_control_pid,rm_chassis_power_buffer,20);
	if(user_chassis_power<=chassis_power_max)
	{
		for(uint8_t i=0;i<4;i++)
		{
			return;
		}
	}
	else
	{
		for(uint8_t i=0;i<4;i++)
		{
			hero_chassis_power_speed_control->chassis_motor[i].motor_power_limit=hero_chassis_power_speed_control->chassis_motor[i].motor_demand_power*chassis_power_max/user_chassis_power;//进行功率动态分配以及衰减
		}
	}
	for(uint8_t i=0;i<4;i++)
	{
		a=K4;
		b=K1+K3*hero_chassis_power_speed_control->chassis_motor[i].motor_omega;
		c=K0+K2*hero_chassis_power_speed_control->chassis_motor[i].motor_omega+K5*hero_chassis_power_speed_control->chassis_motor[i].motor_omega*hero_chassis_power_speed_control->chassis_motor[i].motor_omega-hero_chassis_power_speed_control->chassis_motor[i].motor_power_limit;
		if(b*b-4*a*c<0)
		{
			chassis_motor_i1=chassis_motor_i2=0;
		}
		else
		{
			if(b*b-4*a*c==0)
			{
				arm_sqrt_f32(b*b-4*a*c,&result);
				chassis_motor_i1=chassis_motor_i2=(-b+result)/(2*a);
				hero_chassis_power_speed_control->chassis_motor[i].give_current=chassis_motor_i1/MA_A;
			}
			else
			{
				arm_sqrt_f32(b*b-4*a*c,&result);
				chassis_motor_i1=(-b+result)/(2*a);
				chassis_motor_i2=(-b-result)/(2*a);
				if(Abs(chassis_motor_i1-hero_chassis_power_speed_control->chassis_motor[i].motor_use_current)>Abs(chassis_motor_i2-hero_chassis_power_speed_control->chassis_motor[i].motor_use_current))
				{
					hero_chassis_power_speed_control->chassis_motor[i].give_current=chassis_motor_i2/MA_A;
				}
				else
				{
					hero_chassis_power_speed_control->chassis_motor[i].give_current=chassis_motor_i1/MA_A;
				}
			}			
		}

	}
}

/**
 * @brief   更新功率缓冲能量的可用上限档位
 * @param   chassis_power_feeback_updata  底盘控制结构体指针
 * @note    根据裁判系统 buffer_energy：>70 时按“全增益”档给缓冲功率，
 *          <=60 时按“无增益”档；60~70 之间保持上一档（滞回，防抖）。
 */
static void chassis_update_power_buffer(chassis_control_t *chassis_power_feeback_updata)
{
    if(chassis_power_feeback_updata == NULL)
    {
        return;
    }
    if(chassis_power_feeback_updata->chassis_rm_data->power_heat_data_t.buffer_energy>70)
    {
        chassis_power_feeback_updata->chassis_buffer_power_limit=CHASSIS_POWER_BUFFER_ALL_GAIN;
        chassis_power_feeback_updata->chassis_buffer_power_used=CHASSIS_POWER_BUFFER_USE_MAX_GAIN;
    }
    else if(chassis_power_feeback_updata->chassis_rm_data->power_heat_data_t.buffer_energy<=60)
    {
        chassis_power_feeback_updata->chassis_buffer_power_limit=CHASSIS_POWER_BUFFER_ALl_NO_GAIN;
        chassis_power_feeback_updata->chassis_buffer_power_used=CHASSIS_POWER_BUFFER_USE_MAX_NO_GAIN;
    }

}

/**
 * @brief   麦轮运动学逆解：把底盘 vx/vy/wz 目标反解为四个麦轮的目标转速
 * @param   chassis_vector_to_mecaum_speed  底盘控制结构体指针
 * @note    麦轮布置：四个轮呈 X 型，正负号组合体现各轮对平移/自旋的贡献；
 *          wz 需乘电机到底盘中心的距离系数 MOTOR_DISTANCE_TO_CENTER 统一量纲。
 *          轮0 = -vx+vy+wz；轮1 = +vx+vy+wz；轮2 = +vx-vy+wz；轮3 = -vx-vy+wz。
 */
static void chassis_vector_to_wheel_speed(chassis_control_t *chassis_vector_to_mecaum_speed)
{
    static fp32 vx_set=0.0f;
    static fp32 vy_set=0.0f;
    static fp32 wz_set=0.0f;
    if(chassis_vector_to_mecaum_speed == NULL)
    {
        return;
    }
    vx_set=chassis_vector_to_mecaum_speed->vx_set;
    vy_set=chassis_vector_to_mecaum_speed->vy_set;
    wz_set=chassis_vector_to_mecaum_speed->wz_set*MOTOR_DISTANCE_TO_CENTER;
    chassis_vector_to_mecaum_speed->chassis_motor[0].speed_set=-vx_set+vy_set+wz_set;
    chassis_vector_to_mecaum_speed->chassis_motor[1].speed_set=vx_set+vy_set+wz_set;
    chassis_vector_to_mecaum_speed->chassis_motor[2].speed_set=vx_set-vy_set+wz_set;
    chassis_vector_to_mecaum_speed->chassis_motor[3].speed_set=-vx_set-vy_set+wz_set;
}

/**
 * @brief   电机编码器原始值 -> 相对角度（弧度）
 * @param   ecd                  当前编码器原始值（0..ECD_RANGE）
 * @param   ecd_zero_angle_set   零位编码值（安装零点）
 * @return  相对零位的角度，单位 rad（弧度）
 * @note    处理编码器回绕：差值超过半圈则加减一整圈，保证角度连续无跳变。
 */
static fp32 motor_encoder_to_angle(uint16_t ecd,uint16_t ecd_zero_angle_set)
{
    int32_t relative_ecd=ecd-ecd_zero_angle_set;
    if(relative_ecd>HALF_ECD_RANGE)
    {
        relative_ecd-=ECD_RANGE;
    }
    else if(relative_ecd<-HALF_ECD_RANGE)
    {
        relative_ecd+=ECD_RANGE;
    }
    return relative_ecd*MOTOR_ECD_TO_RAD;
}
