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
 * 底盘模式（operating_mode）：
 *   CHASSIS_OPERATING_MODE_ZERO_FORCE       : 零力矩（完全无输出，电机自由）
 *   CHASSIS_OPERATING_MODE_HOLD_POSITION          : 原地不动（目标速度置 0）
 *   CHASSIS_OPERATING_MODE_INDEPENDENT : 不随云台（平移，不自旋）
 *   CHASSIS_OPERATING_MODE_FOLLOW_GIMBAL    : 云台跟随（平移 + 跟随云台 yaw 自旋）
 *
 * 速度约定（单位统一）：
 *   - 平移速度 forward_speed_revolution_per_minute / leftward_speed_revolution_per_minute        : 底盘坐标系，单位 rpm（麦轮转速折算，见 MOTOR_SPEED_TO_CHASSIS_SPEED_*）
 *   - 自旋角速度 rotation_speed_radian_per_second           : 单位 rpm（已按电机编码折算）
 *   - 电流 commanded_torque_current_count       : 单位 A（安培）
 *   - 时间                   : current_time_millisecond 由 osKernelGetTickCount() 获取，单位 ms
 *
 * 控制说明：
 *   - 遥控器通道经死区（Remote_Deadband_Limit）与灵敏度（RC_X/Y_SENSE）映射为速度；
 *   - 键盘模式按裁判系统功率上限（Chassis_LimitPower）查表得到最大速度；
 *   - 云台跟随模式下用 PID（follow_gimbal_angle_loop_pid）把底盘 yaw 对齐云台 yaw；
 *   - 功率限制用“二次方程反解电流”（Chassis_LimitPower）在功率预算内限制电流。
 */

/**
 * @brief   底盘控制结构体初始化（仅在任务启动时调用一次）
 * @param   chassis_control_state  底盘控制结构体指针
 * @note    完成：默认模式（零力矩）、速度/功率清零、四路电机反馈指针绑定、
 *          PID 参数初始化、云台/INS/遥控器/裁判系统/超电数据指针绑定、
 *          三个轴的低通滤波（First_Lpf）与斜坡规划（slope_init）初始化。
 *          所有指针均来自各模块的 get_*_point() 观测点（共享内存/外设数据）。
 */
static void Chassis_Initialize(chassis_control_t *chassis_control_state);                                               //底盘初始化
/**
 * @brief   从云台 CAN 数据解析本次控制模式（底盘/超电/陀螺/掉头）
 * @param   chassis_control_state  底盘控制结构体指针
 * @note    把来自云台（gimbal_command_data_pointer）的 mode/super_capacitor_mode/top_spin_mode/follow_gimbal_state
 *          整数编码翻译为对应的底盘枚举模式，供后续控制逻辑分支使用。
 */
static void Chassis_ParseOperatingMode(chassis_control_t *chassis_control_state);                                       //底盘模式设置
/**
 * @brief   检测模式是否发生切换，并记录上一次模式
 * @param   chassis_control_state  底盘控制结构体指针
 * @note    仅当 operating_mode != previous_operating_mode 时更新 last；可用于检测模式跳变沿。
 */
static void Chassis_DetectOperatingModeChange(chassis_control_t *chassis_control_state); //底盘模式切换反馈
/**
 * @brief   更新底盘反馈量（电机速度/电流、功率、当前底盘速度、云台相对角）
 * @param   chassis_control_state  底盘控制结构体指针
 * @note    逐电机：读取 CAN 编码器反馈 rotor_speed_revolution_per_minute 与 commanded_torque_current_count；
 *          由四个轮速按麦轮运动学逆解得到底盘当前 forward_speed_revolution_per_minute / leftward_speed_revolution_per_minute / rotation_speed_radian_per_second；
 *          用 chassis_power_fit_coefficient_0..chassis_power_fit_coefficient_5 拟合电机功率 P = f(I,ω)（电流 A、角速度 rad/s）；
 *          更新云台相对角 gimbal_relative_angle_radian（随/不随云台模式取不同零点）。
 */
static void Chassis_UpdateFeedback(chassis_control_t *chassis_control_state);                         //底盘反馈更新
/**
 * @brief   根据底盘模式决定目标速度来源
 * @param   chassis_control_state  底盘控制结构体指针
 * @note    CHASSIS_OPERATING_MODE_HOLD_POSITION：目标速度强制为 0；
 *          NO_FOLLOW / FOLLOW：调用 Chassis_CalculateSpeedSetpoint 由遥控器/键盘解算；
 *          ZERO_FORCE：无需设置目标（控制阶段直接清零输出）。
 */
static void Chassis_SetSpeedSetpoint(chassis_control_t *chassis_control_state);                       //底盘模式切换控制
/**
 * @brief   底盘速度环 PID 控制计算（含功率限制与零力矩处理）
 * @param   chassis_control_state  底盘控制结构体指针
 * @note    1) 根据四轮平均速度动态调整 PID 的 ki 与积分限幅 max_iout（变增益抗饱和）；
 *          2) 对每轮做速度 PID：输入 = 目标转速 rotor_speed_setpoint_revolution_per_minute（叠加云台跟随前馈项），
 *             反馈 = 实际转速 rotor_speed_revolution_per_minute，输出 = 给电机电流 commanded_torque_current_count（单位 A）；
 *          3) 由解算电流预估实际功率，做功率限制（Chassis_LimitPower）；
 *          4) ZERO_FORCE 模式下最终把所有电机输出电流清零。
 */
static void Chassis_CalculateSpeedControlOutput(chassis_control_t *chassis_control_state);                               //底盘模式控制计算
/*子函数*/
/**
 * @brief   由遥控器/键盘原始输入解算底盘目标速度（含跟随、斜坡、滤波）
 * @param   chassis_control_state  底盘控制结构体指针
 * @note    流程：
 *          1. 按 keyboard_control_enabled 选择键盘或遥控器作为 forward_speed_revolution_per_minute/leftward_speed_revolution_per_minute 平移来源；
 *          2. 若陀螺开启（TOP_SPIN_MODE_ON）按方向设定自旋 rotation_speed_radian_per_second；陀螺关闭且跟随云台时用
 *             跟随 PID 把 rotation_speed_radian_per_second 对齐云台（含掉头切换的等待/平滑处理）；
 *          3. 得到 forward_speed_setpoint_revolution_per_minute/leftward_speed_setpoint_revolution_per_minute/rotation_speed_setpoint_radian_per_second（速度单位：rpm）；
 *          4. 云台跟随模式下把平移速度旋转到云台系（坐标系旋转变换）；
 *          5. rotation_speed_radian_per_second 经一阶低通（First_Lpf_Calc），forward_speed_revolution_per_minute/leftward_speed_revolution_per_minute 经斜坡规划（slope_plan）限速；
 *          6. 最后反解四个麦轮目标转速（Chassis_ConvertVectorToWheelSpeed）。
 */
static void Chassis_CalculateSpeedSetpoint(chassis_control_t *chassis_control_state);           //行为控制量计算
/**
 * @brief   遥控器平移控制：把摇杆通道映射为 forward_speed_revolution_per_minute/leftward_speed_revolution_per_minute 目标速度
 * @param   chassis_control_state  底盘控制结构体指针
 * @note    forward_speed_revolution_per_minute = forward_channel_value * remote_control_forward_sensitivity_count，leftward_speed_revolution_per_minute = leftward_channel_value * remote_control_leftward_sensitivity_count
 *          （remote_control_forward_sensitivity_count / remote_control_leftward_sensitivity_count 为灵敏度标定系数，输出量纲为 rpm）。
 */
static void Chassis_ConvertRemoteControlToSpeed(chassis_control_t *chassis_control_state);                     //遥控器控制XY计算
/**
 * @brief   键盘平移控制：按裁判系统功率上限查表得到最大速度，再按按键给出 forward_speed_revolution_per_minute/leftward_speed_revolution_per_minute
 * @param   chassis_control_state  底盘控制结构体指针
 * @note    键盘仅在比赛（keyboard_control_enabled==1）模式启用；WASD 映射前后/左右平移，
 *          开启超电（SUPER_CAPACITOR_MODE_ON）时最大速度提升到 9000rpm。
 */
static void Chassis_ConvertKeyboardToSpeed(chassis_control_t *chassis_control_state);         //键盘控制XY计算
/**
 * @brief   底盘功率限制：在功率预算内反解每轮允许的最大电流
 * @param   chassis_control_state  底盘控制结构体指针
 * @note    核心思路：电机功率 P = chassis_power_fit_coefficient_0+chassis_power_fit_coefficient_1·I+chassis_power_fit_coefficient_2·ω+chassis_power_fit_coefficient_3·ω·I+chassis_power_fit_coefficient_4·I2+chassis_power_fit_coefficient_5·ω2（拟合式），
 *          其中 I 为待求电流（A）、ω 为当前角速度（rad/s）。给定“本轮可用功率
 *          上限”，即解一元二次方程 a·I2+b·I+c=0 求 I，再换算回给电机的
 *          commanded_torque_current_count = I / current_count_to_ampere（current_count_to_ampere 为电流比例系数）。
 *          可用功率 = 裁判系统功率上限 + 超电增益 + 缓冲能量 PID（缓冲闭环）；
 *          若实际需求功率 <= 上限则不加限制。
 */
static void Chassis_ApplyPowerLimit(chassis_control_t *chassis_control_state);                 //功率速度限制控制
/**
 * @brief   更新功率缓冲能量的可用上限档位
 * @param   chassis_control_state  底盘控制结构体指针
 * @note    根据裁判系统 buffer_energy：>70 时按“全增益”档给缓冲功率，
 *          <=60 时按“无增益”档；60~70 之间保持上一档（滞回，防抖）。
 */
static void Chassis_UpdatePowerBuffer(chassis_control_t *chassis_control_state);                    //功率反馈更新
/**
 * @brief   麦轮运动学逆解：把底盘 forward_speed_revolution_per_minute/leftward_speed_revolution_per_minute/rotation_speed_radian_per_second 目标反解为四个麦轮的目标转速
 * @param   chassis_control_state  底盘控制结构体指针
 * @note    麦轮布置：四个轮呈 X 型，正负号组合体现各轮对平移/自旋的贡献；
 *          rotation_speed_radian_per_second 需乘电机到底盘中心的距离系数 motor_distance_to_chassis_center_meter 统一量纲。
 *          轮0 = -forward_speed_revolution_per_minute+leftward_speed_revolution_per_minute+rotation_speed_radian_per_second；轮1 = +forward_speed_revolution_per_minute+leftward_speed_revolution_per_minute+rotation_speed_radian_per_second；轮2 = +forward_speed_revolution_per_minute-leftward_speed_revolution_per_minute+rotation_speed_radian_per_second；轮3 = -forward_speed_revolution_per_minute-leftward_speed_revolution_per_minute+rotation_speed_radian_per_second。
 */
static void Chassis_ConvertVectorToWheelSpeed(chassis_control_t *chassis_control_state);                //底盘向量转麦轮各电机速度
/**
 * @brief   电机编码器原始值 -> 相对角度（弧度）
 * @param   encoder_count                  当前编码器原始值（0..motor_encoder_count_per_revolution）
 * @param   encoder_zero_count   零位编码值（安装零点）
 * @return  相对零位的角度，单位 rad（弧度）
 * @note    处理编码器回绕：差值超过半圈则加减一整圈，保证角度连续无跳变。
 */
static fp32 Chassis_ConvertEncoderCountToAngleRadian(uint16_t encoder_count,uint16_t encoder_zero_count);                                     //电机角度转换
fp32 gimbal_relative_angle_sine =0.0f;
fp32 gimbal_relative_angle_cosine =0.0f; 
chassis_control_t chassis_control_state;
/**
 * @brief   底盘控制任务主循环（CMSIS-RTOS2 线程入口）
 * @param   argument  CMSIS-RTOS2 线程参数（本工程未使用，恒为 NULL）
 * @note    周期 = 1ms（while 内 osDelay(1)）。
 *          每周期执行：模式解析 -> 反馈更新 -> 目标解算 -> 控制计算 -> CAN 发送。
 *          current_time_millisecond 记录本次周期系统 tick，供时间相关逻辑使用（单位 ms）。
 */
void Chassis_Task(void *argument)
{   osDelay(chassis_initialization_delay_millisecond);
    uint32_t current_time_millisecond;
    Chassis_Initialize(&chassis_control_state);
	osDelay(chassis_initialization_delay_millisecond);
    while (1) 
    {   
       current_time_millisecond = osKernelGetTickCount();
        Chassis_ParseOperatingMode(&chassis_control_state);
       Chassis_DetectOperatingModeChange(&chassis_control_state);
       Chassis_UpdateFeedback(&chassis_control_state);
        Chassis_SetSpeedSetpoint(&chassis_control_state);
       Chassis_CalculateSpeedControlOutput(&chassis_control_state);
    	CAN_SendChassisCommand(chassis_control_state.motor_states[0].commanded_torque_current_count,chassis_control_state.motor_states[1].commanded_torque_current_count,chassis_control_state.motor_states[2].commanded_torque_current_count,chassis_control_state.motor_states[3].commanded_torque_current_count);
		
//    #if INCLUDE_uxTaskGetStackHighWaterMark
//        chassis_high_water = uxTaskGetStackHighWaterMark(NULL); //获取任务栈使用情况
//    #endif
    osDelay(1);
	}
}

/**
 * @brief   底盘控制结构体初始化（仅在任务启动时调用一次）
 * @param   chassis_control_state  底盘控制结构体指针
 * @note    完成：默认模式（零力矩）、速度/功率清零、四路电机反馈指针绑定、
 *          PID 参数初始化、云台/INS/遥控器/裁判系统/超电数据指针绑定、
 *          三个轴的低通滤波（First_Lpf）与斜坡规划（slope_init）初始化。
 *          所有指针均来自各模块的 get_*_point() 观测点（共享内存/外设数据）。
 */
static void Chassis_Initialize(chassis_control_t *chassis_control_state)
{
    if(chassis_control_state == NULL)
    {
        return;
    }
    chassis_control_state->operating_mode = CHASSIS_OPERATING_MODE_ZERO_FORCE;
    chassis_control_state->super_capacitor_mode = SUPER_CAPACITOR_MODE_OFF;
    chassis_control_state->top_spin_mode = TOP_SPIN_MODE_OFF;
    chassis_control_state->forward_speed_revolution_per_minute = 0;
    chassis_control_state->leftward_speed_revolution_per_minute = 0;
    chassis_control_state->rotation_speed_radian_per_second = 0;
    chassis_control_state->forward_speed_setpoint_revolution_per_minute = 0;
    chassis_control_state->leftward_speed_setpoint_revolution_per_minute = 0;
    chassis_control_state->rotation_speed_setpoint_radian_per_second = 0;
    chassis_control_state->forward_speed_normal_revolution_per_minute=0;
    chassis_control_state->leftward_speed_normal_revolution_per_minute=0;
    chassis_control_state->rotation_speed_normal_radian_per_second=0;
    chassis_control_state->forward_speed_increment_revolution_per_minute=0;
    chassis_control_state->leftward_speed_increment_revolution_per_minute=0;
    chassis_control_state->rotation_speed_increment_radian_per_second=0;
    chassis_control_state->mechanical_power_watt = 0;
    chassis_control_state->power_available_watt = 0;
    chassis_control_state->power_maximum_watt = 0;
    chassis_control_state->mechanical_power_watt = 0;
	chassis_control_state->follow_gimbal_transition.state = CHASSIS_FOLLOW_GIMBAL_STATE_NORMAL;
	chassis_control_state->follow_gimbal_transition.gimbal_turnaround_elapsed_time_millisecond=0;
	chassis_control_state->follow_gimbal_transition.chassis_response_pending=0;
	chassis_control_state->forward_speed_slope_acceleration=chassis_forward_speed_acceleration_rate;
	chassis_control_state->forward_speed_slope_rate=chassis_forward_speed_deceleration_rate;
	chassis_control_state->leftward_speed_slope_acceleration=chassis_leftward_speed_acceleration_rate;
	chassis_control_state->leftward_speed_slope_rate=chassis_leftward_speed_deceleration_rate;
	chassis_control_state->rotation_speed_slope_acceleration=chassis_rotation_speed_acceleration_rate;
	chassis_control_state->rotation_speed_slope_rate=chassis_rotation_speed_deceleration_rate;
    const fp32 chassis_motor_speed_pid[3]={chassis_speed_loop_proportional_gain,chassis_speed_loop_integral_gain,chassis_speed_loop_derivative_gain};
    const fp32 follow_gimbal_angle_loop_pid[3]={chassis_follow_gimbal_angle_loop_proportional_gain,chassis_follow_gimbal_angle_loop_integral_gain,chassis_follow_gimbal_angle_loop_derivative_gain};
	const fp32 chassis_buffer_pid[3]={chassis_power_buffer_loop_proportional_gain,chassis_power_buffer_loop_integral_gain,chassis_power_buffer_loop_derivative_gain};
    /*电机指针初始化*/
    for(int i=0;i<4;i++)
    {
       chassis_control_state->motor_states[i].motor_measurement_pointer = get_motor_chassis_motor_measure_point(i);
    }
	chassis_control_state->gimbal_yaw_motor_measurement_pointer=get_motor_gimbal_yaw_measure_point();
    /*云台数据指针初始化*/
    chassis_control_state->gimbal_command_data_pointer = get_c_gimbal_data_point();
    /*PID初始化*/
    for(int i=0;i<4;i++)
    {
         pid_init(&chassis_control_state->speed_loop_pid[i],PID_DELTA,chassis_motor_speed_pid,chassis_speed_loop_maximum_output_count,chassis_speed_loop_maximum_integral_output_count);
    }
    pid_init(&chassis_control_state->follow_gimbal_angle_loop_pid,PID_POSITION,follow_gimbal_angle_loop_pid,chassis_follow_gimbal_angle_loop_maximum_output_count,chassis_follow_gimbal_angle_loop_maximum_integral_output_count);
    pid_init(&chassis_control_state->power_buffer_loop_pid,PID_POSITION,chassis_buffer_pid,chassis_power_buffer_loop_maximum_output_count,chassis_power_buffer_loop_maximum_integral_output_count);
    /*INS初始化*/
		chassis_control_state->inertial_measurement_angle_pointer=get_gyro_data_point();
    /*遥控器指针初始化*/
    chassis_control_state->remote_control_data_pointer = get_dt7_remote_control_point();
    /*裁判系统指针初始化*/ 
    chassis_control_state->referee_system_data_pointer=get_Referee_point();
    /*超电数据指针初始化*/
    chassis_control_state->super_capacitor_measurement_pointer=get_super_cap_measure_point();
		/*低通输入初始化*/
		First_Lpf_Init(&chassis_control_state->forward_speed_command_low_pass_filter,chassis_forward_speed_low_pass_filter_alpha,0.001);
		First_Lpf_Init(&chassis_control_state->leftward_speed_command_low_pass_filter,chassis_leftward_speed_low_pass_filter_alpha,0.001);
		First_Lpf_Init(&chassis_control_state->rotation_speed_command_low_pass_filter,chassis_rotation_speed_low_pass_filter_alpha,0.001);	
		slope_init(&chassis_control_state->forward_speed_slope_plan,chassis_control_state->forward_speed_slope_acceleration,chassis_control_state->forward_speed_slope_rate,&chassis_control_state->forward_speed_revolution_per_minute,&chassis_control_state->forward_speed_setpoint_revolution_per_minute);
		slope_init(&chassis_control_state->leftward_speed_slope_plan,chassis_control_state->leftward_speed_slope_acceleration,chassis_control_state->leftward_speed_slope_rate,&chassis_control_state->leftward_speed_revolution_per_minute,&chassis_control_state->leftward_speed_setpoint_revolution_per_minute);
		slope_init(&chassis_control_state->rotation_speed_slope_plan,chassis_control_state->rotation_speed_slope_acceleration,chassis_control_state->rotation_speed_slope_rate,&chassis_control_state->rotation_speed_radian_per_second,&chassis_control_state->rotation_speed_setpoint_radian_per_second);
    Chassis_UpdateFeedback(chassis_control_state);
}

/**
 * @brief   从云台 CAN 数据解析本次控制模式（底盘/超电/陀螺/掉头）
 * @param   chassis_control_state  底盘控制结构体指针
 * @note    把来自云台（gimbal_command_data_pointer）的 mode/super_capacitor_mode/top_spin_mode/follow_gimbal_state
 *          整数编码翻译为对应的底盘枚举模式，供后续控制逻辑分支使用。
 */
static void Chassis_ParseOperatingMode(chassis_control_t *chassis_control_state)
{
    if(chassis_control_state == NULL) 
    {
        return; 
    }
    /*底盘模式*/
        if(chassis_control_state->gimbal_command_data_pointer->operating_mode==0)
		{
				chassis_control_state->operating_mode=CHASSIS_OPERATING_MODE_ZERO_FORCE;
		}
		else if(chassis_control_state->gimbal_command_data_pointer->operating_mode==1)
		{
				chassis_control_state->operating_mode=CHASSIS_OPERATING_MODE_HOLD_POSITION;
		}
		else if(chassis_control_state->gimbal_command_data_pointer->operating_mode==2)
		{
				chassis_control_state->operating_mode=CHASSIS_OPERATING_MODE_INDEPENDENT;
		}
		else if(chassis_control_state->gimbal_command_data_pointer->operating_mode==3)
		{
				chassis_control_state->operating_mode=CHASSIS_OPERATING_MODE_FOLLOW_GIMBAL;
		}
		/*超电模式*/
		if(chassis_control_state->gimbal_command_data_pointer->super_capacitor_mode==0)
		{
				chassis_control_state->super_capacitor_mode=SUPER_CAPACITOR_MODE_OFF;
		}
		else if(chassis_control_state->gimbal_command_data_pointer->super_capacitor_mode==1)
		{
				chassis_control_state->super_capacitor_mode=SUPER_CAPACITOR_MODE_ON;
		}
		/*陀螺模式*/
		if(chassis_control_state->gimbal_command_data_pointer->top_spin_mode==0)
		{
				chassis_control_state->top_spin_mode=TOP_SPIN_MODE_OFF;
		}
		else if(chassis_control_state->gimbal_command_data_pointer->top_spin_mode)
		{
				chassis_control_state->top_spin_mode=TOP_SPIN_MODE_ON;
		}
		/*掉头模式*/
		chassis_control_state->follow_gimbal_transition.previous_state=chassis_control_state->follow_gimbal_transition.state;
		if(chassis_control_state->gimbal_command_data_pointer->follow_gimbal_state==0)
		{
			chassis_control_state->follow_gimbal_transition.state=CHASSIS_FOLLOW_GIMBAL_STATE_NORMAL;
		}
		else if(chassis_control_state->gimbal_command_data_pointer->follow_gimbal_state==1)
		{
			chassis_control_state->follow_gimbal_transition.state=CHASSIS_FOLLOW_GIMBAL_STATE_TURNAROUND;
		}
}

/**
 * @brief   检测模式是否发生切换，并记录上一次模式
 * @param   chassis_control_state  底盘控制结构体指针
 * @note    仅当 operating_mode != previous_operating_mode 时更新 last；可用于检测模式跳变沿。
 */
static void Chassis_DetectOperatingModeChange(chassis_control_t *chassis_control_state)
{
    if(chassis_control_state == NULL)
    {
        return;
    }
    if(chassis_control_state->operating_mode == chassis_control_state->previous_operating_mode)
    {
        return; 
    }
    chassis_control_state->previous_operating_mode = chassis_control_state->operating_mode;
}

/**
 * @brief   更新底盘反馈量（电机速度/电流、功率、当前底盘速度、云台相对角）
 * @param   chassis_control_state  底盘控制结构体指针
 * @note    逐电机：读取 CAN 编码器反馈 rotor_speed_revolution_per_minute 与 commanded_torque_current_count；
 *          由四个轮速按麦轮运动学逆解得到底盘当前 forward_speed_revolution_per_minute / leftward_speed_revolution_per_minute / rotation_speed_radian_per_second；
 *          用 chassis_power_fit_coefficient_0..chassis_power_fit_coefficient_5 拟合电机功率 P = f(I,ω)（电流 A、角速度 rad/s）；
 *          更新云台相对角 gimbal_relative_angle_radian（随/不随云台模式取不同零点）。
 */
static void Chassis_UpdateFeedback(chassis_control_t *chassis_control_state)
{
    if(chassis_control_state == NULL)
    {
        return;
    }
	if(chassis_control_state->follow_gimbal_transition.state==CHASSIS_FOLLOW_GIMBAL_STATE_NORMAL)
	{   
		if(chassis_control_state->gimbal_yaw_motor_measurement_pointer->encoder_count==0)
		{
			
			chassis_control_state->gimbal_relative_angle_radian=Chassis_ConvertEncoderCountToAngleRadian(chassis_control_state->gimbal_yaw_motor_measurement_pointer->previous_encoder_count,gimbal_yaw_zero_encoder_count);
		}
		chassis_control_state->gimbal_relative_angle_radian=Chassis_ConvertEncoderCountToAngleRadian(chassis_control_state->gimbal_yaw_motor_measurement_pointer->encoder_count,gimbal_yaw_zero_encoder_count);
	}
	else if(chassis_control_state->follow_gimbal_transition.state==CHASSIS_FOLLOW_GIMBAL_STATE_TURNAROUND)
	{
	    chassis_control_state->gimbal_relative_angle_radian=Chassis_ConvertEncoderCountToAngleRadian(chassis_control_state->gimbal_yaw_motor_measurement_pointer->encoder_count,gimbal_yaw_turnaround_encoder_count);
	}
    for(int i=0;i<4;i++)
    {	
        chassis_control_state->motor_states[i].rotor_speed_revolution_per_minute = chassis_control_state->motor_states[i].motor_measurement_pointer->rotor_speed_revolution_per_minute;
        chassis_control_state->motor_states[i].commanded_torque_current_count = chassis_control_state->motor_states[i].motor_measurement_pointer->commanded_torque_current_count;
				chassis_control_state->motor_states[i].measured_torque_current_ampere=chassis_control_state->motor_states[i].motor_measurement_pointer->commanded_torque_current_count*current_count_to_ampere;
				chassis_control_state->motor_states[i].rotor_angular_velocity_radian_per_second = chassis_control_state->motor_states[i].motor_measurement_pointer->rotor_speed_revolution_per_minute*revolution_per_minute_to_radian_per_second;
        chassis_control_state->motor_states[i].mechanical_power_watt = (chassis_power_fit_coefficient_0+(chassis_power_fit_coefficient_1*chassis_control_state->motor_states[i].measured_torque_current_ampere)+
					(chassis_power_fit_coefficient_2*chassis_control_state->motor_states[i].rotor_angular_velocity_radian_per_second)+
						(chassis_power_fit_coefficient_3*chassis_control_state->motor_states[i].rotor_angular_velocity_radian_per_second*chassis_control_state->motor_states[i].measured_torque_current_ampere)+
							(chassis_power_fit_coefficient_4*chassis_control_state->motor_states[i].measured_torque_current_ampere*chassis_control_state->motor_states[i].measured_torque_current_ampere)+
								(chassis_power_fit_coefficient_5*chassis_control_state->motor_states[i].rotor_angular_velocity_radian_per_second*chassis_control_state->motor_states[i].rotor_angular_velocity_radian_per_second)); 
        chassis_control_state->forward_speed_revolution_per_minute = (-chassis_control_state->motor_states[0].rotor_speed_revolution_per_minute + chassis_control_state->motor_states[1].rotor_speed_revolution_per_minute + chassis_control_state->motor_states[2].rotor_speed_revolution_per_minute - chassis_control_state->motor_states[3].rotor_speed_revolution_per_minute)*motor_rotor_speed_to_forward_speed_scale;
        chassis_control_state->leftward_speed_revolution_per_minute = (-chassis_control_state->motor_states[0].rotor_speed_revolution_per_minute - chassis_control_state->motor_states[1].rotor_speed_revolution_per_minute + chassis_control_state->motor_states[2].rotor_speed_revolution_per_minute + chassis_control_state->motor_states[3].rotor_speed_revolution_per_minute)*motor_rotor_speed_to_leftward_speed_scale;
        chassis_control_state->rotation_speed_radian_per_second = (-chassis_control_state->motor_states[0].rotor_speed_revolution_per_minute - chassis_control_state->motor_states[1].rotor_speed_revolution_per_minute - chassis_control_state->motor_states[2].rotor_speed_revolution_per_minute - chassis_control_state->motor_states[3].rotor_speed_revolution_per_minute)*motor_rotor_speed_to_rotation_speed_scale/motor_distance_to_chassis_center_meter;
    }
    chassis_control_state->mechanical_power_watt=chassis_control_state->motor_states[0].mechanical_power_watt+chassis_control_state->motor_states[1].mechanical_power_watt+chassis_control_state->motor_states[2].mechanical_power_watt+chassis_control_state->motor_states[3].mechanical_power_watt;
    chassis_control_state->chassis_angular_velocity_radian_per_second=*(chassis_control_state->inertial_measurement_angle_pointer+INS_GYRO_Z_ADDRESS_OFFSET);
    Chassis_UpdatePowerBuffer(chassis_control_state);
}

/**
 * @brief   根据底盘模式决定目标速度来源
 * @param   chassis_control_state  底盘控制结构体指针
 * @note    CHASSIS_OPERATING_MODE_HOLD_POSITION：目标速度强制为 0；
 *          NO_FOLLOW / FOLLOW：调用 Chassis_CalculateSpeedSetpoint 由遥控器/键盘解算；
 *          ZERO_FORCE：无需设置目标（控制阶段直接清零输出）。
 */
static void Chassis_SetSpeedSetpoint(chassis_control_t *chassis_control_state)
{
    if(chassis_control_state == NULL)
    {
        return;
   
    }

    if(chassis_control_state->operating_mode == CHASSIS_OPERATING_MODE_HOLD_POSITION)
    {
        chassis_control_state->forward_speed_setpoint_revolution_per_minute = 0;
        chassis_control_state->leftward_speed_setpoint_revolution_per_minute = 0;
        chassis_control_state->rotation_speed_setpoint_radian_per_second = 0;
    }
    else if(chassis_control_state->operating_mode == CHASSIS_OPERATING_MODE_INDEPENDENT||chassis_control_state->operating_mode == CHASSIS_OPERATING_MODE_FOLLOW_GIMBAL)
    {
        Chassis_CalculateSpeedSetpoint(chassis_control_state);
    }

}

/**
 * @brief   底盘速度环 PID 控制计算（含功率限制与零力矩处理）
 * @param   chassis_control_state  底盘控制结构体指针
 * @note    1) 根据四轮平均速度动态调整 PID 的 ki 与积分限幅 max_iout（变增益抗饱和）；
 *          2) 对每轮做速度 PID：输入 = 目标转速 rotor_speed_setpoint_revolution_per_minute（叠加云台跟随前馈项），
 *             反馈 = 实际转速 rotor_speed_revolution_per_minute，输出 = 给电机电流 commanded_torque_current_count（单位 A）；
 *          3) 由解算电流预估实际功率，做功率限制（Chassis_LimitPower）；
 *          4) ZERO_FORCE 模式下最终把所有电机输出电流清零。
 */
static void Chassis_CalculateSpeedControlOutput(chassis_control_t *chassis_control_state)
{
    if(chassis_control_state == NULL)
    {
        return;
    }
    fp32 average_rotor_speed_revolution_per_minute=fabs(chassis_control_state->motor_states[0].rotor_speed_revolution_per_minute+
			chassis_control_state->motor_states[1].rotor_speed_revolution_per_minute+
				chassis_control_state->motor_states[2].rotor_speed_revolution_per_minute+
					chassis_control_state->motor_states[3].rotor_speed_revolution_per_minute)/4;
    for (uint8_t i = 0; i < 4; i++)
    {
        chassis_control_state->speed_loop_pid[i].ki=0.20+1.0*exp(-0.001*fabs(average_rotor_speed_revolution_per_minute)-1.8);
        chassis_control_state->speed_loop_pid[i].max_iout=fmin((440 + 6 * fabs(average_rotor_speed_revolution_per_minute / 100)),1000);
        chassis_control_state->motor_states[i].commanded_torque_current_count=pid_calc(&chassis_control_state->speed_loop_pid[i],
					chassis_control_state->motor_states[i].rotor_speed_revolution_per_minute,
						chassis_control_state->motor_states[i].rotor_speed_setpoint_revolution_per_minute+chassis_control_state->chassis_angular_velocity_radian_per_second*chassis_follow_gimbal_forward_proportional_gain);
    }
    for(uint8_t i=0;i<4;i++)
    {
			chassis_control_state->motor_states[i].allocated_torque_current_ampere=chassis_control_state->motor_states[i].commanded_torque_current_count*current_count_to_ampere;
			chassis_control_state->motor_states[i].rotor_angular_velocity_radian_per_second = chassis_control_state->motor_states[i].motor_measurement_pointer->rotor_speed_revolution_per_minute*revolution_per_minute_to_radian_per_second;
			chassis_control_state->motor_states[i].demanded_mechanical_power_watt=(chassis_power_fit_coefficient_0+
				(chassis_power_fit_coefficient_1*chassis_control_state->motor_states[i].allocated_torque_current_ampere)+
					(chassis_power_fit_coefficient_2*chassis_control_state->motor_states[i].rotor_angular_velocity_radian_per_second)+
						(chassis_power_fit_coefficient_3*chassis_control_state->motor_states[i].rotor_angular_velocity_radian_per_second*chassis_control_state->motor_states[i].allocated_torque_current_ampere)+
							(chassis_power_fit_coefficient_4*chassis_control_state->motor_states[i].allocated_torque_current_ampere*chassis_control_state->motor_states[i].allocated_torque_current_ampere)+
								(chassis_power_fit_coefficient_5*chassis_control_state->motor_states[i].rotor_angular_velocity_radian_per_second*chassis_control_state->motor_states[i].rotor_angular_velocity_radian_per_second));
		}
	chassis_control_state->demanded_mechanical_power_watt=chassis_control_state->motor_states[0].demanded_mechanical_power_watt+
		chassis_control_state->motor_states[1].demanded_mechanical_power_watt+
			chassis_control_state->motor_states[2].demanded_mechanical_power_watt+
				chassis_control_state->motor_states[3].demanded_mechanical_power_watt;
	Chassis_ApplyPowerLimit(chassis_control_state);
	if(chassis_control_state->operating_mode == CHASSIS_OPERATING_MODE_ZERO_FORCE)
    {
       for(uint8_t i=0;i<4;i++)
       {
           chassis_control_state->motor_states[i].commanded_torque_current_count = 0;
		   
       }	   
    }
}

/**
 * @brief   由遥控器/键盘原始输入解算底盘目标速度（含跟随、斜坡、滤波）
 * @param   chassis_control_state  底盘控制结构体指针
 * @note    流程：
 *          1. 按 keyboard_control_enabled 选择键盘或遥控器作为 forward_speed_revolution_per_minute/leftward_speed_revolution_per_minute 平移来源；
 *          2. 若陀螺开启（TOP_SPIN_MODE_ON）按方向设定自旋 rotation_speed_radian_per_second；陀螺关闭且跟随云台时用
 *             跟随 PID 把 rotation_speed_radian_per_second 对齐云台（含掉头切换的等待/平滑处理）；
 *          3. 得到 forward_speed_setpoint_revolution_per_minute/leftward_speed_setpoint_revolution_per_minute/rotation_speed_setpoint_radian_per_second（速度单位：rpm）；
 *          4. 云台跟随模式下把平移速度旋转到云台系（坐标系旋转变换）；
 *          5. rotation_speed_radian_per_second 经一阶低通（First_Lpf_Calc），forward_speed_revolution_per_minute/leftward_speed_revolution_per_minute 经斜坡规划（slope_plan）限速；
 *          6. 最后反解四个麦轮目标转速（Chassis_ConvertVectorToWheelSpeed）。
 */
static void Chassis_CalculateSpeedSetpoint(chassis_control_t *chassis_control_state)
{
    fp32 forward_speed_revolution_per_minute=0;
	fp32 leftward_speed_revolution_per_minute=0;
    if(chassis_control_state == NULL)
    {
        return;
    }
	 if(chassis_control_state->gimbal_command_data_pointer->keyboard_control_enabled==1)//进入比赛模式
     {
	      Chassis_ConvertKeyboardToSpeed(chassis_control_state);  //键盘控制
	 }
	 else
	 {
	 	  Chassis_ConvertRemoteControlToSpeed(chassis_control_state);  //遥控器通道计算
	 }
   
    if(chassis_control_state->top_spin_mode == TOP_SPIN_MODE_ON)
    {
        if(chassis_control_state->super_capacitor_mode == SUPER_CAPACITOR_MODE_ON)
        {
            chassis_control_state->rotation_speed_normal_radian_per_second=chassis_rotation_speed_maximum_super_capacitor_on_revolution_per_minute;
        }
        else if(chassis_control_state->super_capacitor_mode == SUPER_CAPACITOR_MODE_OFF)
        {
            chassis_control_state->rotation_speed_normal_radian_per_second=chassis_rotation_speed_maximum_super_capacitor_off_revolution_per_minute;
        }
        if(chassis_control_state->top_spin_direction == CHASSIS_TOP_SPIN_DIRECTION_COUNTERCLOCKWISE)
        {
            chassis_control_state->rotation_speed_normal_radian_per_second = chassis_control_state->rotation_speed_normal_radian_per_second;
        }
        else if(chassis_control_state->top_spin_direction == CHASSIS_TOP_SPIN_DIRECTION_CLOCKWISE)
        {
            chassis_control_state->rotation_speed_normal_radian_per_second = -chassis_control_state->rotation_speed_normal_radian_per_second;
        }
    }
    else if(chassis_control_state->top_spin_mode == TOP_SPIN_MODE_OFF)
    {   
		if(chassis_control_state->operating_mode==CHASSIS_OPERATING_MODE_FOLLOW_GIMBAL)
        {
			if(chassis_control_state->follow_gimbal_transition.state!=chassis_control_state->follow_gimbal_transition.previous_state)
			{
				chassis_control_state->follow_gimbal_transition.chassis_response_pending=1;
			}
			if(chassis_control_state->follow_gimbal_transition.chassis_response_pending==1)
			{   chassis_control_state->rotation_speed_normal_radian_per_second = 0;
				if(chassis_control_state->follow_gimbal_transition.gimbal_turnaround_elapsed_time_millisecond<=gimbal_turnaround_wait_time_millisecond/2)
				{
					chassis_control_state->follow_gimbal_transition.gimbal_turnaround_elapsed_time_millisecond+=1;
					
				}
				else
				{
					chassis_control_state->follow_gimbal_transition.chassis_response_pending=0;
					chassis_control_state->follow_gimbal_transition.gimbal_turnaround_elapsed_time_millisecond=0;
				}
			}
			else
			{
				chassis_control_state->rotation_speed_normal_radian_per_second=pid_calc(&chassis_control_state->follow_gimbal_angle_loop_pid,chassis_control_state->gimbal_relative_angle_radian,chassis_control_state->gimbal_relative_angle_setpoint_radian);
			}	
			
        }
        else if(CHASSIS_OPERATING_MODE_INDEPENDENT)
        {
            chassis_control_state->rotation_speed_normal_radian_per_second = 0;
        } 
    }
    chassis_control_state->forward_speed_setpoint_revolution_per_minute=chassis_control_state->forward_speed_normal_revolution_per_minute+chassis_control_state->forward_speed_increment_revolution_per_minute;
    chassis_control_state->leftward_speed_setpoint_revolution_per_minute=chassis_control_state->leftward_speed_normal_revolution_per_minute+chassis_control_state->leftward_speed_increment_revolution_per_minute;
    chassis_control_state->rotation_speed_setpoint_radian_per_second=chassis_control_state->rotation_speed_normal_radian_per_second+chassis_control_state->rotation_speed_increment_radian_per_second;
	if(chassis_control_state->follow_gimbal_transition.state==CHASSIS_FOLLOW_GIMBAL_STATE_TURNAROUND)
	{
		chassis_control_state->forward_speed_setpoint_revolution_per_minute=-chassis_control_state->forward_speed_setpoint_revolution_per_minute;
		chassis_control_state->leftward_speed_setpoint_revolution_per_minute=-chassis_control_state->leftward_speed_setpoint_revolution_per_minute;
	}
    if(chassis_control_state->operating_mode==CHASSIS_OPERATING_MODE_FOLLOW_GIMBAL)
    {  
		    gimbal_relative_angle_cosine=cos(chassis_control_state->gimbal_relative_angle_radian);
			gimbal_relative_angle_sine=sin(chassis_control_state->gimbal_relative_angle_radian);
			forward_speed_revolution_per_minute=chassis_control_state->forward_speed_setpoint_revolution_per_minute;
		    leftward_speed_revolution_per_minute=chassis_control_state->leftward_speed_setpoint_revolution_per_minute;
			chassis_control_state->forward_speed_setpoint_revolution_per_minute = forward_speed_revolution_per_minute*gimbal_relative_angle_cosine + leftward_speed_revolution_per_minute*gimbal_relative_angle_sine;//跟随相对平滑处理
			chassis_control_state->leftward_speed_setpoint_revolution_per_minute = -forward_speed_revolution_per_minute*gimbal_relative_angle_sine + leftward_speed_revolution_per_minute*gimbal_relative_angle_cosine;
	}
	if(chassis_control_state->top_spin_mode==TOP_SPIN_MODE_OFF)
	{
		if(fabsf(chassis_control_state->rotation_speed_setpoint_radian_per_second)>800)
		{
			chassis_control_state->forward_speed_setpoint_revolution_per_minute= chassis_control_state->forward_speed_setpoint_revolution_per_minute*0.8;
		}
	}
	chassis_control_state->rotation_speed_setpoint_radian_per_second=First_Lpf_Calc(&chassis_control_state->rotation_speed_command_low_pass_filter,chassis_control_state->rotation_speed_setpoint_radian_per_second);
    chassis_control_state->forward_speed_setpoint_revolution_per_minute=slope_plan(&chassis_control_state->forward_speed_slope_plan);
    chassis_control_state->leftward_speed_setpoint_revolution_per_minute=slope_plan(&chassis_control_state->leftward_speed_slope_plan);
    //chassis_control_state->rotation_speed_setpoint_radian_per_second=slope_plan(&chassis_control_state->rotation_speed_slope_plan);
    Chassis_ConvertVectorToWheelSpeed(chassis_control_state);
}

/**
 * @brief   遥控器平移控制：把摇杆通道映射为 forward_speed_revolution_per_minute/leftward_speed_revolution_per_minute 目标速度
 * @param   chassis_control_state  底盘控制结构体指针
 * @note    forward_speed_revolution_per_minute = forward_channel_value * remote_control_forward_sensitivity_count，leftward_speed_revolution_per_minute = leftward_channel_value * remote_control_leftward_sensitivity_count
 *          （remote_control_forward_sensitivity_count / remote_control_leftward_sensitivity_count 为灵敏度标定系数，输出量纲为 rpm）。
 */
static void Chassis_ConvertRemoteControlToSpeed(chassis_control_t *chassis_control_state)
{
	  
		chassis_control_state->forward_speed_normal_revolution_per_minute=chassis_control_state->gimbal_command_data_pointer->forward_channel_value*remote_control_forward_sensitivity_count;
		chassis_control_state->leftward_speed_normal_revolution_per_minute=chassis_control_state->gimbal_command_data_pointer->leftward_channel_value*remote_control_leftward_sensitivity_count;
}

/**
 * @brief   键盘平移控制：按裁判系统功率上限查表得到最大速度，再按按键给出 forward_speed_revolution_per_minute/leftward_speed_revolution_per_minute
 * @param   chassis_control_state  底盘控制结构体指针
 * @note    键盘仅在比赛（keyboard_control_enabled==1）模式启用；WASD 映射前后/左右平移，
 *          开启超电（SUPER_CAPACITOR_MODE_ON）时最大速度提升到 9000rpm。
 */
static void Chassis_ConvertKeyboardToSpeed(chassis_control_t *chassis_control_state)
{
    if(chassis_control_state == NULL) 
    {
        return; 
    }
    if(chassis_control_state->referee_system_data_pointer->robot_state.chassis_power_limit==50)
	{
		chassis_control_state->forward_speed_maximum_revolution_per_minute=4000;
	}
	else if(chassis_control_state->referee_system_data_pointer->robot_state.chassis_power_limit==60)
	{
		chassis_control_state->forward_speed_maximum_revolution_per_minute=5400;
	}
	else if(chassis_control_state->referee_system_data_pointer->robot_state.chassis_power_limit==70)
	{
		chassis_control_state->forward_speed_maximum_revolution_per_minute=6000;
	}
	else if(chassis_control_state->referee_system_data_pointer->robot_state.chassis_power_limit==80)
	{
		chassis_control_state->forward_speed_maximum_revolution_per_minute=6200;
	}
	else if(chassis_control_state->referee_system_data_pointer->robot_state.chassis_power_limit==90)
	{
		chassis_control_state->forward_speed_maximum_revolution_per_minute=6800;
	}
	else if(chassis_control_state->referee_system_data_pointer->robot_state.chassis_power_limit==100)
	{
		chassis_control_state->forward_speed_maximum_revolution_per_minute=7000;
	}
	else if(chassis_control_state->referee_system_data_pointer->robot_state.chassis_power_limit==110)
	{
		chassis_control_state->forward_speed_maximum_revolution_per_minute=8500;
	}
	else if(chassis_control_state->referee_system_data_pointer->robot_state.chassis_power_limit==120)
	{
		chassis_control_state->forward_speed_maximum_revolution_per_minute=8000;
	}
	else
	{
		chassis_control_state->forward_speed_maximum_revolution_per_minute=4200;
	}
	if(chassis_control_state->super_capacitor_mode==SUPER_CAPACITOR_MODE_ON)
	{
		chassis_control_state->forward_speed_maximum_revolution_per_minute=9000;
	}
    if(chassis_control_state->gimbal_command_data_pointer->forward_key_value&keyboard_forward_key_mask)
    {
        chassis_control_state->forward_speed_normal_revolution_per_minute=-chassis_control_state->forward_speed_maximum_revolution_per_minute;
    }
    else if(chassis_control_state->gimbal_command_data_pointer->forward_key_value&keyboard_backward_key_mask)
    {
        chassis_control_state->forward_speed_normal_revolution_per_minute=chassis_control_state->forward_speed_maximum_revolution_per_minute;
    }
    else
    {
        chassis_control_state->forward_speed_normal_revolution_per_minute=0;  
    }
    if(chassis_control_state->gimbal_command_data_pointer->leftward_key_value&keyboard_rightward_key_mask)
    {
        chassis_control_state->leftward_speed_normal_revolution_per_minute=chassis_leftward_speed_maximum_keyboard_revolution_per_minute;
    }
    else if(chassis_control_state->gimbal_command_data_pointer->leftward_key_value&keyboard_leftward_key_mask)
    {
        chassis_control_state->leftward_speed_normal_revolution_per_minute=-chassis_leftward_speed_maximum_keyboard_revolution_per_minute;
    }
    else
    {
        chassis_control_state->leftward_speed_normal_revolution_per_minute=0;
    }
}
/**
 * @brief   底盘功率限制：在功率预算内反解每轮允许的最大电流
 * @param   chassis_control_state  底盘控制结构体指针
 * @note    核心思路：电机功率 P = chassis_power_fit_coefficient_0+chassis_power_fit_coefficient_1·I+chassis_power_fit_coefficient_2·ω+chassis_power_fit_coefficient_3·ω·I+chassis_power_fit_coefficient_4·I2+chassis_power_fit_coefficient_5·ω2（拟合式），
 *          其中 I 为待求电流（A）、ω 为当前角速度（rad/s）。给定“本轮可用功率
 *          上限”，即解一元二次方程 a·I2+b·I+c=0 求 I，再换算回给电机的
 *          commanded_torque_current_count = I / current_count_to_ampere（current_count_to_ampere 为电流比例系数）。
 *          可用功率 = 裁判系统功率上限 + 超电增益 + 缓冲能量 PID（缓冲闭环）；
 *          若实际需求功率 <= 上限则不加限制。
 */
static void Chassis_ApplyPowerLimit(chassis_control_t *chassis_control_state)
{
    if(chassis_control_state == NULL)
    {
        return;
    }
	static fp32 referee_power_limit_watt =55.0f;  //p_max  来自RM
	static fp32 referee_power_buffer_watt = 60.0f; //p_buff 来自RM
	static fp32 user_power_demand_watt = 0.0f;      //当前电机所需消耗
	static fp32 power_maximum_watt = 0.0f;       //最大可放功率
	static fp32 super_capacitor_power_gain=0.0f;         //开启超电功率
	static fp32 referee_power_buffer_limit_watt=0.0f;    //缓冲上限
	static fp32 referee_power_buffer_used_watt=0.0f;    //用户使用缓冲功率
	static fp32 motor_current_solution_1_ampere=0.0f;         //电机的电流解1
	static fp32 motor_current_solution_2_ampere=0.0f;              //电机的电流解2
	static fp32 a=0.0f;							   //方程a项
	static fp32 b=0.0f;							   //方程b项	
	static fp32 c=0.0f;							   //方程c项	
	static fp32 square_root_result=0.0f;                        //中间平方根结果
	static fp32 attenuation_numerator=0.0f;                     //衰减分子
	static fp32 attenuation_denominator=0.0f;                   //衰减分母
	if(chassis_control_state->super_capacitor_mode==SUPER_CAPACITOR_MODE_ON&&chassis_control_state->super_capacitor_measurement_pointer->voltage_volt>=19)
	{
		super_capacitor_power_gain=200;
	}
	else
	{
		super_capacitor_power_gain=0;
	}
    referee_power_limit_watt=chassis_control_state->referee_system_data_pointer->robot_state.chassis_power_limit;
    referee_power_buffer_watt=chassis_control_state->referee_system_data_pointer->power_heat_data_t.buffer_energy;
	referee_power_buffer_limit_watt=chassis_control_state->power_buffer_limit_watt;
    user_power_demand_watt=chassis_control_state->demanded_mechanical_power_watt;
	referee_power_buffer_used_watt=referee_power_buffer_watt-chassis_control_state->power_buffer_used_watt;
	power_maximum_watt=referee_power_limit_watt+super_capacitor_power_gain+pid_calc(&chassis_control_state->power_buffer_loop_pid,referee_power_buffer_watt,20);
	if(user_power_demand_watt<=power_maximum_watt)
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
			chassis_control_state->motor_states[i].mechanical_power_limit_watt=chassis_control_state->motor_states[i].demanded_mechanical_power_watt*power_maximum_watt/user_power_demand_watt;//进行功率动态分配以及衰减
		}
	}
	for(uint8_t i=0;i<4;i++)
	{
		a=chassis_power_fit_coefficient_4;
		b=chassis_power_fit_coefficient_1+chassis_power_fit_coefficient_3*chassis_control_state->motor_states[i].rotor_angular_velocity_radian_per_second;
		c=chassis_power_fit_coefficient_0+chassis_power_fit_coefficient_2*chassis_control_state->motor_states[i].rotor_angular_velocity_radian_per_second+chassis_power_fit_coefficient_5*chassis_control_state->motor_states[i].rotor_angular_velocity_radian_per_second*chassis_control_state->motor_states[i].rotor_angular_velocity_radian_per_second-chassis_control_state->motor_states[i].mechanical_power_limit_watt;
		if(b*b-4*a*c<0)
		{
			motor_current_solution_1_ampere=motor_current_solution_2_ampere=0;
		}
		else
		{
			if(b*b-4*a*c==0)
			{
				arm_sqrt_f32(b*b-4*a*c,&square_root_result);
				motor_current_solution_1_ampere=motor_current_solution_2_ampere=(-b+square_root_result)/(2*a);
				chassis_control_state->motor_states[i].commanded_torque_current_count=motor_current_solution_1_ampere/current_count_to_ampere;
			}
			else
			{
				arm_sqrt_f32(b*b-4*a*c,&square_root_result);
				motor_current_solution_1_ampere=(-b+square_root_result)/(2*a);
				motor_current_solution_2_ampere=(-b-square_root_result)/(2*a);
				if(fabsf(motor_current_solution_1_ampere-chassis_control_state->motor_states[i].allocated_torque_current_ampere)>fabsf(motor_current_solution_2_ampere-chassis_control_state->motor_states[i].allocated_torque_current_ampere))
				{
					chassis_control_state->motor_states[i].commanded_torque_current_count=motor_current_solution_2_ampere/current_count_to_ampere;
				}
				else
				{
					chassis_control_state->motor_states[i].commanded_torque_current_count=motor_current_solution_1_ampere/current_count_to_ampere;
				}
			}			
		}

	}
}

/**
 * @brief   更新功率缓冲能量的可用上限档位
 * @param   chassis_control_state  底盘控制结构体指针
 * @note    根据裁判系统 buffer_energy：>70 时按“全增益”档给缓冲功率，
 *          <=60 时按“无增益”档；60~70 之间保持上一档（滞回，防抖）。
 */
static void Chassis_UpdatePowerBuffer(chassis_control_t *chassis_control_state)
{
    if(chassis_control_state == NULL)
    {
        return;
    }
    if(chassis_control_state->referee_system_data_pointer->power_heat_data_t.buffer_energy>70)
    {
        chassis_control_state->power_buffer_limit_watt=chassis_power_buffer_limit_with_gain_watt;
        chassis_control_state->power_buffer_used_watt=chassis_power_buffer_usage_maximum_with_gain_watt;
    }
    else if(chassis_control_state->referee_system_data_pointer->power_heat_data_t.buffer_energy<=60)
    {
        chassis_control_state->power_buffer_limit_watt=chassis_power_buffer_limit_without_gain_watt;
        chassis_control_state->power_buffer_used_watt=chassis_power_buffer_usage_maximum_without_gain_watt;
    }

}

/**
 * @brief   麦轮运动学逆解：把底盘 forward_speed_revolution_per_minute/leftward_speed_revolution_per_minute/rotation_speed_radian_per_second 目标反解为四个麦轮的目标转速
 * @param   chassis_control_state  底盘控制结构体指针
 * @note    麦轮布置：四个轮呈 X 型，正负号组合体现各轮对平移/自旋的贡献；
 *          rotation_speed_radian_per_second 需乘电机到底盘中心的距离系数 motor_distance_to_chassis_center_meter 统一量纲。
 *          轮0 = -forward_speed_revolution_per_minute+leftward_speed_revolution_per_minute+rotation_speed_radian_per_second；轮1 = +forward_speed_revolution_per_minute+leftward_speed_revolution_per_minute+rotation_speed_radian_per_second；轮2 = +forward_speed_revolution_per_minute-leftward_speed_revolution_per_minute+rotation_speed_radian_per_second；轮3 = -forward_speed_revolution_per_minute-leftward_speed_revolution_per_minute+rotation_speed_radian_per_second。
 */
static void Chassis_ConvertVectorToWheelSpeed(chassis_control_t *chassis_control_state)
{
    static fp32 forward_speed_setpoint_revolution_per_minute=0.0f;
    static fp32 leftward_speed_setpoint_revolution_per_minute=0.0f;
    static fp32 rotation_speed_setpoint_radian_per_second=0.0f;
    if(chassis_control_state == NULL)
    {
        return;
    }
    forward_speed_setpoint_revolution_per_minute=chassis_control_state->forward_speed_setpoint_revolution_per_minute;
    leftward_speed_setpoint_revolution_per_minute=chassis_control_state->leftward_speed_setpoint_revolution_per_minute;
    rotation_speed_setpoint_radian_per_second=chassis_control_state->rotation_speed_setpoint_radian_per_second*motor_distance_to_chassis_center_meter;
    chassis_control_state->motor_states[0].rotor_speed_setpoint_revolution_per_minute=-forward_speed_setpoint_revolution_per_minute+leftward_speed_setpoint_revolution_per_minute+rotation_speed_setpoint_radian_per_second;
    chassis_control_state->motor_states[1].rotor_speed_setpoint_revolution_per_minute=forward_speed_setpoint_revolution_per_minute+leftward_speed_setpoint_revolution_per_minute+rotation_speed_setpoint_radian_per_second;
    chassis_control_state->motor_states[2].rotor_speed_setpoint_revolution_per_minute=forward_speed_setpoint_revolution_per_minute-leftward_speed_setpoint_revolution_per_minute+rotation_speed_setpoint_radian_per_second;
    chassis_control_state->motor_states[3].rotor_speed_setpoint_revolution_per_minute=-forward_speed_setpoint_revolution_per_minute-leftward_speed_setpoint_revolution_per_minute+rotation_speed_setpoint_radian_per_second;
}

/**
 * @brief   电机编码器原始值 -> 相对角度（弧度）
 * @param   encoder_count                  当前编码器原始值（0..motor_encoder_count_per_revolution）
 * @param   encoder_zero_count   零位编码值（安装零点）
 * @return  相对零位的角度，单位 rad（弧度）
 * @note    处理编码器回绕：差值超过半圈则加减一整圈，保证角度连续无跳变。
 */
static fp32 Chassis_ConvertEncoderCountToAngleRadian(uint16_t encoder_count,uint16_t encoder_zero_count)
{
    int32_t relative_encoder_count=encoder_count-encoder_zero_count;
    if(relative_encoder_count>motor_encoder_half_count_range)
    {
        relative_encoder_count-=motor_encoder_count_per_revolution;
    }
    else if(relative_encoder_count<-motor_encoder_half_count_range)
    {
        relative_encoder_count+=motor_encoder_count_per_revolution;
    }
    return relative_encoder_count*motor_encoder_count_to_radian;
}
