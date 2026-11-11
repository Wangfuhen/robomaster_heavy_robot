/**
 * @file gimbal_task.c
 * @brief 云台控制任务
 * @note 此文件实现了云台系统的完整控制逻辑，包括初始化、模式切换、反馈更新、控制计算等功能
 *       支持多种控制模式：无力模式、编码器模式、陀螺仪模式、自动瞄准模式
 */

#include "gimbal_task.h"
#include "shoot_task.h"
#include "string.h"

// 遥控器死区控制宏
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
}

// 静态函数声明
static void Gimbal_Init(gimbal_control_t *hero_gimbal_init);          // 云台初始化
static void Gimbal_Set_Mode(gimbal_control_t *hero_gimbal_set_mode);  // 遥控器设置模式
static void Gimbal_Mode_Change_Feedback_Change(gimbal_control_t *hero_gimbal_mode_change_feedback_change);  // 模式切换反馈
static void Gimbal_Feedback_Updata(gimbal_control_t *hero_gimbal_feedback_updata);    // 反馈更新
static void Gimbal_Mode_Set_Control(gimbal_control_t *hero_gimbal_mode_set_control);  // 模式切换控制
static void Gimbal_Control_Cacl(gimbal_control_t *hero_gimbal_control_cacl);          // 控制计算

// 子函数声明
static fp32 Motor_Ecd_To_Angle_yaw(uint16_t ecd,uint16_t ecd_zero_angle_set);  // YAW轴编码器转角度
static fp32 Motor_Ecd_To_Angle_pitch(uint16_t ecd,uint16_t ecd_zero_angle_set);  // PITCH轴编码器转角度
static void Gimbal_Zero_Force_Control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);  // 无力控制
static void Gimbal_Relative_Angle_Control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);  // 相对角度控制
static void Gimbal_Absolute_Angle_Control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);  // 绝对角度控制
static void Gimbal_Auto_Control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);  // 自动控制

// 全局变量定义
extern gimbal_control_t mini_gimbal;  // 云台控制结构体实例
extern hero_shoot_t hero_shoot;  // 射击任务外部变量
uint8_t auto_mode_flag_qian = 0;  // 自动模式前一个标志
uint8_t auto_mode_flag = 0;  // 自动模式标志

/**
 * @brief 云台控制任务主函数
 * @param argument FreeRTOS任务参数
 * @note 此函数是云台控制的主要循环，周期性执行云台控制的所有步骤
 */
void Gimbal_Task(void const * argument)
{
    vTaskDelay(HERO_GIMBAL_INIT_TIME);  // 延时等待系统初始化完成
    
    Gimbal_Init(&mini_gimbal);  // 云台初始化
    
    while(1)
    {
        // 云台控制主循环
        Gimbal_Set_Mode(&mini_gimbal);  // 设置云台控制模式
        Gimbal_Mode_Change_Feedback_Change(&mini_gimbal);  // 模式切换反馈处理
        Gimbal_Feedback_Updata(&mini_gimbal);  // 更新反馈数据
        Gimbal_Mode_Set_Control(&mini_gimbal);  // 根据模式设置控制量
        Gimbal_Control_Cacl(&mini_gimbal);  // 控制计算
        
        // 发送控制指令到电机
        CAN_cmd_gimbal_yaw(mini_gimbal.yaw_gimbal_motor.give_current);
        CAN_motor_send_data_pitch(mini_gimbal.pitch_gimbal_motor.give_current);
        
        vTaskDelay(GIMBAL_LOOP_TIME);  // 控制周期延时
    }
}

/**
 * @brief 云台初始化函数
 * @param hero_gimbal_init 云台控制结构体指针
 * @note 初始化云台的所有参数、PID控制器、获取传感器数据指针等
 */
static void Gimbal_Init(gimbal_control_t *hero_gimbal_init)
{
    // 获取电机测量数据指针
	hero_gimbal_init->yaw_gimbal_motor.gimbal_motor_measure = get_motor_gimbal_yaw_measure_point();
	hero_gimbal_init->pitch_gimbal_motor.gimbal_motor_measure = get_motor_gimbal_pitch_measure_point();
    
    // 获取IMU数据指针
	hero_gimbal_init->gimbal_INS_angle_point = get_INS_angle_point();
	hero_gimbal_init->gimbal_INS_gyro_point = get_gyro_data_point();
    
    // 获取遥控器数据指针
	hero_gimbal_init->gimbal_dt7_remote_control = get_dt7_remote_control_point();
    
    // 初始化云台模式
	hero_gimbal_init->gimbal_mode = GIMBAL_ZERO_FORCE;  // 默认无力模式
    hero_gimbal_init->yaw_gimbal_motor.gimbal_motor_mode = MOTOR_ZERO_FORCE;
    hero_gimbal_init->yaw_gimbal_motor.gimbal_motor_mode_last = MOTOR_ZERO_FORCE;
    hero_gimbal_init->pitch_gimbal_motor.gimbal_motor_mode = MOTOR_ZERO_FORCE;
    hero_gimbal_init->pitch_gimbal_motor.gimbal_motor_mode_last = MOTOR_ZERO_FORCE;
    
    // 初始化云台方向状态
	hero_gimbal_init->gimbal_angle_change.gimbal_change = GIMABL_NOMAL_FORWARD;  // 默认前向
	hero_gimbal_init->gimbal_angle_change.gimbal_change_last = GIMABL_NOMAL_FORWARD;
    
    // 初始化Yaw轴零位
	hero_gimbal_init->yaw_gimbal_motor.relative_ecd_aero = YAW_ZERO_ANGLE_SET;
    
    // 获取PC数据指针
	hero_gimbal_init->hero_pc = get_crg_pc_point();
    
    // 初始化角度计算相关变量
	hero_gimbal_init->yaw_gimbal_motor.feed_angle_cacl.in = 0;
	hero_gimbal_init->yaw_gimbal_motor.feed_angle_cacl.last_in = 0;
    
    // 初始化YAW轴相对编码器PID控制器
	fp32 yaw_speed_relative[3] = {YAW_ANGLE_KP_RELATIVE, YAW_ANGLE_KI_RELATIVE, YAW_ANGLE_KD_RELATIVE};
    fp32 yaw_angle_relative[3] = {YAW_SPEED_KP_RELATIVE, YAW_SPEED_KI_RELATIVE, YAW_SPEED_KD_RELATIVE};
	pid_init(&hero_gimbal_init->yaw_gimbal_motor.pid_gimbal_angle_relative, PID_POSITION, yaw_angle_relative, YAW_ANGLE_MAX_RELATIVE, YAW_ANGLE_I_MAX_RELATIVE);
	pid_init(&hero_gimbal_init->yaw_gimbal_motor.pid_gimbal_speed_relative, PID_POSITION, yaw_speed_relative, YAW_SPEED_MAX_RELATIVE, YAW_SPEED_I_MAX_RELATIVE);
    
    // 初始化YAW轴陀螺仪PID控制器
	fp32 yaw_speed[3] = {YAW_SPEED_KP, YAW_SPEED_KI, YAW_SPEED_KD};
    fp32 yaw_angle[3] = {YAW_ANGLE_KP, YAW_ANGLE_KI, YAW_ANGLE_KD};
	pid_init(&hero_gimbal_init->yaw_gimbal_motor.pid_gimbal_angle, PID_POSITION, yaw_angle, YAW_ANGLE_MAX, YAW_ANGLE_I_MAX);
	pid_init(&hero_gimbal_init->yaw_gimbal_motor.pid_gimbal_speed, PID_POSITION, yaw_speed, YAW_SPEED_MAX, YAW_SPEED_I_MAX);
    
    // 初始化YAW轴自瞄PID控制器
    fp32 yaw_speed_ac[3] = {YAW_SPEED_KP_AC, YAW_SPEED_KI_AC, YAW_SPEED_KD_AC};
    fp32 yaw_angle_ac[3] = {YAW_ANGLE_KP_AC, YAW_ANGLE_KI_AC, YAW_ANGLE_KD_AC};
    pid_init(&hero_gimbal_init->yaw_gimbal_motor.pid_gimbal_angle_ac, PID_POSITION, yaw_angle_ac, YAW_ANGLE_AC_MAX, YAW_ANGLE_I_AC_MAX);
	pid_init(&hero_gimbal_init->yaw_gimbal_motor.pid_gimbal_speed_ac, PID_POSITION, yaw_speed_ac, YAW_SPEED_AC_MAX, YAW_SPEED_I_AC_MAX);
    
    // 初始化PITCH轴LQR控制器
    fp32 pitch_speed[4] = {PITCH_SPEED_KP, PITCH_SPEED_KI, PITCH_SPEED_I_MAX, PITCH_SPEED_MAX};
    fp32 pitch_angle[4] = {PITCH_ANGLE_KP, PITCH_ANGLE_KI, PITCH_ANGLE_I_MAX, PITCH_ANGLE_MAX};
    fp32 pitch_current[4] = {PITCH_CURRENT_KP, 0, 0, PITCH_CURRENT_MAX};
    lqr_init(&hero_gimbal_init->pitch_gimbal_motor.lqr, pitch_speed, pitch_angle, pitch_current, 30000);
    
    // 初始化PITCH轴自瞄LQR控制器
    fp32 pitch_speed_ac[4] = {PITCH_SPEED_KP_AC, PITCH_SPEED_KI_AC, PITCH_SPEED_I_MAX, PITCH_SPEED_MAX};
    fp32 pitch_angle_ac[4] = {PITCH_ANGLE_KP_AC, PITCH_ANGLE_KI_AC, PITCH_ANGLE_I_MAX, PITCH_ANGLE_MAX};
    fp32 pitch_current_ac[4] = {PITCH_CURRENT_KP_AC, 0, 0, PITCH_CURRENT_MAX};
    lqr_init(&hero_gimbal_init->pitch_gimbal_motor.lqr_ac, pitch_speed_ac, pitch_angle_ac, pitch_current_ac, 30000);
    
    // 初始化PITCH轴串级PID控制器
	static const fp32 Pitch_ABSOLUTE_speed_pid[3] = {PITCH_ABSOLUTE_SPEED_PID_KP, PITCH_ABSOLUTE_SPEED_PID_KI, PITCH_ABSOLUTE_SPEED_PID_KD};
	static const fp32 Pitch_ABSOLUTE_angle_pid[3] = {PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD};
	pid_init(&hero_gimbal_init->pitch_gimbal_motor.pid_gimbal_angle, PID_POSITION, Pitch_ABSOLUTE_angle_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT);
	pid_init(&hero_gimbal_init->pitch_gimbal_motor.pid_gimbal_speed, PID_POSITION, Pitch_ABSOLUTE_speed_pid, PITCH_ABSOLUTE_SPEED_PID_MAX_OUT, PITCH_ABSOLUTE_SPEED_PID_MAX_IOUT);
    
    // 初始化PITCH轴自瞄串级PID控制器
	static const fp32 Pitch_ABSOLUTE_speed_pid_ac[3] = {PITCH_ABSOLUTE_SPEED_PID_KP_AC, PITCH_ABSOLUTE_SPEED_PID_KI_AC, PITCH_ABSOLUTE_SPEED_PID_KD_AC};
	static const fp32 Pitch_ABSOLUTE_angle_pid_ac[3] = {PITCH_GYRO_ABSOLUTE_PID_KP_AC, PITCH_GYRO_ABSOLUTE_PID_KI_AC, PITCH_GYRO_ABSOLUTE_PID_KD_AC};
	pid_init(&hero_gimbal_init->pitch_gimbal_motor.pid_gimbal_angle_ac, PID_POSITION, Pitch_ABSOLUTE_angle_pid_ac, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT_AC, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT_AC);
	pid_init(&hero_gimbal_init->pitch_gimbal_motor.pid_gimbal_speed_ac, PID_POSITION, Pitch_ABSOLUTE_speed_pid_ac, PITCH_ABSOLUTE_SPEED_PID_MAX_OUT_AC, PITCH_ABSOLUTE_SPEED_PID_MAX_IOUT_AC);
    
    // 初始化角度设定值
	hero_gimbal_init->pitch_gimbal_motor.absolute_angle_set = hero_gimbal_init->pitch_gimbal_motor.relative_angle_set = 0;
	hero_gimbal_init->yaw_gimbal_motor.absolute_angle_set = hero_gimbal_init->yaw_gimbal_motor.relative_angle_set = 0;
    
    // 初始化云台方向变化相关参数
	hero_gimbal_init->gimbal_angle_change.gimbal_change_flag = 0;
	hero_gimbal_init->gimbal_angle_change.gimbal_change_time = 0;
	hero_gimbal_init->gimbal_angle_change.gimbal_change_chassis_time = 0;
	hero_gimbal_init->gimbal_angle_change.gimbal_chnamge_chassis_flag = 0;
    
    // 更新反馈数据
    Gimbal_Feedback_Updata(hero_gimbal_init);
}

/**
 * @brief 设置云台控制模式
 * @param hero_gimbal_set_mode 云台控制结构体指针
 * @note 根据遥控器开关状态设置云台的控制模式
 */
static void Gimbal_Set_Mode(gimbal_control_t *hero_gimbal_set_mode)
{
    if(hero_gimbal_set_mode == NULL)
    {
        return;
    }
    
    // 右拨杆向上：比赛模式
	if(switch_is_up(hero_gimbal_set_mode->gimbal_dt7_remote_control->rc.s[RIGHT]))
	{
		hero_gimbal_set_mode->gimbal_mode = GIMBAL_GYRO;  // 默认陀螺仪模式
		
        // 鼠标右键按下：自动瞄准模式
		if(hero_gimbal_set_mode->gimbal_dt7_remote_control->mouse.press_r == 1)
		{
			hero_gimbal_set_mode->gimbal_mode = GIMBAL_AUTO;
		}
		else  // 开启自瞄的时候不允许掉头
		{
            // 保存上一次的云台方向
			hero_gimbal_set_mode->gimbal_angle_change.gimbal_change_last = hero_gimbal_set_mode->gimbal_angle_change.gimbal_change;
            
            // 处理云台掉头控制逻辑
            // 条件：通道4值>=300 或 按下E键 且 掉头标志为0 且 掉头等待时间满足
			if((hero_gimbal_set_mode->gimbal_dt7_remote_control->rc.ch[4] >= 300 || 
                hero_gimbal_set_mode->gimbal_dt7_remote_control->key.v & KEY_PRESSED_OFFSET_E) &&
                hero_gimbal_set_mode->gimbal_angle_change.gimbal_change_flag == 0 &&
                hero_gimbal_set_mode->gimbal_angle_change.gimbal_change_time >= YAW_CHANGE_WAIT_TIME/2)
		    {
                // 设置新的零位角度，切换为前向
				hero_gimbal_set_mode->yaw_gimbal_motor.relative_ecd_aero = YAW_CHANGE_ANGLE_SET;
				hero_gimbal_set_mode->gimbal_angle_change.gimbal_change = GIMABL_CHANGE_FORWARD;
				hero_gimbal_set_mode->gimbal_angle_change.gimbal_change_flag = 1;
				hero_gimbal_set_mode->gimbal_angle_change.gimbal_change_time = 0;
		    }
            // 条件：通道4值>=300 或 按下E键 且 掉头标志为1 且 掉头等待时间满足
			else if((hero_gimbal_set_mode->gimbal_dt7_remote_control->rc.ch[4] >= 300 || 
                     hero_gimbal_set_mode->gimbal_dt7_remote_control->key.v & KEY_PRESSED_OFFSET_E) &&
                    hero_gimbal_set_mode->gimbal_angle_change.gimbal_change_flag == 1 &&
                    hero_gimbal_set_mode->gimbal_angle_change.gimbal_change_time >= YAW_CHANGE_WAIT_TIME/2)
			{
                // 恢复原始零位角度，切换为正常前向
				hero_gimbal_set_mode->yaw_gimbal_motor.relative_ecd_aero = YAW_ZERO_ANGLE_SET;
				hero_gimbal_set_mode->gimbal_angle_change.gimbal_change = GIMABL_NOMAL_FORWARD;
				hero_gimbal_set_mode->gimbal_angle_change.gimbal_change_flag = 0;
				hero_gimbal_set_mode->gimbal_angle_change.gimbal_change_time = 0;
			}
            
            // 更新掉头时间计数器，防止溢出
			if(hero_gimbal_set_mode->gimbal_angle_change.gimbal_change_time <= YAW_CHANGE_WAIT_TIME)
			{
				hero_gimbal_set_mode->gimbal_angle_change.gimbal_change_time++;
			}
			else
			{
				hero_gimbal_set_mode->gimbal_angle_change.gimbal_change_time = YAW_CHANGE_WAIT_TIME;
			}
		}
	}
	else  // 调试模式
	{
        // 根据右拨杆位置设置模式
		if(switch_is_down(hero_gimbal_set_mode->gimbal_dt7_remote_control->rc.s[RIGHT]))
		{
			hero_gimbal_set_mode->gimbal_mode = GIMBAL_GYRO;
		}
		else if(switch_is_mid(hero_gimbal_set_mode->gimbal_dt7_remote_control->rc.s[RIGHT]))
		{
			hero_gimbal_set_mode->gimbal_mode = GIMBAL_GYRO;
		}
        
        // 调试模式下的自瞄控制
		if(hero_gimbal_set_mode->gimbal_mode == GIMBAL_GYRO)
		{
			if(switch_is_down(hero_gimbal_set_mode->gimbal_dt7_remote_control->rc.s[LEFT]) || 
               switch_is_mid(hero_gimbal_set_mode->gimbal_dt7_remote_control->rc.s[LEFT]))
			{
				hero_gimbal_set_mode->gimbal_mode = GIMBAL_GYRO;
			}
			else
			{
				hero_gimbal_set_mode->gimbal_mode = GIMBAL_AUTO;
			}
		}
	}
    
    // 根据云台模式设置电机模式
    if(hero_gimbal_set_mode->gimbal_mode == GIMBAL_ZERO_FORCE)
    {
        hero_gimbal_set_mode->yaw_gimbal_motor.gimbal_motor_mode = MOTOR_ZERO_FORCE;
        hero_gimbal_set_mode->pitch_gimbal_motor.gimbal_motor_mode = MOTOR_ZERO_FORCE;
    }
    else if(hero_gimbal_set_mode->gimbal_mode == GIMBAL_ENCONDE)
    {
        hero_gimbal_set_mode->yaw_gimbal_motor.gimbal_motor_mode = MOTOR_ENCONDE;
        hero_gimbal_set_mode->pitch_gimbal_motor.gimbal_motor_mode = MOTOR_ENCONDE;
    }
    else if(hero_gimbal_set_mode->gimbal_mode == GIMBAL_GYRO || hero_gimbal_set_mode->gimbal_mode == GIMBAL_AUTO)
    {
        hero_gimbal_set_mode->yaw_gimbal_motor.gimbal_motor_mode = MOTOR_GYRO;
        hero_gimbal_set_mode->pitch_gimbal_motor.gimbal_motor_mode = MOTOR_GYRO;
    }
}

/**
 * @brief 模式切换反馈处理
 * @param hero_gimbal_mode_change_feedback_change 云台控制结构体指针
 * @note 处理模式切换时的初始值设置，避免突变
 */
static void Gimbal_Mode_Change_Feedback_Change(gimbal_control_t *hero_gimbal_mode_change_feedback_change)
{
    if(hero_gimbal_mode_change_feedback_change == NULL)
    {
        return;
    }
    
    // YAW轴模式切换处理
    if(hero_gimbal_mode_change_feedback_change->yaw_gimbal_motor.gimbal_motor_mode == MOTOR_ZERO_FORCE && 
       hero_gimbal_mode_change_feedback_change->yaw_gimbal_motor.gimbal_motor_mode_last != MOTOR_ZERO_FORCE)
    {
        // 无力模式切换处理
    }
    else if(hero_gimbal_mode_change_feedback_change->yaw_gimbal_motor.gimbal_motor_mode == MOTOR_ENCONDE && 
            hero_gimbal_mode_change_feedback_change->yaw_gimbal_motor.gimbal_motor_mode_last != MOTOR_ENCONDE)
    {
        // 编码器模式：设置相对角度设定值为当前相对角度
    	hero_gimbal_mode_change_feedback_change->yaw_gimbal_motor.relative_angle_set = 
            hero_gimbal_mode_change_feedback_change->yaw_gimbal_motor.relative_angle;
    }
    else if(hero_gimbal_mode_change_feedback_change->yaw_gimbal_motor.gimbal_motor_mode == MOTOR_GYRO && 
            hero_gimbal_mode_change_feedback_change->yaw_gimbal_motor.gimbal_motor_mode_last != MOTOR_GYRO)
    {
        // 陀螺仪模式：设置绝对角度设定值为当前绝对角度
        hero_gimbal_mode_change_feedback_change->pitch_gimbal_motor.absolute_angle_set = 
            hero_gimbal_mode_change_feedback_change->pitch_gimbal_motor.absolute_angle;
        hero_gimbal_mode_change_feedback_change->yaw_gimbal_motor.absolute_angle_set = 
            hero_gimbal_mode_change_feedback_change->yaw_gimbal_motor.absolute_angle;
    }
    
    // PITCH轴模式切换处理
    if(hero_gimbal_mode_change_feedback_change->pitch_gimbal_motor.gimbal_motor_mode == MOTOR_ZERO_FORCE && 
       hero_gimbal_mode_change_feedback_change->pitch_gimbal_motor.gimbal_motor_mode_last != MOTOR_ZERO_FORCE)
    {
        // 无力模式切换处理
    }
    else if(hero_gimbal_mode_change_feedback_change->pitch_gimbal_motor.gimbal_motor_mode == MOTOR_ENCONDE && 
            hero_gimbal_mode_change_feedback_change->pitch_gimbal_motor.gimbal_motor_mode_last != MOTOR_ENCONDE)
    {
        // 编码器模式：设置相对角度设定值为当前相对角度
        hero_gimbal_mode_change_feedback_change->pitch_gimbal_motor.relative_angle_set = 
            hero_gimbal_mode_change_feedback_change->pitch_gimbal_motor.relative_angle;
    }
    else if(hero_gimbal_mode_change_feedback_change->pitch_gimbal_motor.gimbal_motor_mode == MOTOR_GYRO && 
            hero_gimbal_mode_change_feedback_change->pitch_gimbal_motor.gimbal_motor_mode_last != MOTOR_GYRO)
    {
        // 陀螺仪模式：设置绝对角度设定值为当前绝对角度
        hero_gimbal_mode_change_feedback_change->pitch_gimbal_motor.absolute_angle_set = 
            hero_gimbal_mode_change_feedback_change->pitch_gimbal_motor.absolute_angle;
    }
    
    // 保存当前模式为上一次模式
	hero_gimbal_mode_change_feedback_change->pitch_gimbal_motor.gimbal_motor_mode_last = 
        hero_gimbal_mode_change_feedback_change->pitch_gimbal_motor.gimbal_motor_mode;
	hero_gimbal_mode_change_feedback_change->yaw_gimbal_motor.gimbal_motor_mode_last = 
        hero_gimbal_mode_change_feedback_change->yaw_gimbal_motor.gimbal_motor_mode;
}

/**
 * @brief 更新反馈数据
 * @param hero_gimbal_feedback_updata 云台控制结构体指针
 * @note 从传感器读取并更新云台的角度、角速度等反馈数据
 */
static void Gimbal_Feedback_Updata(gimbal_control_t *hero_gimbal_feedback_updata)
{
    if(hero_gimbal_feedback_updata == NULL)
    {
        return;
    }
    
    // 更新PITCH轴反馈数据
    hero_gimbal_feedback_updata->pitch_gimbal_motor.relative_angle = 
        Motor_Ecd_To_Angle_pitch(hero_gimbal_feedback_updata->pitch_gimbal_motor.gimbal_motor_measure->ecd, PITCH_ZERO_ANGLE_SET);
    
    hero_gimbal_feedback_updata->pitch_gimbal_motor.absolute_angle = 
        *(hero_gimbal_feedback_updata->gimbal_INS_angle_point + INS_PITCH_ADDRESS_OFFSET);
    
	hero_gimbal_feedback_updata->pitch_gimbal_motor.absolute_gyro = 
        *(hero_gimbal_feedback_updata->gimbal_INS_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);
    
	hero_gimbal_feedback_updata->pitch_gimbal_motor.relative_omega = 
        (hero_gimbal_feedback_updata->pitch_gimbal_motor.gimbal_motor_measure->speed_rpm * MOTOR_RPM_RADS);
    
    // 更新YAW轴反馈数据
    hero_gimbal_feedback_updata->yaw_gimbal_motor.relative_angle = 
        Motor_Ecd_To_Angle_yaw(hero_gimbal_feedback_updata->yaw_gimbal_motor.gimbal_motor_measure->ecd, 
                              hero_gimbal_feedback_updata->yaw_gimbal_motor.relative_ecd_aero);
    
	hero_gimbal_feedback_updata->yaw_gimbal_motor.relative_omega = 
        hero_gimbal_feedback_updata->yaw_gimbal_motor.gimbal_motor_measure->speed_rpm * MOTOR_RPM_RADS;
    
    hero_gimbal_feedback_updata->yaw_gimbal_motor.absolute_angle = 
        *(hero_gimbal_feedback_updata->gimbal_INS_angle_point + INS_YAW_ADDRESS_OFFSET);
    
	hero_gimbal_feedback_updata->yaw_gimbal_motor.absolute_gyro = 
        *(hero_gimbal_feedback_updata->gimbal_INS_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET);
}

/**
 * @brief 根据模式设置控制输入
 * @param hero_gimbal_mode_set_control 云台控制结构体指针
 * @note 根据不同的控制模式计算控制增量，并更新设定值
 */
static void Gimbal_Mode_Set_Control(gimbal_control_t *hero_gimbal_mode_set_control)
{
    if(hero_gimbal_mode_set_control == NULL)
    {
        return;
    }
    
    fp32 add_yaw_angle = 0.0f;    // YAW轴角度增量
    fp32 add_pitch_angle = 0.0f;   // PITCH轴角度增量
    
    // 根据不同模式调用对应的控制函数
    if(hero_gimbal_mode_set_control->gimbal_mode == GIMBAL_ZERO_FORCE)
    {
        Gimbal_Zero_Force_Control(&add_yaw_angle, &add_pitch_angle, hero_gimbal_mode_set_control);
    }
    else if(hero_gimbal_mode_set_control->gimbal_mode == GIMBAL_ENCONDE)
    {
        // 编码器模式：相对角度控制
        Gimbal_Relative_Angle_Control(&add_yaw_angle, &add_pitch_angle, hero_gimbal_mode_set_control);
        
        // 更新相对角度设定值
        hero_gimbal_mode_set_control->pitch_gimbal_motor.relative_angle_set += add_pitch_angle;
        hero_gimbal_mode_set_control->yaw_gimbal_motor.relative_angle_set += add_yaw_angle;
        
        // YAW轴角度限幅 [-π, π]
        if(hero_gimbal_mode_set_control->yaw_gimbal_motor.relative_angle_set > PI)
            hero_gimbal_mode_set_control->yaw_gimbal_motor.relative_angle_set -= 2 * PI;
        else if(hero_gimbal_mode_set_control->yaw_gimbal_motor.relative_angle_set < -PI)
            hero_gimbal_mode_set_control->yaw_gimbal_motor.relative_angle_set += 2 * PI;
        
        // PITCH轴角度限幅
        if(hero_gimbal_mode_set_control->pitch_gimbal_motor.relative_angle_set > GIMBAL_PITCH_RELATIVE_MAX_ANGLE)
            hero_gimbal_mode_set_control->pitch_gimbal_motor.relative_angle_set = GIMBAL_PITCH_RELATIVE_MAX_ANGLE;
        else if(hero_gimbal_mode_set_control->pitch_gimbal_motor.relative_angle_set < GIMBAL_PITCH_RELATIVE_MIN_ANGLE)
            hero_gimbal_mode_set_control->pitch_gimbal_motor.relative_angle_set = GIMBAL_PITCH_RELATIVE_MIN_ANGLE;
    }
    else if(hero_gimbal_mode_set_control->gimbal_mode == GIMBAL_GYRO)
    {
        // 陀螺仪模式：绝对角度控制
        Gimbal_Absolute_Angle_Control(&add_yaw_angle, &add_pitch_angle, hero_gimbal_mode_set_control);
        
        // 更新绝对角度设定值
        hero_gimbal_mode_set_control->pitch_gimbal_motor.absolute_angle_set += add_pitch_angle;
        hero_gimbal_mode_set_control->yaw_gimbal_motor.absolute_angle_set += add_yaw_angle;
        
        // 处理云台掉头逻辑
		if(hero_gimbal_mode_set_control->gimbal_angle_change.gimbal_change != 
           hero_gimbal_mode_set_control->gimbal_angle_change.gimbal_change_last)
		{
			if(hero_gimbal_mode_set_control->gimbal_angle_change.gimbal_change == GIMABL_NOMAL_FORWARD)
			{
                // 从掉头状态恢复：加π
				hero_gimbal_mode_set_control->yaw_gimbal_motor.absolute_angle_set = 
                    hero_gimbal_mode_set_control->yaw_gimbal_motor.absolute_angle_set + PI;
			}
			else if(hero_gimbal_mode_set_control->gimbal_angle_change.gimbal_change == GIMABL_CHANGE_FORWARD)
			{
                // 进入掉头状态：减π
				hero_gimbal_mode_set_control->yaw_gimbal_motor.absolute_angle_set = 
                    hero_gimbal_mode_set_control->yaw_gimbal_motor.absolute_angle_set - PI;
			}
		}
        
        // YAW轴角度限幅 [-π, π]
        if(hero_gimbal_mode_set_control->yaw_gimbal_motor.absolute_angle_set > PI)
            hero_gimbal_mode_set_control->yaw_gimbal_motor.absolute_angle_set -= 2 * PI;
        else if(hero_gimbal_mode_set_control->yaw_gimbal_motor.absolute_angle_set < -PI)
            hero_gimbal_mode_set_control->yaw_gimbal_motor.absolute_angle_set += 2 * PI;
        
        // PITCH轴角度限幅
        if(hero_gimbal_mode_set_control->pitch_gimbal_motor.absolute_angle_set > GIMBAL_PITCH_ABSOULATE_MAX_ANGLE)
            hero_gimbal_mode_set_control->pitch_gimbal_motor.absolute_angle_set = GIMBAL_PITCH_ABSOULATE_MAX_ANGLE;
        else if(hero_gimbal_mode_set_control->pitch_gimbal_motor.absolute_angle_set < GIMBAL_PITCH_ABSOULATE_MIN_ANGLE)
            hero_gimbal_mode_set_control->pitch_gimbal_motor.absolute_angle_set = GIMBAL_PITCH_ABSOULATE_MIN_ANGLE;
    }
    else if(hero_gimbal_mode_set_control->gimbal_mode == GIMBAL_AUTO)
    {
        // 自动瞄准模式
        if(hero_gimbal_mode_set_control->hero_pc->pc_fire_mode == 1 || 
           hero_gimbal_mode_set_control->hero_pc->pc_fire_mode == 2)  // 有目标
        {
            // 自动控制：使用PC发送的目标角度
            Gimbal_Auto_Control(&add_yaw_angle, &add_pitch_angle, hero_gimbal_mode_set_control);
            hero_gimbal_mode_set_control->pitch_gimbal_motor.absolute_angle_set = add_pitch_angle;
            hero_gimbal_mode_set_control->yaw_gimbal_motor.absolute_angle_set = add_yaw_angle;
        }
        else
        {
            // 无目标时使用绝对角度控制
            Gimbal_Absolute_Angle_Control(&add_yaw_angle, &add_pitch_angle, hero_gimbal_mode_set_control);
            hero_gimbal_mode_set_control->pitch_gimbal_motor.absolute_angle_set += add_pitch_angle;
            hero_gimbal_mode_set_control->yaw_gimbal_motor.absolute_angle_set += add_yaw_angle;
        }
        
        // 角度限幅
        if(hero_gimbal_mode_set_control->yaw_gimbal_motor.absolute_angle_set > PI)
            hero_gimbal_mode_set_control->yaw_gimbal_motor.absolute_angle_set -= 2 * PI;
        else if(hero_gimbal_mode_set_control->yaw_gimbal_motor.absolute_angle_set < -PI)
            hero_gimbal_mode_set_control->yaw_gimbal_motor.absolute_angle_set += 2 * PI;
        
        if(hero_gimbal_mode_set_control->pitch_gimbal_motor.absolute_angle_set > GIMBAL_PITCH_ABSOULATE_MAX_ANGLE)
            hero_gimbal_mode_set_control->pitch_gimbal_motor.absolute_angle_set = GIMBAL_PITCH_ABSOULATE_MAX_ANGLE;
        else if(hero_gimbal_mode_set_control->pitch_gimbal_motor.absolute_angle_set < GIMBAL_PITCH_ABSOULATE_MIN_ANGLE)
            hero_gimbal_mode_set_control->pitch_gimbal_motor.absolute_angle_set = GIMBAL_PITCH_ABSOULATE_MIN_ANGLE;
    }
}

/**
 * @brief 控制计算
 * @param hero_gimbal_control_cacl 云台控制结构体指针
 * @note 根据电机模式和控制模式计算最终的控制电流
 */
static void Gimbal_Control_Cacl(gimbal_control_t *hero_gimbal_control_cacl)
{
    // 无力模式
    if(hero_gimbal_control_cacl->yaw_gimbal_motor.gimbal_motor_mode == MOTOR_ZERO_FORCE)
    {
        hero_gimbal_control_cacl->yaw_gimbal_motor.give_current = 0;
        hero_gimbal_control_cacl->pitch_gimbal_motor.give_current = 0;
    }
    // 编码器模式
    else if(hero_gimbal_control_cacl->yaw_gimbal_motor.gimbal_motor_mode == MOTOR_ENCONDE)
    {
        // YAW轴：相对角度PID控制
		hero_gimbal_control_cacl->yaw_gimbal_motor.relative_omega_set = 
            pid_calc_gimbal(&hero_gimbal_control_cacl->yaw_gimbal_motor.pid_gimbal_angle_relative,
                           hero_gimbal_control_cacl->yaw_gimbal_motor.relative_angle,
                           hero_gimbal_control_cacl->yaw_gimbal_motor.relative_angle_set);
        
		hero_gimbal_control_cacl->yaw_gimbal_motor.give_current = 
            pid_calc(&hero_gimbal_control_cacl->yaw_gimbal_motor.pid_gimbal_speed_relative,
                    hero_gimbal_control_cacl->yaw_gimbal_motor.relative_omega,
                    hero_gimbal_control_cacl->yaw_gimbal_motor.relative_omega_set);
        
        // PITCH轴：LQR控制
		fp32 pitch_feedback_enconde[3] = {
            (fp32) hero_gimbal_control_cacl->pitch_gimbal_motor.relative_omega,
            (fp32) hero_gimbal_control_cacl->pitch_gimbal_motor.relative_angle,
            (fp32) hero_gimbal_control_cacl->pitch_gimbal_motor.gimbal_motor_measure->given_current
        };
        
		fp32 pitch_set_enconde[3] = {
            0,
            (fp32) hero_gimbal_control_cacl->pitch_gimbal_motor.relative_angle_set,
            0
        };
        
		hero_gimbal_control_cacl->pitch_gimbal_motor.give_current = 
            lqr_cal(&hero_gimbal_control_cacl->pitch_gimbal_motor.lqr,
                   pitch_feedback_enconde,
                   pitch_set_enconde);
    }
    // 陀螺仪模式
    else if(hero_gimbal_control_cacl->yaw_gimbal_motor.gimbal_motor_mode == MOTOR_GYRO)
    {
        // 陀螺仪反馈数据准备
		fp32 pitch_feedback_gyro[3] = { 
            (fp32) hero_gimbal_control_cacl->pitch_gimbal_motor.absolute_gyro,
            (fp32) hero_gimbal_control_cacl->pitch_gimbal_motor.absolute_angle,
            (fp32) hero_gimbal_control_cacl->pitch_gimbal_motor.gimbal_motor_measure->given_current
        };
        
		fp32 pitch_set_gyro[3] = { 
            0,
            (fp32) hero_gimbal_control_cacl->pitch_gimbal_motor.absolute_angle_set,
            0
        };
        
        // 普通陀螺仪模式
        if(hero_gimbal_control_cacl->gimbal_mode == GIMBAL_GYRO)
        {
            // YAW轴：串级PID控制
			hero_gimbal_control_cacl->yaw_gimbal_motor.absolute_gyro_set = 
                pid_calc_gimbal(&hero_gimbal_control_cacl->yaw_gimbal_motor.pid_gimbal_angle,
                               hero_gimbal_control_cacl->yaw_gimbal_motor.absolute_angle,
                               hero_gimbal_control_cacl->yaw_gimbal_motor.absolute_angle_set);
            
			hero_gimbal_control_cacl->yaw_gimbal_motor.give_current = 
                pid_calc(&hero_gimbal_control_cacl->yaw_gimbal_motor.pid_gimbal_speed,
                        hero_gimbal_control_cacl->yaw_gimbal_motor.absolute_gyro,
                        hero_gimbal_control_cacl->yaw_gimbal_motor.absolute_gyro_set);
            
            // PITCH轴：串级PID控制
			hero_gimbal_control_cacl->pitch_gimbal_motor.absolute_gyro_set = 
                pid_calc(&hero_gimbal_control_cacl->pitch_gimbal_motor.pid_gimbal_angle,
                        hero_gimbal_control_cacl->pitch_gimbal_motor.absolute_angle,
                        hero_gimbal_control_cacl->pitch_gimbal_motor.absolute_angle_set);
            
			hero_gimbal_control_cacl->pitch_gimbal_motor.give_current = 
                -pid_calc(&hero_gimbal_control_cacl->pitch_gimbal_motor.pid_gimbal_speed,
                        hero_gimbal_control_cacl->pitch_gimbal_motor.absolute_gyro,
                        hero_gimbal_control_cacl->pitch_gimbal_motor.absolute_gyro_set);
        }
        // 自动瞄准模式
        else if(hero_gimbal_control_cacl->gimbal_mode == GIMBAL_AUTO)
        {
            // YAW轴：自瞄模式串级PID控制
			hero_gimbal_control_cacl->yaw_gimbal_motor.absolute_gyro_set = 
                pid_calc_gimbal(&hero_gimbal_control_cacl->yaw_gimbal_motor.pid_gimbal_angle_ac,
                               hero_gimbal_control_cacl->yaw_gimbal_motor.absolute_angle,
                               hero_gimbal_control_cacl->yaw_gimbal_motor.absolute_angle_set);
            
			hero_gimbal_control_cacl->yaw_gimbal_motor.give_current = 
                pid_calc(&hero_gimbal_control_cacl->yaw_gimbal_motor.pid_gimbal_speed_ac,
                        hero_gimbal_control_cacl->yaw_gimbal_motor.absolute_gyro,
                        hero_gimbal_control_cacl->yaw_gimbal_motor.absolute_gyro_set);
            
            // PITCH轴：自瞄模式串级PID控制
			hero_gimbal_control_cacl->pitch_gimbal_motor.absolute_gyro_set = 
               pid_calc(&hero_gimbal_control_cacl->pitch_gimbal_motor.pid_gimbal_angle_ac,
                        hero_gimbal_control_cacl->pitch_gimbal_motor.absolute_angle,
                        hero_gimbal_control_cacl->pitch_gimbal_motor.absolute_angle_set);
            
			hero_gimbal_control_cacl->pitch_gimbal_motor.give_current =  -pid_calc(&hero_gimbal_control_cacl->pitch_gimbal_motor.pid_gimbal_speed_ac,
                        hero_gimbal_control_cacl->pitch_gimbal_motor.absolute_gyro,
                        hero_gimbal_control_cacl->pitch_gimbal_motor.absolute_gyro_set);
        }
    }
}

/**
 * @brief YAW轴编码器值转换为角度
 * @param ecd 编码器原始值
 * @param ecd_zero_angle_set 编码器零位
 * @return 计算得到的角度值（弧度）
 * @note 处理编码器溢出，将编码器值转换为弧度
 */
static fp32 Motor_Ecd_To_Angle_yaw(uint16_t ecd, uint16_t ecd_zero_angle_set)
{
    int32_t relative_ecd = ecd - ecd_zero_angle_set;
    
    // 处理编码器溢出
    if(relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if(relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }
    
    return relative_ecd * ECD_TO_RAD;  // 转换为弧度
}

/**
 * @brief PITCH轴编码器值转换为角度
 * @param ecd 编码器原始值
 * @param ecd_zero_angle_set 编码器零位
 * @return 计算得到的角度值（弧度）
 * @note 处理编码器溢出，将编码器值转换为弧度
 */
static fp32 Motor_Ecd_To_Angle_pitch(uint16_t ecd, uint16_t ecd_zero_angle_set)
{
    int32_t relative_ecd = ecd - ecd_zero_angle_set;
    
    // 处理编码器溢出
    if(relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if(relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }
    
    return relative_ecd * MOTOR_ECD_TO_RAD;  // 转换为弧度
}

/**
 * @brief 获取云台控制结构体指针
 * @return 云台控制结构体指针
 * @note 提供全局云台控制实例的访问接口
 */
extern gimbal_control_t *get_gimbal_control_point(void)
{
    return &mini_gimbal;
}

/**
 * @brief 无力控制函数
 * @param yaw YAW轴角度增量输出
 * @param pitch PITCH轴角度增量输出
 * @param gimbal_control_set 云台控制结构体指针
 * @note 在无力模式下，输出为零
 */
static void Gimbal_Zero_Force_Control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
	if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    *yaw = 0.0f;
    *pitch = 0.0f;
}

/**
 * @brief 相对角度控制函数
 * @param yaw YAW轴角度增量输出
 * @param pitch PITCH轴角度增量输出
 * @param gimbal_control_set 云台控制结构体指针
 * @note 根据遥控器通道和鼠标输入计算角度增量
 */
static void Gimbal_Relative_Angle_Control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    static int16_t yaw_channel = 0, pitch_channel = 0;
    
    // 遥控器通道死区处理
    Remote_Deadband_Limit(gimbal_control_set->gimbal_dt7_remote_control->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    Remote_Deadband_Limit(gimbal_control_set->gimbal_dt7_remote_control->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);
    
    // 计算角度增量：遥控器通道 + 鼠标输入
    *yaw = yaw_channel * YAW_RC_SEN - gimbal_control_set->gimbal_dt7_remote_control->mouse.x * YAW_MOUSE_SEN;
    *pitch = -pitch_channel * PITCH_RC_SEN + gimbal_control_set->gimbal_dt7_remote_control->mouse.y * PITCH_MOUSE_SEN;
}

/**
 * @brief 绝对角度控制函数
 * @param yaw YAW轴角度增量输出
 * @param pitch PITCH轴角度增量输出
 * @param gimbal_control_set 云台控制结构体指针
 * @note 根据遥控器通道和鼠标输入计算角度增量
 */
static void Gimbal_Absolute_Angle_Control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    static int16_t yaw_channel = 0, pitch_channel = 0;
    
    // 遥控器通道死区处理
    Remote_Deadband_Limit(gimbal_control_set->gimbal_dt7_remote_control->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    Remote_Deadband_Limit(gimbal_control_set->gimbal_dt7_remote_control->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);
    
    // 计算角度增量：遥控器通道 + 鼠标输入
    *yaw = yaw_channel * YAW_RC_SEN - gimbal_control_set->gimbal_dt7_remote_control->mouse.x * YAW_MOUSE_SEN;
    *pitch = -pitch_channel * -PITCH_RC_SEN + gimbal_control_set->gimbal_dt7_remote_control->mouse.y * PITCH_MOUSE_SEN;
}

/**
 * @brief 自动控制函数
 * @param yaw YAW轴角度增量输出
 * @param pitch PITCH轴角度增量输出
 * @param gimbal_control_set 云台控制结构体指针
 * @note 从PC接收的目标信息中获取角度设定值
 */
static void Gimbal_Auto_Control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    *yaw = gimbal_control_set->hero_pc->pc_yaw;
    *pitch = gimbal_control_set->hero_pc->pc_pitch;
}



