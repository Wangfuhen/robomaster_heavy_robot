/**
 * @file gimbal_task.c
 * @brief 云台控制任务
 * @note 此文件实现了云台系统的完整控制逻辑，包括初始化、模式切换、反馈更新、控制计算等功能
 *       支持多种控制模式：无力模式、编码器模式、陀螺仪模式、自动瞄准模式
 */

#include "gimbal_task.h"
#include "shoot_task.h"
#include "string.h"

/* ============ AUTO aim (vision) parameters ============ */
#define AUTO_YAW_SET_RATE          3.0f      /* rad/s, vision target slew limit */
#define AUTO_PITCH_SET_RATE        1.0f      /* rad/s, vision target slew limit */
#define PITCH_AUTO_ANGLE_KP        3000.0f   /* rpm per rad, pitch position loop */
#define PITCH_AUTO_ANGLE_KD        0.0f      /* rpm per rad/s, pitch damping */
#define PITCH_AUTO_RPM_MAX         PITCH_SCREW_RPM
#define PITCH_AUTO_RPM_DIR         PITCH_TRAVEL_DIR  /* same plant sign as travel limit */

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
static void Gimbal_Screw_Speed_Control(fp32 *rpm, gimbal_control_t *gimbal_control_set);  /* pitch screw fixed speed */
static void Gimbal_ApplyPitchTravelLimit(gimbal_control_t *g);  /* pitch travel limit, all modes */
/* ==================== YAW 轴软件限位与回位 ==================== */
static uint8_t Gimbal_Yaw_Limit_Recover(uint16_t ecd, int16_t *iq_out);
static void    Gimbal_Yaw_Limit_Setpoint(gimbal_control_t *g);
static int32_t Yaw_Ecd_Phi(uint16_t ecd);
static uint8_t Gimbal_Yaw_Safe_Check(int16_t iq, int16_t speed);
static void    Yaw_Retreat_Start(uint16_t ecd, int8_t dir);
static uint8_t Yaw_Retreat_Step(uint16_t ecd, int16_t *iq_out);

volatile uint32_t yaw_ecd_update_tick = 0;   // 0x141 收到时刻，can_user.c 中更新

static yaw_limit_state_e yaw_limit_state = YAW_LIMIT_IDLE;
static uint8_t  yaw_need_home        = 1;   // 开机首次越界需要回中
static int8_t   yaw_recover_dir        = 0;
static uint8_t  yaw_recover_try        = 0;
static uint32_t yaw_recover_tick       = 0;
static uint32_t yaw_backoff_tick       = 0;
static uint32_t yaw_backoff_ms         = 0;
static uint16_t yaw_recover_ecd        = 0;
static uint32_t yaw_recover_stall_tick = 0;

static uint8_t  yaw_retreat_on         = 0;
static uint8_t  yaw_retreat_done       = 0;
static int8_t   yaw_retreat_dir        = 0;
static uint16_t yaw_retreat_last       = 0;
static int16_t  yaw_retreat_acc        = 0;
static uint32_t yaw_retreat_tick       = 0;
static uint32_t yaw_retreat_stall_tick = 0;

static uint8_t  yaw_stall_holding      = 0;
static uint32_t yaw_stall_hold_tick    = 0;
static uint16_t yaw_block_cnt          = 0;

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
        uint16_t yaw_ecd = mini_gimbal.yaw_gimbal_motor.gimbal_motor_measure->ecd;
        int16_t  safe_iq = 0;

        // 越界回位（含反退），期间跳过模式控制
        if (Gimbal_Yaw_Limit_Recover(yaw_ecd, &safe_iq))
        {
            CAN_cmd_gimbal_yaw(safe_iq);
            CAN_motor_send_data_pitch(0);
            vTaskDelay(GIMBAL_LOOP_TIME);
            continue;
        }

        // 运行堵转反退
        if (Yaw_Retreat_Step(yaw_ecd, &safe_iq) == 1)
        {
            CAN_cmd_gimbal_yaw(safe_iq);
            CAN_motor_send_data_pitch(0);
            vTaskDelay(GIMBAL_LOOP_TIME);
            continue;
        }

        // 反退结束后对齐设定值，避免弹回原位
        if (yaw_retreat_done)
        {
            yaw_retreat_done = 0;
            mini_gimbal.yaw_gimbal_motor.absolute_angle_set = mini_gimbal.yaw_gimbal_motor.absolute_angle;
            mini_gimbal.yaw_gimbal_motor.absolute_gyro_set = 0;
            pid_clear(&mini_gimbal.yaw_gimbal_motor.pid_gimbal_angle);
            pid_clear(&mini_gimbal.yaw_gimbal_motor.pid_gimbal_speed);
        }

        // 云台控制主循环
        Gimbal_Set_Mode(&mini_gimbal);  // 设置云台控制模式
        Gimbal_Mode_Change_Feedback_Change(&mini_gimbal);  // 模式切换反馈处理
        Gimbal_Feedback_Updata(&mini_gimbal);  // 更新反馈数据
        Gimbal_Mode_Set_Control(&mini_gimbal);  // 根据模式设置控制量
        Gimbal_Yaw_Limit_Setpoint(&mini_gimbal);  // 限位：设定值钳位 + 减速包络
        Gimbal_Control_Cacl(&mini_gimbal);  // 控制计算
        Gimbal_ApplyPitchTravelLimit(&mini_gimbal);  // pitch 行程限位（全模式）
        
        // 看门狗
        safe_iq = (int16_t)mini_gimbal.yaw_gimbal_motor.give_current;

        if (Gimbal_Yaw_Safe_Check(safe_iq,
                mini_gimbal.yaw_gimbal_motor.gimbal_motor_measure->speed_rpm))
        {
            int8_t drive_dir = (safe_iq * YAW_LIMIT_DIR > 0) ? (+1) : (-1);

            Yaw_Retreat_Start(yaw_ecd, (int8_t)(-drive_dir));
            safe_iq = 0;
        }

        mini_gimbal.yaw_gimbal_motor.give_current = safe_iq;

        // 发送控制指令到电机
        CAN_cmd_gimbal_yaw(safe_iq);
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
	static const fp32 Pitch_SCREW_speed_pid[3] = {PITCH_SCREW_SPEED_PID_KP, PITCH_SCREW_SPEED_PID_KI, PITCH_SCREW_SPEED_PID_KD};
	pid_init(&hero_gimbal_init->pitch_gimbal_motor.pid_gimbal_screw_speed, PID_POSITION, Pitch_SCREW_speed_pid, PITCH_SCREW_SPEED_MAX_OUT, PITCH_SCREW_SPEED_MAX_IOUT);
    
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
/* ===== ONE-KEY TURN-AROUND DISABLED (kept for rollback) =====
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

*/
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
        pid_clear(&hero_gimbal_mode_change_feedback_change->pitch_gimbal_motor.pid_gimbal_screw_speed);
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

/* ==================== AUTO aim helpers ==================== */
/**
 * @brief PC vision target validity check
 * @note Rejects stale, NaN/out-of-range and non-target frames.
 */
static uint8_t Gimbal_Auto_Target_Valid(const autom_pc *pc)
{
    if (pc == NULL)
    {
        return 0;
    }
    if (pc->pc_enable_flag <= 0)
    {
        return 0;
    }
    if (pc->pc_fire_mode != 1 && pc->pc_fire_mode != 2)
    {
        return 0;
    }
    if (!(pc->pc_yaw > -PI && pc->pc_yaw < PI))
    {
        return 0;
    }
    if (!(pc->pc_pitch > GIMBAL_PITCH_ABSOULATE_MIN_ANGLE &&
          pc->pc_pitch < GIMBAL_PITCH_ABSOULATE_MAX_ANGLE))
    {
        return 0;
    }
    return 1;
}

/**
 * @brief Normalize angle to [-PI, PI]
 */
static fp32 Gimbal_WrapPi(fp32 angle)
{
    while (angle > PI)
    {
        angle -= 2.0f * PI;
    }
    while (angle < -PI)
    {
        angle += 2.0f * PI;
    }
    return angle;
}

/**
 * @brief Linear slew limit for the pitch target
 */
static fp32 Gimbal_SlewLimit(fp32 current, fp32 target, fp32 max_step)
{
    fp32 diff = target - current;

    if (diff > max_step)
    {
        diff = max_step;
    }
    else if (diff < -max_step)
    {
        diff = -max_step;
    }
    return current + diff;
}

/**
 * @brief Shortest-path slew limit for the yaw target
 */
static fp32 Gimbal_SlewLimitAngle(fp32 current, fp32 target, fp32 max_step)
{
    fp32 diff = Gimbal_WrapPi(target - current);

    if (diff > max_step)
    {
        diff = max_step;
    }
    else if (diff < -max_step)
    {
        diff = -max_step;
    }
    return Gimbal_WrapPi(current + diff);
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
    fp32 pitch_rpm_set = 0.0f;     /* pitch screw target rpm */
    
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
        Gimbal_Screw_Speed_Control(&pitch_rpm_set, hero_gimbal_mode_set_control);
        hero_gimbal_mode_set_control->pitch_gimbal_motor.relative_omega_set = pitch_rpm_set;
        hero_gimbal_mode_set_control->yaw_gimbal_motor.absolute_angle_set += add_yaw_angle;
        
/* ===== TURN-AROUND +/-PI COMPENSATION DISABLED (kept for rollback) =====
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

*/
        
        // YAW轴角度限幅 [-π, π]
        if(hero_gimbal_mode_set_control->yaw_gimbal_motor.absolute_angle_set > PI)
            hero_gimbal_mode_set_control->yaw_gimbal_motor.absolute_angle_set -= 2 * PI;
        else if(hero_gimbal_mode_set_control->yaw_gimbal_motor.absolute_angle_set < -PI)
            hero_gimbal_mode_set_control->yaw_gimbal_motor.absolute_angle_set += 2 * PI;
        
    }
    else if(hero_gimbal_mode_set_control->gimbal_mode == GIMBAL_AUTO)
    {
        /* AUTO is a vision-ready GYRO mode:
         * - yaw/pitch use the same stable controllers as GYRO;
         * - PC targets are used only when fresh and in range;
         * - targets are slew-limited, so entering AUTO cannot jump. */
        if(Gimbal_Auto_Target_Valid(hero_gimbal_mode_set_control->hero_pc))
        {
            fp32 yaw_target;
            fp32 pitch_target;
            fp32 pitch_err;
            fp32 pitch_rpm;

            Gimbal_Auto_Control(&add_yaw_angle, &add_pitch_angle, hero_gimbal_mode_set_control);
            yaw_target   = Gimbal_WrapPi(add_yaw_angle);
            pitch_target = add_pitch_angle;

            if(pitch_target > GIMBAL_PITCH_ABSOULATE_MAX_ANGLE)
                pitch_target = GIMBAL_PITCH_ABSOULATE_MAX_ANGLE;
            else if(pitch_target < GIMBAL_PITCH_ABSOULATE_MIN_ANGLE)
                pitch_target = GIMBAL_PITCH_ABSOULATE_MIN_ANGLE;

            hero_gimbal_mode_set_control->yaw_gimbal_motor.absolute_angle_set =
                Gimbal_SlewLimitAngle(hero_gimbal_mode_set_control->yaw_gimbal_motor.absolute_angle_set,
                                      yaw_target,
                                      AUTO_YAW_SET_RATE * 0.001f);

            hero_gimbal_mode_set_control->pitch_gimbal_motor.absolute_angle_set =
                Gimbal_SlewLimit(hero_gimbal_mode_set_control->pitch_gimbal_motor.absolute_angle_set,
                                 pitch_target,
                                 AUTO_PITCH_SET_RATE * 0.001f);

            /* Position error -> 3508 rpm.  PITCH_AUTO_RPM_DIR uses the
             * calibrated plant sign (same as PITCH_TRAVEL_DIR). */
            pitch_err = hero_gimbal_mode_set_control->pitch_gimbal_motor.absolute_angle_set
                      - hero_gimbal_mode_set_control->pitch_gimbal_motor.absolute_angle;
            pitch_rpm = PITCH_AUTO_RPM_DIR *
                        (PITCH_AUTO_ANGLE_KP * pitch_err
                         - PITCH_AUTO_ANGLE_KD * hero_gimbal_mode_set_control->pitch_gimbal_motor.absolute_gyro);

            if(pitch_rpm > PITCH_AUTO_RPM_MAX) pitch_rpm = PITCH_AUTO_RPM_MAX;
            else if(pitch_rpm < -PITCH_AUTO_RPM_MAX) pitch_rpm = -PITCH_AUTO_RPM_MAX;

            hero_gimbal_mode_set_control->pitch_gimbal_motor.relative_omega_set = pitch_rpm;
        }
        else
        {
            /* No valid vision target: behave exactly like GYRO, so AUTO
             * never shakes while the PC is absent or the target is stale. */
            Gimbal_Absolute_Angle_Control(&add_yaw_angle, &add_pitch_angle, hero_gimbal_mode_set_control);
            hero_gimbal_mode_set_control->yaw_gimbal_motor.absolute_angle_set += add_yaw_angle;

            Gimbal_Screw_Speed_Control(&pitch_rpm_set, hero_gimbal_mode_set_control);
            hero_gimbal_mode_set_control->pitch_gimbal_motor.relative_omega_set = pitch_rpm_set;
        }

        hero_gimbal_mode_set_control->yaw_gimbal_motor.absolute_angle_set =
            Gimbal_WrapPi(hero_gimbal_mode_set_control->yaw_gimbal_motor.absolute_angle_set);

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
            
		/* PITCH: single speed loop (3508 + C620 + lead screw), target comes from relative_omega_set */
		hero_gimbal_control_cacl->pitch_gimbal_motor.give_current =
			pid_calc(&hero_gimbal_control_cacl->pitch_gimbal_motor.pid_gimbal_screw_speed,
				(fp32)hero_gimbal_control_cacl->pitch_gimbal_motor.gimbal_motor_measure->speed_rpm,
				hero_gimbal_control_cacl->pitch_gimbal_motor.relative_omega_set);
        }
        // 自动瞄准模式
        else if(hero_gimbal_control_cacl->gimbal_mode == GIMBAL_AUTO)
        {
            /* AUTO keeps exactly the same loops as GYRO.  Auto only changes
             * how the setpoints are generated, so entering AUTO cannot make
             * the pitch loop fight the 3508 lead screw. */
            hero_gimbal_control_cacl->yaw_gimbal_motor.absolute_gyro_set =
                pid_calc_gimbal(&hero_gimbal_control_cacl->yaw_gimbal_motor.pid_gimbal_angle,
                               hero_gimbal_control_cacl->yaw_gimbal_motor.absolute_angle,
                               hero_gimbal_control_cacl->yaw_gimbal_motor.absolute_angle_set);

            hero_gimbal_control_cacl->yaw_gimbal_motor.give_current =
                pid_calc(&hero_gimbal_control_cacl->yaw_gimbal_motor.pid_gimbal_speed,
                        hero_gimbal_control_cacl->yaw_gimbal_motor.absolute_gyro,
                        hero_gimbal_control_cacl->yaw_gimbal_motor.absolute_gyro_set);

            hero_gimbal_control_cacl->pitch_gimbal_motor.give_current =
                pid_calc(&hero_gimbal_control_cacl->pitch_gimbal_motor.pid_gimbal_screw_speed,
                        (fp32)hero_gimbal_control_cacl->pitch_gimbal_motor.gimbal_motor_measure->speed_rpm,
                        hero_gimbal_control_cacl->pitch_gimbal_motor.relative_omega_set);
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

/* pitch screw (3508 + C620): fixed speed from the remote stick, no angle loop */
static void Gimbal_Screw_Speed_Control(fp32 *rpm, gimbal_control_t *gimbal_control_set)
{
    static int16_t pitch_channel = 0;

    if (rpm == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    Remote_Deadband_Limit(gimbal_control_set->gimbal_dt7_remote_control->rc.ch[PITCH_CHANNEL],
                          pitch_channel, PITCH_SCREW_RC_THRESH);

    if (pitch_channel > 0)          /* stick up   -> forward */
    {
        *rpm =  PITCH_SCREW_RPM * PITCH_SCREW_DIR;
    }
    else if (pitch_channel < 0)     /* stick down -> reverse */
    {
        *rpm = -PITCH_SCREW_RPM * PITCH_SCREW_DIR;
    }
    else
    {
        *rpm = 0.0f;
    }
}

/**
 * @brief PITCH 行程限位（作用在最终电流，对所有模式生效）
 * @param g 云台控制结构体指针
 * @note 只削“朝限位外”的电流，往回永远放行；
 *       量程内不动，进入软区按距离线性缩小，到限位直接归零
 */
static void Gimbal_ApplyPitchTravelLimit(gimbal_control_t *g)
{
    fp32 measured = 0.0f;
    fp32 outward = 0.0f;
    fp32 scale = 1.0f;          /* 必须是自动变量，写成 static 会残留上一次的 0 */

    if (g == NULL)
    {
        return;
    }

#if PITCH_TRAVEL_ENABLE
    measured = g->pitch_gimbal_motor.absolute_angle;
    outward  = g->pitch_gimbal_motor.give_current * (fp32)PITCH_TRAVEL_DIR;

    if (measured >= (fp32)PITCH_TRAVEL_MAX_RAD)
    {
        if (outward > 0.0f)
        {
            scale = 0.0f;                                   /* 到上限，禁止继续往上 */
        }
    }
    else if (measured > (fp32)PITCH_TRAVEL_MAX_RAD - (fp32)PITCH_TRAVEL_SOFT_RAD)
    {
        if (outward > 0.0f)
        {
            scale = ((fp32)PITCH_TRAVEL_MAX_RAD - measured) / (fp32)PITCH_TRAVEL_SOFT_RAD;
        }
    }
    else if (measured <= (fp32)PITCH_TRAVEL_MIN_RAD)
    {
        if (outward < 0.0f)
        {
            scale = 0.0f;                                   /* 到下限，禁止继续往下 */
        }
    }
    else if (measured < (fp32)PITCH_TRAVEL_MIN_RAD + (fp32)PITCH_TRAVEL_SOFT_RAD)
    {
        if (outward < 0.0f)
        {
            scale = (measured - (fp32)PITCH_TRAVEL_MIN_RAD) / (fp32)PITCH_TRAVEL_SOFT_RAD;
        }
    }

    if (scale < 0.0f)       /* 实测已在限位外时比例会算成负值 */
    {
        scale = 0.0f;
    }

    if (scale < 1.0f)
    {
        g->pitch_gimbal_motor.give_current *= scale;
    }
#endif
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

/* ==================== YAW 轴软件限位与回位（实现） ==================== */

/**
 * @brief 把 ECD 换算成相对右极限的偏移
 * @return phi ∈ [0, 65535]，允许区间是 [0, YAW_ECD_SPAN]
 */
static int32_t Yaw_Ecd_Phi(uint16_t ecd)
{
    return ((int32_t)ecd - (int32_t)YAW_ECD_LIMIT_R + 65536) % 65536;
}

/**
 * @brief 反退启动
 * @param ecd 当前编码器值
 * @param dir +1 表示让 ECD 增大
 */
static void Yaw_Retreat_Start(uint16_t ecd, int8_t dir)
{
    yaw_retreat_on         = 1;
    yaw_retreat_done       = 0;
    yaw_retreat_dir        = dir;
    yaw_retreat_last       = ecd;
    yaw_retreat_acc        = 0;
    yaw_retreat_tick       = HAL_GetTick();
    yaw_retreat_stall_tick = HAL_GetTick();
}

/**
 * @brief 反退一步
 * @return 0 未启动/已完成，1 进行中，2 失败（反退方向也被挡住）
 */
static uint8_t Yaw_Retreat_Step(uint16_t ecd, int16_t *iq_out)
{
    int16_t d = 0;

    if (iq_out == NULL)
    {
        return 0;
    }
    *iq_out = 0;

    if (!yaw_retreat_on)
    {
        return 0;
    }

    d = (int16_t)(ecd - yaw_retreat_last);
    yaw_retreat_last = ecd;

    if (d >= YAW_RETREAT_STALL_CNT || d <= -YAW_RETREAT_STALL_CNT)
    {
        yaw_retreat_stall_tick = HAL_GetTick();
    }

    yaw_retreat_acc += (int16_t)(yaw_retreat_dir * d);
    if (yaw_retreat_acc < 0)
    {
        yaw_retreat_acc = 0;
    }

    /* 反退到位 */
    if (yaw_retreat_acc >= YAW_RETREAT_CNT)
    {
        yaw_retreat_on   = 0;
        yaw_retreat_done = 1;
        return 0;
    }

    /* 反退方向也卡住 */
    if (HAL_GetTick() - yaw_retreat_stall_tick > YAW_RETREAT_STALL_MS)
    {
        yaw_retreat_on   = 0;
        yaw_retreat_done = 1;
        return 2;
    }

    /* 反退总超时按失败处理 */
    if (HAL_GetTick() - yaw_retreat_tick > YAW_RETREAT_MS)
    {
        yaw_retreat_on   = 0;
        yaw_retreat_done = 1;
        return 2;
    }

    *iq_out = (int16_t)(YAW_RETREAT_IQ * yaw_retreat_dir * YAW_LIMIT_DIR);
    return 1;
}

/**
 * @brief YAW 轴越界回位
 * @note 越界后只回到「最近的限位边缘」，不回中点；
 *       只有开机首次越界、或运行中越界超过 YAW_ECD_DEEP 才触发，
 *       摇杆打到限位造成的几度超调不会触发
 * @return 1 正在保护流程中（调用方跳过正常控制），0 已回到区间内
 */
static uint8_t Gimbal_Yaw_Limit_Recover(uint16_t ecd, int16_t *iq_out)
{
    int32_t phi = 0;
    int32_t to_l = 0;
    int32_t to_r = 0;
    int32_t over = 0;
    int32_t dist = 0;
    int32_t d_ecd = 0;
    int16_t iq = 0;
    int8_t  dir = 0;
    uint8_t r = 0;
    uint8_t need = 0;

    if (iq_out == NULL)
    {
        return 0;
    }
    *iq_out = 0;

    /* 反馈未就绪：宁可不动，避免用上电初值的假 ECD 触发回位 */
    if (HAL_GetTick() - yaw_ecd_update_tick > YAW_ECD_LOST_MS)
    {
        return 1;
    }

    phi = Yaw_Ecd_Phi(ecd);

    /* 越界时：到两端边缘的距离，选近的 */
    if (phi > (int32_t)YAW_ECD_SPAN)
    {
        to_l = phi - (int32_t)YAW_ECD_SPAN;      /* 向 L 端：ECD 减小 */
        to_r = 65536 - phi;                      /* 向 R 端：ECD 增大 */
        over = (to_l < to_r) ? to_l : to_r;

        if (to_l <= to_r)
        {
            dir  = -1;
            dist = to_l;
        }
        else
        {
            dir  = +1;
            dist = to_r;
        }
    }

    /* 是否需要回位：开机首次越界，或越界很深 */
    if ((yaw_need_home && (over > (int32_t)YAW_ECD_TOL)) ||
        (over > (int32_t)YAW_ECD_DEEP))
    {
        need = 1;
    }

    /* 在区间内，或不在流程中且不需要回位 → 交还控制 */
    if ((over <= 0) ||
        (((yaw_limit_state == YAW_LIMIT_IDLE) || (yaw_limit_state == YAW_LIMIT_DONE)) && (!need)))
    {
        if (yaw_limit_state != YAW_LIMIT_DONE)
        {
            mini_gimbal.yaw_gimbal_motor.absolute_angle_set =
                mini_gimbal.yaw_gimbal_motor.absolute_angle;
            mini_gimbal.yaw_gimbal_motor.absolute_gyro_set = 0;
            pid_clear(&mini_gimbal.yaw_gimbal_motor.pid_gimbal_angle);
            pid_clear(&mini_gimbal.yaw_gimbal_motor.pid_gimbal_speed);
        }
        yaw_need_home   = 0;
        yaw_limit_state = YAW_LIMIT_DONE;
        return 0;
    }

    /* 启动回位 */
    if ((yaw_limit_state == YAW_LIMIT_IDLE) || (yaw_limit_state == YAW_LIMIT_DONE))
    {
        yaw_recover_try  = 0;
        yaw_backoff_ms   = 100;             // 先等 100ms，避开上电抖动
        yaw_backoff_tick = HAL_GetTick();
        yaw_limit_state  = YAW_LIMIT_BACKOFF;
        return 1;
    }

    /* 回位流程进行中 */
    switch (yaw_limit_state)
    {
    case YAW_LIMIT_BACKOFF:
        if (HAL_GetTick() - yaw_backoff_tick >= yaw_backoff_ms)
        {
            yaw_recover_tick       = HAL_GetTick();
            yaw_recover_ecd        = ecd;
            yaw_recover_stall_tick = HAL_GetTick();
            yaw_limit_state        = YAW_LIMIT_RECOVER;
        }
        break;

    case YAW_LIMIT_RECOVER:
        /* 已经贴到边缘：到位，交还控制 */
        if (dist <= 200)
        {
            mini_gimbal.yaw_gimbal_motor.absolute_angle_set =
                mini_gimbal.yaw_gimbal_motor.absolute_angle;
            mini_gimbal.yaw_gimbal_motor.absolute_gyro_set = 0;
            pid_clear(&mini_gimbal.yaw_gimbal_motor.pid_gimbal_angle);
            pid_clear(&mini_gimbal.yaw_gimbal_motor.pid_gimbal_speed);
            yaw_need_home   = 0;
            yaw_recover_try = 0;
            yaw_limit_state = YAW_LIMIT_DONE;
            return 0;
        }

        d_ecd = (int32_t)ecd - (int32_t)yaw_recover_ecd;
        if (d_ecd < 0)
        {
            d_ecd = -d_ecd;
        }
        if (d_ecd >= YAW_SAFE_STALL_CNT)
        {
            yaw_recover_ecd        = ecd;
            yaw_recover_stall_tick = HAL_GetTick();
        }

        /* 判定失败：堵转或超时 → 先反向松一下，再继续重试 */
        if ((HAL_GetTick() - yaw_recover_stall_tick > YAW_SAFE_STALL_MS) ||
            (HAL_GetTick() - yaw_recover_tick > YAW_SAFE_TIMEOUT_MS))
        {
            Yaw_Retreat_Start(ecd, (int8_t)(-dir));
            yaw_recover_try++;
            yaw_limit_state = YAW_LIMIT_RETREAT;
            break;
        }

        /* 末段减速 */
        iq = (dist > YAW_SAFE_SLOW)
           ? YAW_SAFE_IQ
           : (int16_t)((int32_t)YAW_SAFE_IQ * dist / YAW_SAFE_SLOW);

        *iq_out = (int16_t)(iq * dir * YAW_LIMIT_DIR);
        break;

    case YAW_LIMIT_RETREAT:
        r = Yaw_Retreat_Step(ecd, &iq);

        if (r == 1)
        {
            *iq_out = iq;                   // 还在反退
        }
        else
        {
            /* 反退完成或失败：一律退避后重试，不再锁死 */
            if (yaw_recover_try == 1)       yaw_backoff_ms = YAW_BACKOFF_MS_1;
            else if (yaw_recover_try == 2)  yaw_backoff_ms = YAW_BACKOFF_MS_2;
            else                            yaw_backoff_ms = YAW_BACKOFF_MS_3;

            yaw_backoff_tick = HAL_GetTick();
            yaw_limit_state  = YAW_LIMIT_BACKOFF;
        }
        break;

    default:
        break;
    }

    return 1;
}
/**
 * @brief YAW 轴设定值钳位 + 减速包络
 * @note 1) 设定值不落在禁止区；2) 接近限位时只限制“朝限位方向”的超前量，
 *       反方向（离开限位）完全不限制，保证摇杆随时能把云台拉回来
 */
static void Gimbal_Yaw_Limit_Setpoint(gimbal_control_t *g)
{
    uint16_t ecd = 0;
    fp32 phi = 0.0f;
    fp32 angle = 0.0f;
    fp32 set = 0.0f;
    fp32 dphi = 0.0f;
    fp32 dist = 0.0f;
    fp32 v_allow = 0.0f;
    fp32 ahead_max = 0.0f;

    if (g == NULL)
    {
        return;
    }

    ecd   = g->yaw_gimbal_motor.gimbal_motor_measure->ecd;
    phi   = (fp32)Yaw_Ecd_Phi(ecd);
    angle = g->yaw_gimbal_motor.absolute_angle;
    set   = g->yaw_gimbal_motor.absolute_angle_set;

    /* 设定值相对实际的角度差，折算成 ECD 计数并处理 ±π 回绕 */
    dphi = set - angle;
    if (dphi > PI)
    {
        dphi -= 2.0f * PI;
    }
    else if (dphi < -PI)
    {
        dphi += 2.0f * PI;
    }
    dphi = dphi / ECD_TO_RAD;

    /* 1) 位置钳位：设定值不能落在禁止区 */
    if (phi > (fp32)YAW_ECD_SPAN)
    {
        /* 越界：就近钳到某一端，且允许继续往里推
         * 这里必须按“离哪端近”分，不能只看 phi > SPAN：
         * 右端超调后 phi 会落在 65536 附近，误判成 L 端会把设定值甩出 180° */
        if ((phi - (fp32)YAW_ECD_SPAN) <= (65536.0f - phi))
        {
            /* 靠 L 端 */
            if (dphi > (fp32)YAW_ECD_SPAN - phi)
            {
                dphi = (fp32)YAW_ECD_SPAN - phi;
            }
        }
        else
        {
            /* 靠 R 端（phi 接近 65536） */
            if (dphi < 65536.0f - phi)
            {
                dphi = 65536.0f - phi;
            }
        }
    }
    else
    {
        if (phi + dphi > (fp32)YAW_ECD_SPAN)
        {
            dphi = (fp32)YAW_ECD_SPAN - phi;
        }
        if (phi + dphi < 0.0f)
        {
            dphi = -phi;
        }
    }

    /* 2) 减速包络：只限制朝限位方向的超前量，反方向不限制 */
    if ((YAW_ANGLE_KP > 0.1f) && (phi <= (fp32)YAW_ECD_SPAN))
    {
        if (dphi > 0.0f)
        {
            dist = (fp32)YAW_ECD_SPAN - phi;        // 冲向 L 端
            if (dist < (fp32)YAW_ECD_SLOW)
            {
                if (dist < 0.0f)
                {
                    dist = 0.0f;
                }
                v_allow   = YAW_SLOW_RATE * dist / (fp32)YAW_ECD_SLOW;
                ahead_max = v_allow / YAW_ANGLE_KP / ECD_TO_RAD;
                if (dphi > ahead_max)
                {
                    dphi = ahead_max;
                }
            }
        }
        else if (dphi < 0.0f)
        {
            dist = phi;                             // 冲向 R 端
            if (dist < (fp32)YAW_ECD_SLOW)
            {
                if (dist < 0.0f)
                {
                    dist = 0.0f;
                }
                v_allow   = YAW_SLOW_RATE * dist / (fp32)YAW_ECD_SLOW;
                ahead_max = v_allow / YAW_ANGLE_KP / ECD_TO_RAD;
                if (dphi < -ahead_max)
                {
                    dphi = -ahead_max;
                }
            }
        }
    }

    g->yaw_gimbal_motor.absolute_angle_set = angle + dphi * ECD_TO_RAD;
}
/**
 * @brief YAW 轴看门狗：0x141 丢失 / 堵转
 * @return 1 需要强制停机
 */
static uint8_t Gimbal_Yaw_Safe_Check(int16_t iq, int16_t speed)
{
    /* 0x141 丢失：限位判断失效，立即停机，恢复后自动继续 */
    if (HAL_GetTick() - yaw_ecd_update_tick > YAW_ECD_LOST_MS)
    {
        yaw_block_cnt = 0;
        return 1;
    }

    /* 堵转停机窗口 */
    if (yaw_stall_holding)
    {
        if (HAL_GetTick() - yaw_stall_hold_tick < YAW_STALL_HOLD_MS)
        {
            return 1;
        }
        yaw_stall_holding = 0;
        yaw_block_cnt = 0;
    }

    if ((iq > YAW_STALL_IQ || iq < -YAW_STALL_IQ) &&
        (speed < YAW_STALL_SPEED && speed > -YAW_STALL_SPEED))
    {
        if (yaw_block_cnt < YAW_STALL_MS)
        {
            yaw_block_cnt++;
        }
        if (yaw_block_cnt >= YAW_STALL_MS)
        {
            yaw_stall_holding   = 1;
            yaw_stall_hold_tick = HAL_GetTick();
            yaw_block_cnt       = 0;
            return 1;
        }
    }
    else
    {
        yaw_block_cnt = 0;
    }

    return 0;
}
