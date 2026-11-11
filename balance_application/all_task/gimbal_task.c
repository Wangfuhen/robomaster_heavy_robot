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
static void Gimbal_Initialize(gimbal_control_t *gimbal_control_state);          // 云台初始化
static void Gimbal_UpdateOperatingMode(gimbal_control_t *gimbal_control_state);  // 遥控器设置模式
static void Gimbal_HandleOperatingModeChange(gimbal_control_t *gimbal_control_state);  // 模式切换反馈
static void Gimbal_UpdateFeedback(gimbal_control_t *gimbal_control_state);    // 反馈更新
static void Gimbal_UpdateControlSetpoint(gimbal_control_t *gimbal_control_state);  // 模式切换控制
static void Gimbal_CalculateControlOutput(gimbal_control_t *gimbal_control_state);          // 控制计算

// 子函数声明
static fp32 Gimbal_ConvertEncoderCountToYawAngleRadian(uint16_t encoder_count,uint16_t encoder_zero_count);  // YAW轴编码器转角度
static fp32 Gimbal_ConvertEncoderCountToPitchRotorAngleRadian(uint16_t encoder_count,uint16_t encoder_zero_count);  // PITCH轴编码器转角度
static void Gimbal_CalculateZeroForceSetpoint(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_state);  // 无力控制
static void Gimbal_CalculateRelativeAngleSetpoint(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_state);  // 相对角度控制
static void Gimbal_CalculateAbsoluteAngleSetpoint(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_state);  // 绝对角度控制
static void Gimbal_CalculateAutomaticAimSetpoint(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_state);  // 自动控制
static void Gimbal_CalculatePitchScrewSpeedSetpoint(fp32 *rpm, gimbal_control_t *gimbal_control_state);  /* pitch screw fixed speed */

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
    vTaskDelay(gimbal_initialization_delay_millisecond);  // 延时等待系统初始化完成
    
    Gimbal_Initialize(&mini_gimbal);  // 云台初始化
    
    while(1)
    {
        // 云台控制主循环
        Gimbal_UpdateOperatingMode(&mini_gimbal);  // 设置云台控制模式
        Gimbal_HandleOperatingModeChange(&mini_gimbal);  // 模式切换反馈处理
        Gimbal_UpdateFeedback(&mini_gimbal);  // 更新反馈数据
        Gimbal_UpdateControlSetpoint(&mini_gimbal);  // 根据模式设置控制量
        Gimbal_CalculateControlOutput(&mini_gimbal);  // 控制计算
        
        // 发送控制指令到电机
        CAN_cmd_gimbal_yaw(mini_gimbal.yaw_axis_state.commanded_torque_current_count);
        CAN_SendPitchAxisTorqueCurrentCommand(mini_gimbal.pitch_axis_state.commanded_torque_current_count);
        
        vTaskDelay(gimbal_control_loop_period_millisecond);  // 控制周期延时
    }
}

/**
 * @brief 云台初始化函数
 * @param gimbal_control_state 云台控制结构体指针
 * @note 初始化云台的所有参数、PID控制器、获取传感器数据指针等
 */
static void Gimbal_Initialize(gimbal_control_t *gimbal_control_state)
{
    // 获取电机测量数据指针
	gimbal_control_state->yaw_axis_state.motor_measurement_pointer = GetYawAxisMotorMeasurementPointer();
	gimbal_control_state->pitch_axis_state.motor_measurement_pointer = GetPitchAxisMotorMeasurementPointer();
    
    // 获取IMU数据指针
	gimbal_control_state->gimbal_INS_angle_point = get_INS_angle_point();
	gimbal_control_state->gimbal_INS_gyro_point = get_gyro_data_point();
    
    // 获取遥控器数据指针
	gimbal_control_state->gimbal_dt7_remote_control = get_dt7_remote_control_point();
    
    // 初始化云台模式
	gimbal_control_state->gimbal_mode = GIMBAL_OPERATING_MODE_ZERO_FORCE;  // 默认无力模式
    gimbal_control_state->yaw_axis_state.motor_control_mode = MOTOR_CONTROL_MODE_ZERO_FORCE;
    gimbal_control_state->yaw_axis_state.previous_motor_control_mode = MOTOR_CONTROL_MODE_ZERO_FORCE;
    gimbal_control_state->pitch_axis_state.motor_control_mode = MOTOR_CONTROL_MODE_ZERO_FORCE;
    gimbal_control_state->pitch_axis_state.previous_motor_control_mode = MOTOR_CONTROL_MODE_ZERO_FORCE;
    
    // 初始化云台方向状态
	gimbal_control_state->heading_transition.heading_state = GIMBAL_HEADING_FORWARD;  // 默认前向
	gimbal_control_state->heading_transition.previous_heading_state = GIMBAL_HEADING_FORWARD;
    
    // 初始化Yaw轴零位
	gimbal_control_state->yaw_axis_state.zero_encoder_count = yaw_axis_zero_encoder_count;
    
    // 获取PC数据指针
	gimbal_control_state->hero_pc = get_crg_pc_point();
    
    // 初始化角度计算相关变量
	gimbal_control_state->yaw_axis_state.angle_feedforward_calculation.in = 0;
	gimbal_control_state->yaw_axis_state.angle_feedforward_calculation.last_in = 0;
    
    // 初始化YAW轴相对编码器PID控制器
	fp32 yaw_speed_relative[3] = {YAW_ANGLE_KP_RELATIVE, YAW_ANGLE_KI_RELATIVE, YAW_ANGLE_KD_RELATIVE};
    fp32 yaw_angle_relative[3] = {YAW_SPEED_KP_RELATIVE, YAW_SPEED_KI_RELATIVE, YAW_SPEED_KD_RELATIVE};
	pid_init(&gimbal_control_state->yaw_axis_state.relative_angle_controller, PID_POSITION, yaw_angle_relative, YAW_ANGLE_MAX_RELATIVE, YAW_ANGLE_I_MAX_RELATIVE);
	pid_init(&gimbal_control_state->yaw_axis_state.relative_speed_controller, PID_POSITION, yaw_speed_relative, YAW_SPEED_MAX_RELATIVE, YAW_SPEED_I_MAX_RELATIVE);
    
    // 初始化YAW轴陀螺仪PID控制器
	fp32 yaw_speed[3] = {YAW_SPEED_KP, YAW_SPEED_KI, YAW_SPEED_KD};
    fp32 yaw_angle[3] = {YAW_ANGLE_KP, YAW_ANGLE_KI, YAW_ANGLE_KD};
	pid_init(&gimbal_control_state->yaw_axis_state.angle_controller, PID_POSITION, yaw_angle, YAW_ANGLE_MAX, YAW_ANGLE_I_MAX);
	pid_init(&gimbal_control_state->yaw_axis_state.speed_controller, PID_POSITION, yaw_speed, YAW_SPEED_MAX, YAW_SPEED_I_MAX);
    
    // 初始化YAW轴自瞄PID控制器
    fp32 yaw_speed_ac[3] = {YAW_SPEED_KP_AC, YAW_SPEED_KI_AC, YAW_SPEED_KD_AC};
    fp32 yaw_angle_ac[3] = {YAW_ANGLE_KP_AC, YAW_ANGLE_KI_AC, YAW_ANGLE_KD_AC};
    pid_init(&gimbal_control_state->yaw_axis_state.automatic_aim_angle_controller, PID_POSITION, yaw_angle_ac, YAW_ANGLE_AC_MAX, YAW_ANGLE_I_AC_MAX);
	pid_init(&gimbal_control_state->yaw_axis_state.automatic_aim_speed_controller, PID_POSITION, yaw_speed_ac, YAW_SPEED_AC_MAX, YAW_SPEED_I_AC_MAX);
    
    // 初始化PITCH轴LQR控制器
    fp32 pitch_speed[4] = {pitch_axis_encoder_mode_speed_proportional_gain, pitch_axis_encoder_mode_speed_integral_gain, pitch_axis_speed_loop_maximum_integral_output, pitch_axis_speed_loop_maximum_output_count};
    fp32 pitch_angle[4] = {pitch_axis_encoder_mode_angle_proportional_gain, pitch_axis_encoder_mode_angle_integral_gain, pitch_axis_angle_loop_maximum_integral_output, pitch_axis_angle_loop_maximum_output_count};
    fp32 pitch_current[4] = {pitch_axis_encoder_mode_current_proportional_gain, 0, 0, pitch_axis_current_loop_maximum_output_count};
    lqr_init(&gimbal_control_state->pitch_axis_state.encoder_mode_controller, pitch_speed, pitch_angle, pitch_current, 30000);
    
    // 初始化PITCH轴自瞄LQR控制器
    fp32 pitch_speed_ac[4] = {pitch_axis_automatic_aim_encoder_mode_speed_proportional_gain, pitch_axis_automatic_aim_encoder_mode_speed_integral_gain, pitch_axis_speed_loop_maximum_integral_output, pitch_axis_speed_loop_maximum_output_count};
    fp32 pitch_angle_ac[4] = {pitch_axis_automatic_aim_encoder_mode_angle_proportional_gain, pitch_axis_automatic_aim_encoder_mode_angle_integral_gain, pitch_axis_angle_loop_maximum_integral_output, pitch_axis_angle_loop_maximum_output_count};
    fp32 pitch_current_ac[4] = {pitch_axis_automatic_aim_encoder_mode_current_proportional_gain, 0, 0, pitch_axis_current_loop_maximum_output_count};
    lqr_init(&gimbal_control_state->pitch_axis_state.automatic_aim_encoder_mode_controller, pitch_speed_ac, pitch_angle_ac, pitch_current_ac, 30000);
    
    // 初始化PITCH轴串级PID控制器
	static const fp32 Pitch_ABSOLUTE_speed_pid[3] = {pitch_axis_absolute_speed_proportional_gain, pitch_axis_absolute_speed_integral_gain, pitch_axis_absolute_speed_derivative_gain};
	static const fp32 Pitch_ABSOLUTE_angle_pid[3] = {pitch_axis_absolute_angle_proportional_gain, pitch_axis_absolute_angle_integral_gain, pitch_axis_absolute_angle_derivative_gain};
	pid_init(&gimbal_control_state->pitch_axis_state.angle_controller, PID_POSITION, Pitch_ABSOLUTE_angle_pid, pitch_axis_absolute_angle_maximum_output_radian_per_second, pitch_axis_absolute_angle_maximum_integral_output);
	pid_init(&gimbal_control_state->pitch_axis_state.speed_controller, PID_POSITION, Pitch_ABSOLUTE_speed_pid, pitch_axis_absolute_speed_maximum_output_count, pitch_axis_absolute_speed_maximum_integral_output);
	static const fp32 Pitch_SCREW_speed_pid[3] = {pitch_axis_screw_speed_proportional_gain, pitch_axis_screw_speed_integral_gain, pitch_axis_screw_speed_derivative_gain};
	pid_init(&gimbal_control_state->pitch_axis_state.screw_speed_controller, PID_POSITION, Pitch_SCREW_speed_pid, pitch_axis_screw_speed_maximum_output_count, pitch_axis_screw_speed_maximum_integral_output);
    
    // 初始化PITCH轴自瞄串级PID控制器
	static const fp32 Pitch_ABSOLUTE_speed_pid_ac[3] = {pitch_axis_automatic_aim_speed_proportional_gain, pitch_axis_automatic_aim_speed_integral_gain, pitch_axis_automatic_aim_speed_derivative_gain};
	static const fp32 Pitch_ABSOLUTE_angle_pid_ac[3] = {pitch_axis_automatic_aim_angle_proportional_gain, pitch_axis_automatic_aim_angle_integral_gain, pitch_axis_automatic_aim_angle_derivative_gain};
	pid_init(&gimbal_control_state->pitch_axis_state.automatic_aim_angle_controller, PID_POSITION, Pitch_ABSOLUTE_angle_pid_ac, pitch_axis_automatic_aim_angle_maximum_output_radian_per_second, pitch_axis_automatic_aim_angle_maximum_integral_output);
	pid_init(&gimbal_control_state->pitch_axis_state.automatic_aim_speed_controller, PID_POSITION, Pitch_ABSOLUTE_speed_pid_ac, pitch_axis_automatic_aim_speed_maximum_output_count, pitch_axis_automatic_aim_speed_maximum_integral_output);
    
    // 初始化角度设定值
	gimbal_control_state->pitch_axis_state.absolute_angle_setpoint_radian = gimbal_control_state->pitch_axis_state.relative_angle_setpoint_radian = 0;
	gimbal_control_state->yaw_axis_state.absolute_angle_setpoint_radian = gimbal_control_state->yaw_axis_state.relative_angle_setpoint_radian = 0;
    
    // 初始化云台方向变化相关参数
	gimbal_control_state->heading_transition.heading_state_change_flag = 0;
	gimbal_control_state->heading_transition.heading_state_change_time_millisecond = 0;
	gimbal_control_state->heading_transition.heading_state_chassis_time_millisecond = 0;
	gimbal_control_state->heading_transition.heading_state_chassis_change_flag = 0;
    
    // 更新反馈数据
    Gimbal_UpdateFeedback(gimbal_control_state);
}

/**
 * @brief 设置云台控制模式
 * @param gimbal_control_state 云台控制结构体指针
 * @note 根据遥控器开关状态设置云台的控制模式
 */
static void Gimbal_UpdateOperatingMode(gimbal_control_t *gimbal_control_state)
{
    if(gimbal_control_state == NULL)
    {
        return;
    }
    
    // 右拨杆向上：比赛模式
	if(switch_is_up(gimbal_control_state->gimbal_dt7_remote_control->rc.s[remote_control_right_switch]))
	{
		gimbal_control_state->gimbal_mode = GIMBAL_OPERATING_MODE_ABSOLUTE_ANGLE;  // 默认陀螺仪模式
		
        // 鼠标右键按下：自动瞄准模式
		if(gimbal_control_state->gimbal_dt7_remote_control->mouse.press_r == 1)
		{
			gimbal_control_state->gimbal_mode = GIMBAL_OPERATING_MODE_AUTOMATIC_AIM;
		}
		else  // 开启自瞄的时候不允许掉头
		{
            // 保存上一次的云台方向
			gimbal_control_state->heading_transition.previous_heading_state = gimbal_control_state->heading_transition.heading_state;
            
            // 处理云台掉头控制逻辑
            // 条件：通道4值>=300 或 按下E键 且 掉头标志为0 且 掉头等待时间满足
			if((gimbal_control_state->gimbal_dt7_remote_control->rc.ch[4] >= 300 || 
                gimbal_control_state->gimbal_dt7_remote_control->key.v & KEY_PRESSED_OFFSET_E) &&
                gimbal_control_state->heading_transition.heading_state_change_flag == 0 &&
                gimbal_control_state->heading_transition.heading_state_change_time_millisecond >= yaw_axis_reversal_wait_cycle_count/2)
		    {
                // 设置新的零位角度，切换为前向
				gimbal_control_state->yaw_axis_state.zero_encoder_count = yaw_axis_reversal_zero_encoder_count;
				gimbal_control_state->heading_transition.heading_state = GIMBAL_HEADING_REVERSED;
				gimbal_control_state->heading_transition.heading_state_change_flag = 1;
				gimbal_control_state->heading_transition.heading_state_change_time_millisecond = 0;
		    }
            // 条件：通道4值>=300 或 按下E键 且 掉头标志为1 且 掉头等待时间满足
			else if((gimbal_control_state->gimbal_dt7_remote_control->rc.ch[4] >= 300 || 
                     gimbal_control_state->gimbal_dt7_remote_control->key.v & KEY_PRESSED_OFFSET_E) &&
                    gimbal_control_state->heading_transition.heading_state_change_flag == 1 &&
                    gimbal_control_state->heading_transition.heading_state_change_time_millisecond >= yaw_axis_reversal_wait_cycle_count/2)
			{
                // 恢复原始零位角度，切换为正常前向
				gimbal_control_state->yaw_axis_state.zero_encoder_count = yaw_axis_zero_encoder_count;
				gimbal_control_state->heading_transition.heading_state = GIMBAL_HEADING_FORWARD;
				gimbal_control_state->heading_transition.heading_state_change_flag = 0;
				gimbal_control_state->heading_transition.heading_state_change_time_millisecond = 0;
			}
            
            // 更新掉头时间计数器，防止溢出
			if(gimbal_control_state->heading_transition.heading_state_change_time_millisecond <= yaw_axis_reversal_wait_cycle_count)
			{
				gimbal_control_state->heading_transition.heading_state_change_time_millisecond++;
			}
			else
			{
				gimbal_control_state->heading_transition.heading_state_change_time_millisecond = yaw_axis_reversal_wait_cycle_count;
			}
		}
	}
	else  // 调试模式
	{
        // 根据右拨杆位置设置模式
		if(switch_is_down(gimbal_control_state->gimbal_dt7_remote_control->rc.s[remote_control_right_switch]))
		{
			gimbal_control_state->gimbal_mode = GIMBAL_OPERATING_MODE_ABSOLUTE_ANGLE;
		}
		else if(switch_is_mid(gimbal_control_state->gimbal_dt7_remote_control->rc.s[remote_control_right_switch]))
		{
			gimbal_control_state->gimbal_mode = GIMBAL_OPERATING_MODE_ABSOLUTE_ANGLE;
		}
        
        // 调试模式下的自瞄控制
		if(gimbal_control_state->gimbal_mode == GIMBAL_OPERATING_MODE_ABSOLUTE_ANGLE)
		{
			if(switch_is_down(gimbal_control_state->gimbal_dt7_remote_control->rc.s[remote_control_left_switch]) || 
               switch_is_mid(gimbal_control_state->gimbal_dt7_remote_control->rc.s[remote_control_left_switch]))
			{
				gimbal_control_state->gimbal_mode = GIMBAL_OPERATING_MODE_ABSOLUTE_ANGLE;
			}
			else
			{
				gimbal_control_state->gimbal_mode = GIMBAL_OPERATING_MODE_AUTOMATIC_AIM;
			}
		}
	}
    
    // 根据云台模式设置电机模式
    if(gimbal_control_state->gimbal_mode == GIMBAL_OPERATING_MODE_ZERO_FORCE)
    {
        gimbal_control_state->yaw_axis_state.motor_control_mode = MOTOR_CONTROL_MODE_ZERO_FORCE;
        gimbal_control_state->pitch_axis_state.motor_control_mode = MOTOR_CONTROL_MODE_ZERO_FORCE;
    }
    else if(gimbal_control_state->gimbal_mode == GIMBAL_OPERATING_MODE_ENCODER_ANGLE)
    {
        gimbal_control_state->yaw_axis_state.motor_control_mode = MOTOR_CONTROL_MODE_ENCODER_ANGLE;
        gimbal_control_state->pitch_axis_state.motor_control_mode = MOTOR_CONTROL_MODE_ENCODER_ANGLE;
    }
    else if(gimbal_control_state->gimbal_mode == GIMBAL_OPERATING_MODE_ABSOLUTE_ANGLE || gimbal_control_state->gimbal_mode == GIMBAL_OPERATING_MODE_AUTOMATIC_AIM)
    {
        gimbal_control_state->yaw_axis_state.motor_control_mode = MOTOR_CONTROL_MODE_ABSOLUTE_ANGLE;
        gimbal_control_state->pitch_axis_state.motor_control_mode = MOTOR_CONTROL_MODE_ABSOLUTE_ANGLE;
    }
}

/**
 * @brief 模式切换反馈处理
 * @param gimbal_control_state 云台控制结构体指针
 * @note 处理模式切换时的初始值设置，避免突变
 */
static void Gimbal_HandleOperatingModeChange(gimbal_control_t *gimbal_control_state)
{
    if(gimbal_control_state == NULL)
    {
        return;
    }

    uint8_t pitch_mode_changed =
        (gimbal_control_state->pitch_axis_state.motor_control_mode !=
         gimbal_control_state->pitch_axis_state.previous_motor_control_mode);
    uint8_t yaw_mode_changed =
        (gimbal_control_state->yaw_axis_state.motor_control_mode !=
         gimbal_control_state->yaw_axis_state.previous_motor_control_mode);

    if(pitch_mode_changed || yaw_mode_changed)
    {
        // 任何模式切换：设定值对齐当前反馈，清空所有控制器历史，保证无扰
        gimbal_control_state->pitch_axis_state.absolute_angle_setpoint_radian =
            gimbal_control_state->pitch_axis_state.absolute_angle_radian;
        gimbal_control_state->yaw_axis_state.absolute_angle_setpoint_radian =
            gimbal_control_state->yaw_axis_state.absolute_angle_radian;
        gimbal_control_state->pitch_axis_state.relative_angle_setpoint_radian =
            gimbal_control_state->pitch_axis_state.relative_angle_radian;
        gimbal_control_state->yaw_axis_state.relative_angle_setpoint_radian =
            gimbal_control_state->yaw_axis_state.relative_angle_radian;

        gimbal_control_state->pitch_axis_state.pitch_axis_screw_target_rotor_speed_revolution_per_minute = 0.0f;
        gimbal_control_state->pitch_axis_state.relative_angular_velocity_setpoint_radian_per_second = 0.0f;
        gimbal_control_state->pitch_axis_state.absolute_angular_velocity_setpoint_radian_per_second = 0.0f;
        gimbal_control_state->yaw_axis_state.relative_angular_velocity_setpoint_radian_per_second = 0.0f;
        gimbal_control_state->yaw_axis_state.absolute_angular_velocity_setpoint_radian_per_second = 0.0f;

        pid_clear(&gimbal_control_state->pitch_axis_state.angle_controller);
        pid_clear(&gimbal_control_state->pitch_axis_state.speed_controller);
        pid_clear(&gimbal_control_state->pitch_axis_state.screw_speed_controller);
        pid_clear(&gimbal_control_state->pitch_axis_state.relative_angle_controller);
        pid_clear(&gimbal_control_state->pitch_axis_state.relative_speed_controller);
        pid_clear(&gimbal_control_state->pitch_axis_state.automatic_aim_angle_controller);
        pid_clear(&gimbal_control_state->pitch_axis_state.automatic_aim_speed_controller);

        pid_clear(&gimbal_control_state->yaw_axis_state.angle_controller);
        pid_clear(&gimbal_control_state->yaw_axis_state.speed_controller);
        pid_clear(&gimbal_control_state->yaw_axis_state.relative_angle_controller);
        pid_clear(&gimbal_control_state->yaw_axis_state.relative_speed_controller);
        pid_clear(&gimbal_control_state->yaw_axis_state.automatic_aim_angle_controller);
        pid_clear(&gimbal_control_state->yaw_axis_state.automatic_aim_speed_controller);
    }

    // 保存当前模式为上一次模式
	gimbal_control_state->pitch_axis_state.previous_motor_control_mode = 
        gimbal_control_state->pitch_axis_state.motor_control_mode;
	gimbal_control_state->yaw_axis_state.previous_motor_control_mode = 
        gimbal_control_state->yaw_axis_state.motor_control_mode;
}

/**
 * @brief 更新反馈数据
 * @param gimbal_control_state 云台控制结构体指针
 * @note 从传感器读取并更新云台的角度、角速度等反馈数据
 */
static void Gimbal_UpdateFeedback(gimbal_control_t *gimbal_control_state)
{
    if(gimbal_control_state == NULL)
    {
        return;
    }
    
    // 更新PITCH轴反馈数据
    gimbal_control_state->pitch_axis_state.relative_angle_radian = 
        Gimbal_ConvertEncoderCountToPitchRotorAngleRadian(gimbal_control_state->pitch_axis_state.motor_measurement_pointer->encoder_count, pitch_axis_zero_encoder_count);
    
    gimbal_control_state->pitch_axis_state.absolute_angle_radian = 
        *(gimbal_control_state->gimbal_INS_angle_point + INS_PITCH_ADDRESS_OFFSET);
    
	gimbal_control_state->pitch_axis_state.absolute_angular_velocity_radian_per_second = 
        pitch_axis_inertial_measurement_angular_velocity_sign *
        *(gimbal_control_state->gimbal_INS_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);
    
	gimbal_control_state->pitch_axis_state.relative_angular_velocity_radian_per_second = 
        (gimbal_control_state->pitch_axis_state.motor_measurement_pointer->rotor_speed_revolution_per_minute * motor_rotor_revolution_per_minute_to_radian_per_second);
    
    // 更新YAW轴反馈数据
    gimbal_control_state->yaw_axis_state.relative_angle_radian = 
        Gimbal_ConvertEncoderCountToYawAngleRadian(gimbal_control_state->yaw_axis_state.motor_measurement_pointer->encoder_count, 
                              gimbal_control_state->yaw_axis_state.zero_encoder_count);
    
	gimbal_control_state->yaw_axis_state.relative_angular_velocity_radian_per_second = 
        gimbal_control_state->yaw_axis_state.motor_measurement_pointer->rotor_speed_revolution_per_minute * motor_rotor_revolution_per_minute_to_radian_per_second;
    
    gimbal_control_state->yaw_axis_state.absolute_angle_radian = 
        *(gimbal_control_state->gimbal_INS_angle_point + INS_YAW_ADDRESS_OFFSET);
    
	gimbal_control_state->yaw_axis_state.absolute_angular_velocity_radian_per_second = 
        *(gimbal_control_state->gimbal_INS_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET);
}

/**
 * @brief 限制 pitch 设定值每个控制周期的最大变化量
 * @param increment_radian 本周期期望的角度增量
 * @return 限幅后的角度增量
 * @note 避免设定值跑得比执行器快，解决“松手后追目标、过冲”的问题
 */
static fp32 Gimbal_LimitPitchAngleIncrementRadian(fp32 increment_radian)
{
    const fp32 maximum_increment_radian =
        pitch_axis_setpoint_rate_limit_radian_per_second *
        ((fp32)gimbal_control_loop_period_millisecond / 1000.0f);

    if(increment_radian > maximum_increment_radian)
    {
        return maximum_increment_radian;
    }
    if(increment_radian < -maximum_increment_radian)
    {
        return -maximum_increment_radian;
    }
    return increment_radian;
}

/**
 * @brief 根据模式设置控制输入
 * @param gimbal_control_state 云台控制结构体指针
 * @note 根据不同的控制模式计算控制增量，并更新设定值
 */
static void Gimbal_UpdateControlSetpoint(gimbal_control_t *gimbal_control_state)
{
    if(gimbal_control_state == NULL)
    {
        return;
    }
    
    fp32 yaw_angle_increment_radian = 0.0f;    // YAW轴角度增量
    fp32 pitch_angle_increment_radian = 0.0f;   // PITCH轴角度增量
    fp32 pitch_target_rotor_speed_revolution_per_minute = 0.0f;     /* pitch screw target rpm */
    
    // 根据不同模式调用对应的控制函数
    if(gimbal_control_state->gimbal_mode == GIMBAL_OPERATING_MODE_ZERO_FORCE)
    {
        Gimbal_CalculateZeroForceSetpoint(&yaw_angle_increment_radian, &pitch_angle_increment_radian, gimbal_control_state);
    }
    else if(gimbal_control_state->gimbal_mode == GIMBAL_OPERATING_MODE_ENCODER_ANGLE)
    {
        // 编码器模式：相对角度控制
        Gimbal_CalculateRelativeAngleSetpoint(&yaw_angle_increment_radian, &pitch_angle_increment_radian, gimbal_control_state);
        pitch_angle_increment_radian = Gimbal_LimitPitchAngleIncrementRadian(pitch_angle_increment_radian);
        
        // 更新相对角度设定值
        gimbal_control_state->pitch_axis_state.relative_angle_setpoint_radian += pitch_angle_increment_radian;
        gimbal_control_state->yaw_axis_state.relative_angle_setpoint_radian += yaw_angle_increment_radian;
        
        // YAW轴角度限幅 [-π, π]
        if(gimbal_control_state->yaw_axis_state.relative_angle_setpoint_radian > pi)
            gimbal_control_state->yaw_axis_state.relative_angle_setpoint_radian -= 2 * pi;
        else if(gimbal_control_state->yaw_axis_state.relative_angle_setpoint_radian < -pi)
            gimbal_control_state->yaw_axis_state.relative_angle_setpoint_radian += 2 * pi;
        
        // PITCH轴角度限幅
        if(gimbal_control_state->pitch_axis_state.relative_angle_setpoint_radian > pitch_axis_relative_angle_maximum_radian)
            gimbal_control_state->pitch_axis_state.relative_angle_setpoint_radian = pitch_axis_relative_angle_maximum_radian;
        else if(gimbal_control_state->pitch_axis_state.relative_angle_setpoint_radian < pitch_axis_relative_angle_minimum_radian)
            gimbal_control_state->pitch_axis_state.relative_angle_setpoint_radian = pitch_axis_relative_angle_minimum_radian;
    }
    else if(gimbal_control_state->gimbal_mode == GIMBAL_OPERATING_MODE_ABSOLUTE_ANGLE)
    {
        // 陀螺仪模式：绝对角度控制
        Gimbal_CalculateAbsoluteAngleSetpoint(&yaw_angle_increment_radian, &pitch_angle_increment_radian, gimbal_control_state);
        
        // 更新绝对角度设定值
        Gimbal_CalculatePitchScrewSpeedSetpoint(&pitch_target_rotor_speed_revolution_per_minute, gimbal_control_state);
        gimbal_control_state->pitch_axis_state.pitch_axis_screw_target_rotor_speed_revolution_per_minute = pitch_target_rotor_speed_revolution_per_minute;
        // 角度设定值跟随实测，保证从 GYRO 切到 AUTO 时不跳变
        gimbal_control_state->pitch_axis_state.absolute_angle_setpoint_radian =
            gimbal_control_state->pitch_axis_state.absolute_angle_radian;
        gimbal_control_state->yaw_axis_state.absolute_angle_setpoint_radian += yaw_angle_increment_radian;
        
        // 处理云台掉头逻辑
		if(gimbal_control_state->heading_transition.heading_state != 
           gimbal_control_state->heading_transition.previous_heading_state)
		{
			if(gimbal_control_state->heading_transition.heading_state == GIMBAL_HEADING_FORWARD)
			{
                // 从掉头状态恢复：加π
				gimbal_control_state->yaw_axis_state.absolute_angle_setpoint_radian = 
                    gimbal_control_state->yaw_axis_state.absolute_angle_setpoint_radian + pi;
			}
			else if(gimbal_control_state->heading_transition.heading_state == GIMBAL_HEADING_REVERSED)
			{
                // 进入掉头状态：减π
				gimbal_control_state->yaw_axis_state.absolute_angle_setpoint_radian = 
                    gimbal_control_state->yaw_axis_state.absolute_angle_setpoint_radian - pi;
			}
		}
        
        // YAW轴角度限幅 [-π, π]
        if(gimbal_control_state->yaw_axis_state.absolute_angle_setpoint_radian > pi)
            gimbal_control_state->yaw_axis_state.absolute_angle_setpoint_radian -= 2 * pi;
        else if(gimbal_control_state->yaw_axis_state.absolute_angle_setpoint_radian < -pi)
            gimbal_control_state->yaw_axis_state.absolute_angle_setpoint_radian += 2 * pi;
        
    }
    else if(gimbal_control_state->gimbal_mode == GIMBAL_OPERATING_MODE_AUTOMATIC_AIM)
    {
        // 自动瞄准模式
        if(gimbal_control_state->hero_pc->pc_fire_mode == 1 || 
           gimbal_control_state->hero_pc->pc_fire_mode == 2)  // 有目标
        {
            // 自动控制：使用PC发送的目标角度，按最大速率逼近，避免阶跃
            Gimbal_CalculateAutomaticAimSetpoint(&yaw_angle_increment_radian, &pitch_angle_increment_radian, gimbal_control_state);
            {
                fp32 angle_difference_radian =
                    pitch_angle_increment_radian -
                    gimbal_control_state->pitch_axis_state.absolute_angle_setpoint_radian;
                gimbal_control_state->pitch_axis_state.absolute_angle_setpoint_radian +=
                    Gimbal_LimitPitchAngleIncrementRadian(angle_difference_radian);
            }
            gimbal_control_state->yaw_axis_state.absolute_angle_setpoint_radian = yaw_angle_increment_radian;
        }
        else
        {
            // 无目标时使用绝对角度控制
            Gimbal_CalculateAbsoluteAngleSetpoint(&yaw_angle_increment_radian, &pitch_angle_increment_radian, gimbal_control_state);
            gimbal_control_state->pitch_axis_state.absolute_angle_setpoint_radian +=
                Gimbal_LimitPitchAngleIncrementRadian(pitch_angle_increment_radian);
            gimbal_control_state->yaw_axis_state.absolute_angle_setpoint_radian += yaw_angle_increment_radian;
        }
        
        // 角度限幅
        if(gimbal_control_state->yaw_axis_state.absolute_angle_setpoint_radian > pi)
            gimbal_control_state->yaw_axis_state.absolute_angle_setpoint_radian -= 2 * pi;
        else if(gimbal_control_state->yaw_axis_state.absolute_angle_setpoint_radian < -pi)
            gimbal_control_state->yaw_axis_state.absolute_angle_setpoint_radian += 2 * pi;
        
        if(gimbal_control_state->pitch_axis_state.absolute_angle_setpoint_radian > pitch_axis_absolute_angle_maximum_radian)
            gimbal_control_state->pitch_axis_state.absolute_angle_setpoint_radian = pitch_axis_absolute_angle_maximum_radian;
        else if(gimbal_control_state->pitch_axis_state.absolute_angle_setpoint_radian < pitch_axis_absolute_angle_minimum_radian)
            gimbal_control_state->pitch_axis_state.absolute_angle_setpoint_radian = pitch_axis_absolute_angle_minimum_radian;
    }
}

/**
 * @brief 控制计算
 * @param gimbal_control_state 云台控制结构体指针
 * @note 根据电机模式和控制模式计算最终的控制电流
 */
static void Gimbal_CalculateControlOutput(gimbal_control_t *gimbal_control_state)
{
    const motor_measurement_t *pitch_measurement =
        gimbal_control_state->pitch_axis_state.motor_measurement_pointer;

    // 电调报错或反馈超时：切断输出，避免按旧数据继续积分
    if(pitch_measurement->controller_error_code != 0U ||
       (HAL_GetTick() - pitch_measurement->last_update_time_millisecond) >
           pitch_axis_feedback_timeout_millisecond)
    {
        gimbal_control_state->pitch_axis_state.commanded_torque_current_count = 0.0f;
        gimbal_control_state->yaw_axis_state.commanded_torque_current_count = 0.0f;
        return;
    }

    // 无力模式
    if(gimbal_control_state->yaw_axis_state.motor_control_mode == MOTOR_CONTROL_MODE_ZERO_FORCE)
    {
        gimbal_control_state->yaw_axis_state.commanded_torque_current_count = 0;
        gimbal_control_state->pitch_axis_state.commanded_torque_current_count = 0;
    }
    // 编码器模式
    else if(gimbal_control_state->yaw_axis_state.motor_control_mode == MOTOR_CONTROL_MODE_ENCODER_ANGLE)
    {
        // YAW轴：相对角度PID控制
		gimbal_control_state->yaw_axis_state.relative_angular_velocity_setpoint_radian_per_second = 
            pid_calc_gimbal(&gimbal_control_state->yaw_axis_state.relative_angle_controller,
                           gimbal_control_state->yaw_axis_state.relative_angle_radian,
                           gimbal_control_state->yaw_axis_state.relative_angle_setpoint_radian);
        
		gimbal_control_state->yaw_axis_state.commanded_torque_current_count = 
            pid_calc(&gimbal_control_state->yaw_axis_state.relative_speed_controller,
                    gimbal_control_state->yaw_axis_state.relative_angular_velocity_radian_per_second,
                    gimbal_control_state->yaw_axis_state.relative_angular_velocity_setpoint_radian_per_second);
        
        // PITCH轴：LQR控制
		fp32 pitch_feedback_encoder[3] = {
            (fp32) gimbal_control_state->pitch_axis_state.relative_angular_velocity_radian_per_second,
            (fp32) gimbal_control_state->pitch_axis_state.relative_angle_radian,
            (fp32) gimbal_control_state->pitch_axis_state.motor_measurement_pointer->measured_torque_current_count
        };
        
		fp32 pitch_setpoint_encoder[3] = {
            0,
            (fp32) gimbal_control_state->pitch_axis_state.relative_angle_setpoint_radian,
            0
        };
        
		gimbal_control_state->pitch_axis_state.commanded_torque_current_count = 
            lqr_cal(&gimbal_control_state->pitch_axis_state.encoder_mode_controller,
                   pitch_feedback_encoder,
                   pitch_setpoint_encoder);
    }
    // 陀螺仪模式
    else if(gimbal_control_state->yaw_axis_state.motor_control_mode == MOTOR_CONTROL_MODE_ABSOLUTE_ANGLE)
    {
        // 陀螺仪反馈数据准备
		fp32 pitch_feedback_absolute[3] = { 
            (fp32) gimbal_control_state->pitch_axis_state.absolute_angular_velocity_radian_per_second,
            (fp32) gimbal_control_state->pitch_axis_state.absolute_angle_radian,
            (fp32) gimbal_control_state->pitch_axis_state.motor_measurement_pointer->measured_torque_current_count
        };
        
		fp32 pitch_setpoint_absolute[3] = { 
            0,
            (fp32) gimbal_control_state->pitch_axis_state.absolute_angle_setpoint_radian,
            0
        };
        
        // 普通陀螺仪模式
        if(gimbal_control_state->gimbal_mode == GIMBAL_OPERATING_MODE_ABSOLUTE_ANGLE)
        {
            // YAW轴：串级PID控制
			gimbal_control_state->yaw_axis_state.absolute_angular_velocity_setpoint_radian_per_second = 
                pid_calc_gimbal(&gimbal_control_state->yaw_axis_state.angle_controller,
                               gimbal_control_state->yaw_axis_state.absolute_angle_radian,
                               gimbal_control_state->yaw_axis_state.absolute_angle_setpoint_radian);
            
			gimbal_control_state->yaw_axis_state.commanded_torque_current_count = 
                pid_calc(&gimbal_control_state->yaw_axis_state.speed_controller,
                        gimbal_control_state->yaw_axis_state.absolute_angular_velocity_radian_per_second,
                        gimbal_control_state->yaw_axis_state.absolute_angular_velocity_setpoint_radian_per_second);
            
		/* PITCH: 单速度环（3508 + C620 + 丝杆），目标来自电机坐标系的转速设定 */
		gimbal_control_state->pitch_axis_state.commanded_torque_current_count =
			pid_calc(&gimbal_control_state->pitch_axis_state.screw_speed_controller,
				(fp32)gimbal_control_state->pitch_axis_state.motor_measurement_pointer->rotor_speed_revolution_per_minute,
				gimbal_control_state->pitch_axis_state.pitch_axis_screw_target_rotor_speed_revolution_per_minute);
        }
        // 自动瞄准模式
        else if(gimbal_control_state->gimbal_mode == GIMBAL_OPERATING_MODE_AUTOMATIC_AIM)
        {
            // YAW轴：自瞄模式串级PID控制
			gimbal_control_state->yaw_axis_state.absolute_angular_velocity_setpoint_radian_per_second = 
                pid_calc_gimbal(&gimbal_control_state->yaw_axis_state.automatic_aim_angle_controller,
                               gimbal_control_state->yaw_axis_state.absolute_angle_radian,
                               gimbal_control_state->yaw_axis_state.absolute_angle_setpoint_radian);
            
			gimbal_control_state->yaw_axis_state.commanded_torque_current_count = 
                pid_calc(&gimbal_control_state->yaw_axis_state.automatic_aim_speed_controller,
                        gimbal_control_state->yaw_axis_state.absolute_angular_velocity_radian_per_second,
                        gimbal_control_state->yaw_axis_state.absolute_angular_velocity_setpoint_radian_per_second);
            
            // PITCH轴：自瞄模式串级PID控制
			gimbal_control_state->pitch_axis_state.absolute_angular_velocity_setpoint_radian_per_second = 
               pid_calc(&gimbal_control_state->pitch_axis_state.automatic_aim_angle_controller,
                        gimbal_control_state->pitch_axis_state.absolute_angle_radian,
                        gimbal_control_state->pitch_axis_state.absolute_angle_setpoint_radian);
            
			gimbal_control_state->pitch_axis_state.commanded_torque_current_count =
                pitch_axis_motor_rotation_to_joint_angle_sign *
                pid_calc(&gimbal_control_state->pitch_axis_state.automatic_aim_speed_controller,
                        gimbal_control_state->pitch_axis_state.absolute_angular_velocity_radian_per_second,
                        gimbal_control_state->pitch_axis_state.absolute_angular_velocity_setpoint_radian_per_second);
        }
    }
}

/**
 * @brief YAW轴编码器值转换为角度
 * @param encoder_count 编码器原始值
 * @param encoder_zero_count 编码器零位
 * @return 计算得到的角度值（弧度）
 * @note 处理编码器溢出，将编码器值转换为弧度
 */
static fp32 Gimbal_ConvertEncoderCountToYawAngleRadian(uint16_t encoder_count, uint16_t encoder_zero_count)
{
    int32_t relative_encoder_count = (int32_t)encoder_count - (int32_t)encoder_zero_count;

    // 9025 单圈 65536 计数解绕
    if(relative_encoder_count > yaw_axis_half_encoder_count_range)
    {
        relative_encoder_count -= yaw_axis_encoder_count_per_revolution;
    }
    else if(relative_encoder_count < -yaw_axis_half_encoder_count_range)
    {
        relative_encoder_count += yaw_axis_encoder_count_per_revolution;
    }

    return (fp32)relative_encoder_count * yaw_axis_encoder_count_to_radian;  // 转换为弧度
}

/**
 * @brief PITCH轴编码器值转换为角度
 * @param encoder_count 编码器原始值
 * @param encoder_zero_count 编码器零位
 * @return 计算得到的角度值（弧度）
 * @note 处理编码器溢出，将编码器值转换为弧度
 */
static fp32 Gimbal_ConvertEncoderCountToPitchRotorAngleRadian(uint16_t encoder_count, uint16_t encoder_zero_count)
{
    int32_t relative_encoder_count = (int32_t)encoder_count - (int32_t)encoder_zero_count;

    // 按 M3508 单圈 8192 计数解绕（原先误用了 9025 的 32768，永远不触发）
    if(relative_encoder_count > pitch_axis_half_encoder_count_range)
    {
        relative_encoder_count -= pitch_axis_encoder_count_per_revolution;
    }
    else if(relative_encoder_count < -pitch_axis_half_encoder_count_range)
    {
        relative_encoder_count += pitch_axis_encoder_count_per_revolution;
    }

    return (fp32)relative_encoder_count * pitch_axis_encoder_count_to_rotor_radian;  // 转子角，不是关节角
}

/**
 * @brief 获取云台控制结构体指针
 * @return 云台控制结构体指针
 * @note 提供全局云台控制实例的访问接口
 */
extern gimbal_control_t *GetGimbalControlStatePointer(void)
{
    return &mini_gimbal;
}

/**
 * @brief 无力控制函数
 * @param yaw YAW轴角度增量输出
 * @param pitch PITCH轴角度增量输出
 * @param gimbal_control_state 云台控制结构体指针
 * @note 在无力模式下，输出为零
 */
static void Gimbal_CalculateZeroForceSetpoint(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_state)
{
	if (yaw == NULL || pitch == NULL || gimbal_control_state == NULL)
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
 * @param gimbal_control_state 云台控制结构体指针
 * @note 根据遥控器通道和鼠标输入计算角度增量
 */
static void Gimbal_CalculateRelativeAngleSetpoint(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_state)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_state == NULL)
    {
        return;
    }

    static int16_t yaw_channel = 0, pitch_channel = 0;
    
    // 遥控器通道死区处理
    Remote_Deadband_Limit(gimbal_control_state->gimbal_dt7_remote_control->rc.ch[yaw_axis_remote_control_channel], yaw_channel, remote_control_deadband_count);
    Remote_Deadband_Limit(gimbal_control_state->gimbal_dt7_remote_control->rc.ch[pitch_axis_remote_control_channel], pitch_channel, remote_control_deadband_count);
    
    // 计算角度增量：遥控器通道 + 鼠标输入
    *yaw = yaw_channel * yaw_axis_remote_control_sensitivity_radian_per_count - gimbal_control_state->gimbal_dt7_remote_control->mouse.x * yaw_axis_mouse_sensitivity_radian_per_count;
    *pitch = pitch_channel * pitch_axis_remote_control_sensitivity_radian_per_count + gimbal_control_state->gimbal_dt7_remote_control->mouse.y * pitch_axis_mouse_sensitivity_radian_per_count;
}

/**
 * @brief 绝对角度控制函数
 * @param yaw YAW轴角度增量输出
 * @param pitch PITCH轴角度增量输出
 * @param gimbal_control_state 云台控制结构体指针
 * @note 根据遥控器通道和鼠标输入计算角度增量
 */
static void Gimbal_CalculateAbsoluteAngleSetpoint(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_state)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_state == NULL)
    {
        return;
    }

    static int16_t yaw_channel = 0, pitch_channel = 0;
    
    // 遥控器通道死区处理
    Remote_Deadband_Limit(gimbal_control_state->gimbal_dt7_remote_control->rc.ch[yaw_axis_remote_control_channel], yaw_channel, remote_control_deadband_count);
    Remote_Deadband_Limit(gimbal_control_state->gimbal_dt7_remote_control->rc.ch[pitch_axis_remote_control_channel], pitch_channel, remote_control_deadband_count);
    
    // 计算角度增量：遥控器通道 + 鼠标输入
    *yaw = yaw_channel * yaw_axis_remote_control_sensitivity_radian_per_count - gimbal_control_state->gimbal_dt7_remote_control->mouse.x * yaw_axis_mouse_sensitivity_radian_per_count;
    *pitch = pitch_channel * pitch_axis_remote_control_sensitivity_radian_per_count + gimbal_control_state->gimbal_dt7_remote_control->mouse.y * pitch_axis_mouse_sensitivity_radian_per_count;
}

/* pitch screw (3508 + C620): fixed speed from the remote stick, no angle loop */
static void Gimbal_CalculatePitchScrewSpeedSetpoint(fp32 *rpm, gimbal_control_t *gimbal_control_state)
{
    static int16_t pitch_channel = 0;

    if (rpm == NULL || gimbal_control_state == NULL)
    {
        return;
    }

    Remote_Deadband_Limit(gimbal_control_state->gimbal_dt7_remote_control->rc.ch[pitch_axis_remote_control_channel],
                          pitch_channel, pitch_axis_screw_stick_threshold_count);

    if (pitch_channel > 0)          /* stick up   -> forward */
    {
        *rpm =  pitch_axis_screw_target_rotor_speed_revolution_per_minute * pitch_axis_screw_stick_direction_sign;
    }
    else if (pitch_channel < 0)     /* stick down -> reverse */
    {
        *rpm = -pitch_axis_screw_target_rotor_speed_revolution_per_minute * pitch_axis_screw_stick_direction_sign;
    }
    else
    {
        *rpm = 0.0f;
    }
}

/**
 * @brief 自动控制函数
 * @param yaw YAW轴角度增量输出
 * @param pitch PITCH轴角度增量输出
 * @param gimbal_control_state 云台控制结构体指针
 * @note 从PC接收的目标信息中获取角度设定值
 */
static void Gimbal_CalculateAutomaticAimSetpoint(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_state)
{
    *yaw = gimbal_control_state->hero_pc->pc_yaw;
    *pitch = gimbal_control_state->hero_pc->pc_pitch;
}



