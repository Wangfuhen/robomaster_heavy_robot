#include "chassis_task.h"
#include "main.h"
#include "can_user.h"
#include "cmsis_os.h"
#include "pid.h"
#include "arm_math.h"
#include "ins_task.h"
#include "math.h"
#include <stdlib.h>
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

static void Chassis_Init(chassis_control_t *mini_chassis_init);                                               //底盘初始化
static void Chassis_Set_Mode(chassis_control_t *mini_chassis_set_mode);                                       //底盘模式设置
static void Chassis_Mode_Change_Feedback_Change(chassis_control_t *mini_chassis_mode_change_feedback_change); //底盘模式切换反馈
static void Chassis_Feedback_Updata(chassis_control_t *mini_chassis_feedback_updata);                         //底盘反馈更新
static void Chassis_Mode_Set_Control(chassis_control_t *mini_chassis_mode_set_control);                       //底盘模式切换控制
static void Chassis_Control_Cacl(chassis_control_t *mini_chassis_control_cacl);                               //底盘模式控制计算
/*子函数*/
static void Chassis_Action_To_Control_Cacl(chassis_control_t *mini_chassis_action_to_control_cacl);           //行为控制量计算
static void Chassis_Rc_To_Control_Xy(chassis_control_t *mini_chassis_rc_to_control_cacl);                     //遥控器控制XY计算
static void Chassis_Keyboard_To_Control_Xy(chassis_control_t *mini_chassis_keyboard_to_control_cacl);         //键盘控制XY计算
static void Chassis_Power_Speed_Control(chassis_control_t *hero_chassis_power_speed_control);                 //功率速度限制控制
static void Chassis_Power_Feeback_Updata(chassis_control_t *chassis_power_feeback_updata);                    //功率反馈更新
static void Chassis_Vector_To_Mecaum_Speed(chassis_control_t *chassis_vector_to_mecaum_speed);                //底盘向量转麦轮各电机速度
static fp32 Motor_Ecd_To_Angle(uint16_t ecd,uint16_t ecd_zero_angle_set);                                     //电机角度转换
fp32 sin_yaw =0.0f;
fp32 cos_yaw =0.0f; 
chassis_control_t hero_chassis;
void Chassis_Task(void const * argument)
{   vTaskDelay(MINI_CHASSIS_INIT_TIME);
    portTickType currentTime;
    Chassis_Init(&hero_chassis);
	vTaskDelay(MINI_CHASSIS_INIT_TIME);
    while (1) 
    {   
       currentTime = xTaskGetTickCount();
        Chassis_Set_Mode(&hero_chassis);
       Chassis_Mode_Change_Feedback_Change(&hero_chassis);
       Chassis_Feedback_Updata(&hero_chassis);
        Chassis_Mode_Set_Control(&hero_chassis);
       Chassis_Control_Cacl(&hero_chassis);
    	CAN_cmd_chassis(hero_chassis.chassis_motor[0].give_current,hero_chassis.chassis_motor[1].give_current,hero_chassis.chassis_motor[2].give_current,hero_chassis.chassis_motor[3].give_current);
		
//    #if INCLUDE_uxTaskGetStackHighWaterMark
//        chassis_high_water = uxTaskGetStackHighWaterMark(NULL); //获取任务栈使用情况
//    #endif
    vTaskDelay(1);
	}
}

static void Chassis_Init(chassis_control_t *mini_chassis_init)
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
    mini_chassis_init->vx_nomal_speed=0;
    mini_chassis_init->vy_nomal_speed=0;
    mini_chassis_init->wz_nomal_speed=0;
    mini_chassis_init->vx_add_speed=0;
    mini_chassis_init->vy_add_speed=0;
    mini_chassis_init->wz_add_speed=0;
    mini_chassis_init->chassis_power = 0;
    mini_chassis_init->chassis_power_user_can = 0;
    mini_chassis_init->chassis_power_max = 0;
    mini_chassis_init->chassis_power = 0;
	mini_chassis_init->chassis_follow.gimbal_change = CHASSIS_FOLLOW_GIMABL_NOMAL;
	mini_chassis_init->chassis_follow.gimbal_change_chassis_time=0;
	mini_chassis_init->chassis_follow.gimbal_change_chassis_flag=0;
	mini_chassis_init->chassis_x_slope_accl=SLOPE_CHASSIS_X_ACCL;
	mini_chassis_init->chassis_x_slope_rate=SLOPE_CHASSIS_X_RETA;
	mini_chassis_init->chassis_y_slope_accl=SLOPE_CHASSIS_Y_ACCL;
	mini_chassis_init->chassis_y_slope_rate=SLOPE_CHASSIS_Y_RETA;
	mini_chassis_init->chassis_w_slope_accl=SLOPE_CHASSIS_W_ACCL;
	mini_chassis_init->chassis_w_slope_rate=SLOPE_CHASSIS_W_RETA;
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
    Chassis_Feedback_Updata(mini_chassis_init);
}

static void Chassis_Set_Mode(chassis_control_t *mini_chassis_set_mode)
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
			mini_chassis_set_mode->chassis_follow.gimbal_change=CHASSIS_FOLLOW_GIMABL_NOMAL;
		}
		else if(mini_chassis_set_mode->chassis_gimbal_can_data->follow_mode==1)
		{
			mini_chassis_set_mode->chassis_follow.gimbal_change=CHASSIS_FOLLOW_GIMABL_CHANGE;
		}
}

static void Chassis_Mode_Change_Feedback_Change(chassis_control_t *mini_chassis_mode_change_feedback_change)
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

static void Chassis_Feedback_Updata(chassis_control_t *mini_chassis_feedback_updata)
{
    if(mini_chassis_feedback_updata == NULL)
    {
        return;
    }
	if(mini_chassis_feedback_updata->chassis_follow.gimbal_change==CHASSIS_FOLLOW_GIMABL_NOMAL)
	{   
		if(mini_chassis_feedback_updata->gimbal_yaw_motor->ecd==0)
		{
			
			mini_chassis_feedback_updata->relative_angle=Motor_Ecd_To_Angle(mini_chassis_feedback_updata->gimbal_yaw_motor->last_ecd,YAW_ZERO_ANGLE_SET);
		}
		mini_chassis_feedback_updata->relative_angle=Motor_Ecd_To_Angle(mini_chassis_feedback_updata->gimbal_yaw_motor->ecd,YAW_ZERO_ANGLE_SET);
	}
	else if(mini_chassis_feedback_updata->chassis_follow.gimbal_change==CHASSIS_FOLLOW_GIMABL_CHANGE)
	{
	    mini_chassis_feedback_updata->relative_angle=Motor_Ecd_To_Angle(mini_chassis_feedback_updata->gimbal_yaw_motor->ecd,YAW_CHANGE_ANGLE_SET);
	}
    for(int i=0;i<4;i++)
    {	
        mini_chassis_feedback_updata->chassis_motor[i].speed_rpm = mini_chassis_feedback_updata->chassis_motor[i].chassis_motor_feedback->speed_rpm;
        mini_chassis_feedback_updata->chassis_motor[i].give_current = mini_chassis_feedback_updata->chassis_motor[i].chassis_motor_feedback->given_current;
	    mini_chassis_feedback_updata->chassis_motor[i].motor_i=mini_chassis_feedback_updata->chassis_motor[i].chassis_motor_feedback->given_current*MA_A;
		mini_chassis_feedback_updata->chassis_motor[i].motor_w = mini_chassis_feedback_updata->chassis_motor[i].chassis_motor_feedback->speed_rpm*RPM_RADS;
        mini_chassis_feedback_updata->chassis_motor[i].motor_power = (K0+(K1*mini_chassis_feedback_updata->chassis_motor[i].motor_i)+(K2*mini_chassis_feedback_updata->chassis_motor[i].motor_w)+(K3*mini_chassis_feedback_updata->chassis_motor[i].motor_w*mini_chassis_feedback_updata->chassis_motor[i].motor_i)+(K4*mini_chassis_feedback_updata->chassis_motor[i].motor_i*mini_chassis_feedback_updata->chassis_motor[i].motor_i)+(K5*mini_chassis_feedback_updata->chassis_motor[i].motor_w*mini_chassis_feedback_updata->chassis_motor[i].motor_w)); 
        mini_chassis_feedback_updata->vx_now = (-mini_chassis_feedback_updata->chassis_motor[0].speed_rpm + mini_chassis_feedback_updata->chassis_motor[1].speed_rpm + mini_chassis_feedback_updata->chassis_motor[2].speed_rpm - mini_chassis_feedback_updata->chassis_motor[3].speed_rpm)*MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
        mini_chassis_feedback_updata->vy_now = (-mini_chassis_feedback_updata->chassis_motor[0].speed_rpm - mini_chassis_feedback_updata->chassis_motor[1].speed_rpm + mini_chassis_feedback_updata->chassis_motor[2].speed_rpm + mini_chassis_feedback_updata->chassis_motor[3].speed_rpm)*MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
        mini_chassis_feedback_updata->wz_now = (-mini_chassis_feedback_updata->chassis_motor[0].speed_rpm - mini_chassis_feedback_updata->chassis_motor[1].speed_rpm - mini_chassis_feedback_updata->chassis_motor[2].speed_rpm - mini_chassis_feedback_updata->chassis_motor[3].speed_rpm)*MOTOR_SPEED_TO_CHASSIS_SPEED_WZ/MOTOR_DISTANCE_TO_CENTER;
    }
    mini_chassis_feedback_updata->chassis_power=mini_chassis_feedback_updata->chassis_motor[0].motor_power+mini_chassis_feedback_updata->chassis_motor[1].motor_power+mini_chassis_feedback_updata->chassis_motor[2].motor_power+mini_chassis_feedback_updata->chassis_motor[3].motor_power;
    mini_chassis_feedback_updata->chassis_gyro=*(mini_chassis_feedback_updata->chassis_ins_point+INS_GYRO_Z_ADDRESS_OFFSET);
    Chassis_Power_Feeback_Updata(mini_chassis_feedback_updata);
}

static void Chassis_Mode_Set_Control(chassis_control_t *mini_chassis_mode_set_control)
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
        Chassis_Action_To_Control_Cacl(mini_chassis_mode_set_control);
    }

}

static void Chassis_Control_Cacl(chassis_control_t *mini_chassis_control_cacl)
{
    if(mini_chassis_control_cacl == NULL)
    {
        return;
    }
    fp32 speed_ave=fabs(mini_chassis_control_cacl->chassis_motor[0].speed_rpm+mini_chassis_control_cacl->chassis_motor[1].speed_rpm+mini_chassis_control_cacl->chassis_motor[2].speed_rpm+mini_chassis_control_cacl->chassis_motor[3].speed_rpm)/4;
    for (uint8_t i = 0; i < 4; i++)
    {
        mini_chassis_control_cacl->chassis_speed_pid[i].ki=0.20+1.0*exp(-0.001*fabs(speed_ave)-1.8);
        mini_chassis_control_cacl->chassis_speed_pid[i].max_iout=fmin((440 + 6 * fabs(speed_ave / 100)),1000);
        mini_chassis_control_cacl->chassis_motor[i].give_current=pid_calc(&mini_chassis_control_cacl->chassis_speed_pid[i],mini_chassis_control_cacl->chassis_motor[i].speed_rpm,mini_chassis_control_cacl->chassis_motor[i].speed_set+mini_chassis_control_cacl->chassis_gyro*CHASSIS_SOLLOW_GIMBAL_FORWARD_KP);
    }
    for(uint8_t i=0;i<4;i++)
    {
		mini_chassis_control_cacl->chassis_motor[i].motor_use_i=mini_chassis_control_cacl->chassis_motor[i].give_current*MA_A;
		mini_chassis_control_cacl->chassis_motor[i].motor_w = mini_chassis_control_cacl->chassis_motor[i].chassis_motor_feedback->speed_rpm*RPM_RADS;
		mini_chassis_control_cacl->chassis_motor[i].motor_use_power=(K0+(K1*mini_chassis_control_cacl->chassis_motor[i].motor_use_i)+(K2*mini_chassis_control_cacl->chassis_motor[i].motor_w)+(K3*mini_chassis_control_cacl->chassis_motor[i].motor_w*mini_chassis_control_cacl->chassis_motor[i].motor_use_i)+(K4*mini_chassis_control_cacl->chassis_motor[i].motor_use_i*mini_chassis_control_cacl->chassis_motor[i].motor_use_i)+(K5*mini_chassis_control_cacl->chassis_motor[i].motor_w*mini_chassis_control_cacl->chassis_motor[i].motor_w));
	}
	mini_chassis_control_cacl->chassis_power_use=mini_chassis_control_cacl->chassis_motor[0].motor_use_power+mini_chassis_control_cacl->chassis_motor[1].motor_use_power+mini_chassis_control_cacl->chassis_motor[2].motor_use_power+mini_chassis_control_cacl->chassis_motor[3].motor_use_power;
	Chassis_Power_Speed_Control(mini_chassis_control_cacl);
	if(mini_chassis_control_cacl->chassis_mode == CHASSIS_ZERO_FORCE)
    {
       for(uint8_t i=0;i<4;i++)
       {
           mini_chassis_control_cacl->chassis_motor[i].give_current = 0;
		   
       }	   
    }
}

static void Chassis_Action_To_Control_Cacl(chassis_control_t *mini_chassis_action_to_control_cacl)
{
    fp32 vx=0;
	fp32 vy=0;
    if(mini_chassis_action_to_control_cacl == NULL)
    {
        return;
    }
	 if(mini_chassis_action_to_control_cacl->chassis_gimbal_can_data->key_control==1)//进入比赛模式
     {
	      Chassis_Keyboard_To_Control_Xy(mini_chassis_action_to_control_cacl);  //键盘控制
	 }
	 else
	 {
	 	  Chassis_Rc_To_Control_Xy(mini_chassis_action_to_control_cacl);  //遥控器通道计算
	 }
   
    if(mini_chassis_action_to_control_cacl->chassis_top_mode == TOP_ON)
    {
        if(mini_chassis_action_to_control_cacl->chassis_cap_mode == CAP_ON)
        {
            mini_chassis_action_to_control_cacl->wz_nomal_speed=CHASSIS_SPEED_TOP_CAP_ON;
        }
        else if(mini_chassis_action_to_control_cacl->chassis_cap_mode == CAP_OFF)
        {
            mini_chassis_action_to_control_cacl->wz_nomal_speed=CHASSIS_SPEED_TOP_CAP_OFF;
        }
        if(mini_chassis_action_to_control_cacl->chassis_top_direction == CHASSIS_TOP_INVE)
        {
            mini_chassis_action_to_control_cacl->wz_nomal_speed = mini_chassis_action_to_control_cacl->wz_nomal_speed;
        }
        else if(mini_chassis_action_to_control_cacl->chassis_top_direction == CHASSIS_TOP_FWAD)
        {
            mini_chassis_action_to_control_cacl->wz_nomal_speed = -mini_chassis_action_to_control_cacl->wz_nomal_speed;
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
			{   mini_chassis_action_to_control_cacl->wz_nomal_speed = 0;
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
				mini_chassis_action_to_control_cacl->wz_nomal_speed=pid_calc(&mini_chassis_action_to_control_cacl->chassis_follow_gimbal_pid,mini_chassis_action_to_control_cacl->relative_angle,mini_chassis_action_to_control_cacl->set_relative_angle);
			}	
			
        }
        else if(CHASSIS_NO_FOLLOW_GIMBAL)
        {
            mini_chassis_action_to_control_cacl->wz_nomal_speed = 0;
        } 
    }
    mini_chassis_action_to_control_cacl->vx_set=mini_chassis_action_to_control_cacl->vx_nomal_speed+mini_chassis_action_to_control_cacl->vx_add_speed;
    mini_chassis_action_to_control_cacl->vy_set=mini_chassis_action_to_control_cacl->vy_nomal_speed+mini_chassis_action_to_control_cacl->vy_add_speed;
    mini_chassis_action_to_control_cacl->wz_set=mini_chassis_action_to_control_cacl->wz_nomal_speed+mini_chassis_action_to_control_cacl->wz_add_speed;
	if(mini_chassis_action_to_control_cacl->chassis_follow.gimbal_change==CHASSIS_FOLLOW_GIMABL_CHANGE)
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
    Chassis_Vector_To_Mecaum_Speed(mini_chassis_action_to_control_cacl);
}

static void Chassis_Rc_To_Control_Xy(chassis_control_t *mini_chassis_rc_to_control_cacl)
{
	  
		mini_chassis_rc_to_control_cacl->vx_nomal_speed=mini_chassis_rc_to_control_cacl->chassis_gimbal_can_data->x_rc_data*RC_X_SENSE;
		mini_chassis_rc_to_control_cacl->vy_nomal_speed=mini_chassis_rc_to_control_cacl->chassis_gimbal_can_data->y_rc_data*RC_Y_SENSE;
}

static void Chassis_Keyboard_To_Control_Xy(chassis_control_t *mini_chassis_keyboard_to_control_cacl)
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
        mini_chassis_keyboard_to_control_cacl->vx_nomal_speed=-mini_chassis_keyboard_to_control_cacl->vx_max_speed;
    }
    else if(mini_chassis_keyboard_to_control_cacl->chassis_gimbal_can_data->x_key_data&BACK)
    {
        mini_chassis_keyboard_to_control_cacl->vx_nomal_speed=mini_chassis_keyboard_to_control_cacl->vx_max_speed;
    }
    else
    {
        mini_chassis_keyboard_to_control_cacl->vx_nomal_speed=0;  
    }
    if(mini_chassis_keyboard_to_control_cacl->chassis_gimbal_can_data->y_key_data&RGHTWARD)
    {
        mini_chassis_keyboard_to_control_cacl->vy_nomal_speed=CHASSIS_SPEED_MAX_VY_KEYBOARD;
    }
    else if(mini_chassis_keyboard_to_control_cacl->chassis_gimbal_can_data->y_key_data&LEFTWARD)
    {
        mini_chassis_keyboard_to_control_cacl->vy_nomal_speed=-CHASSIS_SPEED_MAX_VY_KEYBOARD;
    }
    else
    {
        mini_chassis_keyboard_to_control_cacl->vy_nomal_speed=0;
    }
}
static void Chassis_Power_Speed_Control(chassis_control_t *hero_chassis_power_speed_control)
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
	chassis_buffer_rm_max=hero_chassis_power_speed_control->chassis_buffer_rm;
    user_chassis_power=hero_chassis_power_speed_control->chassis_power_use;
	chassos_buffer_rm_use=rm_chassis_power_buffer-hero_chassis_power_speed_control->chassis_buffer_user;
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
			hero_chassis_power_speed_control->chassis_motor[i].motor_p=hero_chassis_power_speed_control->chassis_motor[i].motor_use_power*chassis_power_max/user_chassis_power;//进行功率动态分配以及衰减
		}
	}
	for(uint8_t i=0;i<4;i++)
	{
		a=K4;
		b=K1+K3*hero_chassis_power_speed_control->chassis_motor[i].motor_w;
		c=K0+K2*hero_chassis_power_speed_control->chassis_motor[i].motor_w+K5*hero_chassis_power_speed_control->chassis_motor[i].motor_w*hero_chassis_power_speed_control->chassis_motor[i].motor_w-hero_chassis_power_speed_control->chassis_motor[i].motor_p;
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
				if(Abs(chassis_motor_i1-hero_chassis_power_speed_control->chassis_motor[i].motor_use_i)>Abs(chassis_motor_i2-hero_chassis_power_speed_control->chassis_motor[i].motor_use_i))
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

static void Chassis_Power_Feeback_Updata(chassis_control_t *chassis_power_feeback_updata)
{
    if(chassis_power_feeback_updata == NULL)
    {
        return;
    }
    if(chassis_power_feeback_updata->chassis_rm_data->power_heat_data_t.buffer_energy>70)
    {
        chassis_power_feeback_updata->chassis_buffer_rm=CHASSIS_POWER_BUFFER_ALL_GAIN;
        chassis_power_feeback_updata->chassis_buffer_user=CHASSIS_POWER_BUFFER_USE_MAX_GAIN;
    }
    else if(chassis_power_feeback_updata->chassis_rm_data->power_heat_data_t.buffer_energy<=60)
    {
        chassis_power_feeback_updata->chassis_buffer_rm=CHASSIS_POWER_BUFFER_ALl_NO_GAIN;
        chassis_power_feeback_updata->chassis_buffer_user=CHASSIS_POWER_BUFFER_USE_MAX_NO_GAIN;
    }

}

static void Chassis_Vector_To_Mecaum_Speed(chassis_control_t *chassis_vector_to_mecaum_speed)
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

static fp32 Motor_Ecd_To_Angle(uint16_t ecd,uint16_t ecd_zero_angle_set)
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
