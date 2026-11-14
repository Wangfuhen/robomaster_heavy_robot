#include "chassis_data_task.h"
#include "../can/can_user.h"

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

static void mini_chassis_mode_set(gimbal_c_send_data_t *gimbal_send_data,gimbal_control_t *mini_gimbal);
static void send_remote_to_chassis(gimbal_control_t *mini_gimbal);
extern gimbal_control_t mini_gimbal;
void Chassis_Data_Task(void const * argument)
{
	vTaskDelay(CHASSIS_DATA_DELAY);
	while(1)
	{
		mini_chassis_mode_set(&gimbal_send_chassis_data,&mini_gimbal);
		CAN_gimbal_send_chassis(&gimbal_send_chassis_data);
		   const RC_ctrl_t *rc_data = get_dt7_remote_control_point();
	
		    if(rc_data != NULL)
        {
        	send_remote_to_chassis(&mini_gimbal);
        }
		vTaskDelay(1);
	}
	
}
static void send_remote_to_chassis(gimbal_control_t *mini_gimbal)
{
    static uint32_t last_send_time = 0;
    uint32_t current_time = HAL_GetTick();
    
   
    if (current_time - last_send_time < 10)
    {
        return;
    }
    
    if (mini_gimbal->gimbal_dt7_remote_control != NULL)
    {
        
        uint8_t left_switch = mini_gimbal->gimbal_dt7_remote_control->rc.s[1]; 
 
        
       
        CAN_send_remote_to_chassis(left_switch); 
    }
    
    last_send_time = current_time;
}

static void mini_chassis_mode_set(gimbal_c_send_data_t *gimbal_send_data,gimbal_control_t *mini_gimbal)
{
	int16_t x_rc=0;
	int16_t y_rc=0;
	Remote_Deadband_Limit(mini_gimbal->gimbal_dt7_remote_control->rc.ch[X_rc_channel],x_rc,10);
	Remote_Deadband_Limit(mini_gimbal->gimbal_dt7_remote_control->rc.ch[Y_rc_channel],y_rc,10);
	
	/* 【已停用】一键掉头至正位：机器人加装机械臂后不再支持该模式，
	   云台朝向状态恒为 GIMBAL_HEADING_FORWARD，因此发往底盘的
	   follow_mode / follow_mode_last 固定为 0（正常跟随方向）。
	gimbal_send_data->follow_mode=mini_gimbal->heading_transition.heading_state;
	gimbal_send_data->follow_mode_last=mini_gimbal->heading_transition.previous_heading_state;
	*/
	gimbal_send_data->follow_mode=0;
	gimbal_send_data->follow_mode_last=0;
	
	if(switch_is_up(mini_gimbal->gimbal_dt7_remote_control->rc.s[remote_control_right_switch])) //进入比赛模式
	{
		gimbal_send_data->key_control=3;
		
		if(mini_gimbal->gimbal_dt7_remote_control->key.v&FORWARED)    //前
		{
			gimbal_send_data->x_key_data=1;
		}
		else if(mini_gimbal->gimbal_dt7_remote_control->key.v&BACK)  //后
		{
			gimbal_send_data->x_key_data=2;
		}
		else
		{
			gimbal_send_data->x_key_data=0;
		}
		if(mini_gimbal->gimbal_dt7_remote_control->key.v&RGHTWARD)     //右
		{
			gimbal_send_data->y_key_data=1;
		}
		else if(mini_gimbal->gimbal_dt7_remote_control->key.v&LEFTWARD)//左
		{
			gimbal_send_data->y_key_data=2;
		}
		else
		{
			gimbal_send_data->y_key_data=0;
		}

		if(mini_gimbal->gimbal_dt7_remote_control->key.v&CAP_ON_KEY)
		{
			gimbal_send_data->cap_mode=1;
		}
		else
		{
			gimbal_send_data->cap_mode=0;
		}

		/* 【已停用】小陀螺：机器人加装机械臂后无法再支持小陀螺模式，
		   这里把遥控器 C 键（TOP_ON_KEY）的解析整段注释，top_mode 恒为 0。
		if(mini_gimbal->gimbal_dt7_remote_control->key.v&TOP_ON_KEY)
		{
			gimbal_send_data->top_mode=1;
		}
		else
		{
			gimbal_send_data->top_mode=0;
		}
		*/
		gimbal_send_data->top_mode=0;
	}
	else //调试
	{
		gimbal_send_data->key_control=0;
		gimbal_send_data->x_rc_data=x_rc;
	    gimbal_send_data->y_rc_data=y_rc;
		if(switch_is_mid(mini_gimbal->gimbal_dt7_remote_control->rc.s[remote_control_right_switch]))
		{
			gimbal_send_data->chassis_mode=3;
			if(mini_gimbal->gimbal_dt7_remote_control->rc.ch[4]>300)
			{
				gimbal_send_data->cap_mode=1;
			}
			else
			{
				gimbal_send_data->cap_mode=0;
			}
			
			/* 【已停用】小陀螺：机器人加装机械臂后无法再支持小陀螺模式，
			   这里把遥控器通道4下拨进入小陀螺的解析整段注释，top_mode 恒为 0。
			if(mini_gimbal->gimbal_dt7_remote_control->rc.ch[4]<-300)
			{
				gimbal_send_data->top_mode=1;
			}
			else
			{
				gimbal_send_data->top_mode=0;
			}
			*/
			gimbal_send_data->top_mode=0;
		}
		else if(switch_is_down(mini_gimbal->gimbal_dt7_remote_control->rc.s[remote_control_right_switch]))
		{
			gimbal_send_data->chassis_mode=2;
		}
	}
	
}
