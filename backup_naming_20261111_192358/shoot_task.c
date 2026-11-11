#include "shoot_task.h"
#include "stdlib.h"
#include "user_lib.h"


// 射击系统LQR控制器参数
fp32 shoot_lqr[5]= {0.0f,1500.0f,0.0f,1000.0f,0.0f};

// 静态函数声明
static void Shoot_Init(hero_shoot_t *shoot_init);                     // 射击系统初始化
static void Shoot_Cacl_Loop(hero_shoot_t *shoot_cacl);                // 射击计算循环
static void Shoot_Mode_Set(hero_shoot_t *shoot_mode);                 // 射击模式设置
static void Shoot_Feedback_Updata(hero_shoot_t *shoot_feedback_updata); // 反馈数据更新
static void Shoot_Control_Set(hero_shoot_t *shoot_control_set);       // 控制量设置

static fp32 Motor_Ecd_To_Angle(uint16_t ecd,uint16_t ecd_zero_angle_set); // 电机编码值转角度
static void shoot_speed_set(hero_shoot_t *shoot_speed_set);            // 射击速度控制
static void shoot_power_set(hero_shoot_t *shoot_speed_set);            // 枪口热量控制
static void shoot_bullet_control(hero_shoot_t *shooot_bullet_control); // 射击频率控制
static void trigger_motor_turn_back(hero_shoot_t *shoot_block);        // 射击堵转控制

// 全局射击系统结构体
hero_shoot_t hero_shoot;
gimbal_control_t mini_gimbal;

int cnt_shoot = 0;
float tem_shoot_current=0.0f;

/**
  * @brief 射击系统任务
  * @param argument: 任务参数
  */
void Shoot_Task(void const * argument)
{
    // 初始化射击系统
    Shoot_Init(&hero_shoot);
    vTaskDelay(SHOOT_INIT_TIM);
      Shoot_Feedback_Updata(&hero_shoot);
    while(1)
    {
        // 设置射击模式
        Shoot_Mode_Set(&hero_shoot);
        // 更新反馈数据
        Shoot_Feedback_Updata(&hero_shoot);
        // 设置控制量
        Shoot_Control_Set(&hero_shoot);
        // 计算循环
        Shoot_Cacl_Loop(&hero_shoot);
         
        // 注释掉的实际CAN通信代码
       CAN_cmd_shoot(hero_shoot.shoot_motor_1.give_current,hero_shoot.shoot_motor_2.give_current,hero_shoot.shoot_motor_3.give_current);
       //tem_shoot_current = hero_shoot.trigger_motor.give_current=1000.0f;
       abs_limit(&hero_shoot.trigger_motor.give_current, 10000.0f);
		CAN_cmd_trigger(hero_shoot.trigger_motor.give_current, mini_gimbal.yaw_gimbal_motor.gimbal_motor_measure->ecd);

        
       cnt_shoot++;
        vTaskDelay(SHOOT_LOOP_TIME);
    }
}

/**
  * @brief 射击系统初始化
  * @param shoot_init: 射击系统结构体指针
  */
static void Shoot_Init(hero_shoot_t *shoot_init)
{
    // 获取遥控器数据指针
    shoot_init->shoot_dt7_remote_control = get_dt7_remote_control_point();
    // 获取射击电机测量数据指针
    shoot_init->shoot_motor_1.shoot_motor_measure = get_motor_gimbal_fire_right_measure_point();
    shoot_init->shoot_motor_2.shoot_motor_measure = get_motor_gimbal_fire_left_measure_point();
    shoot_init->shoot_motor_3.shoot_motor_measure = get_motor_gimbal_fire_down_measure_point();
    // 获取拨弹电机测量数据指针
    shoot_init->trigger_motor.trigger_motor_measure = get_motor_gimbal_trigger_measure_point();
    // 获取云台控制指针
    shoot_init->gimbal = get_gimbal_control_point();
    
    // 初始化状态
    shoot_init->shoot_mode = SHOOT_STOP;
    shoot_init->trigger_motor.trigger_mode = TRIGGER_STOP;
    shoot_init->fire_flag_change = 0;
    shoot_init->fire_flag_time = 0;
    
     // 拨弹盘速度PID初始化
    static const fp32 trigger_speed_pid[3] = {TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD};
    pid_init(&shoot_init->trigger_motor.speed_pid, PID_POSITION, trigger_speed_pid, 
             TRIGGER_SPEED_PID_MAX_OUT, TRIGGER_SPEED_PID_MAX_IOUT);
    
    // 拨弹盘LQR控制器初始化
    static fp32 triggle_speed_factor[4] = {TRIGGLE_SPEED_KP, TRIGGLE_SPEED_KI, TRIGGLE_SPEED_I_MAX, TRIGGLE_SPEED_OUT_MAX};
    static fp32 triggle_angle_factor[4] = {TRIGGLE_ANGLE_KP, TRIGGLE_ANGLE_KI, TRIGGLE_ANGLE_I_MAX, TRIGGLE_ANGLE_OUT_MAX};
    static fp32 triggle_current_factor[4] = {TRIGGLE_CURRENT_KP, TRIGGLE_CURRENT_KI, TRIGGLE_CURRENT_I_MAX, TRIGGLE_CURRENT_OUT_MAX};
    lqr_init(&shoot_init->trigger_motor.lqr, triggle_speed_factor, triggle_angle_factor, 
             triggle_current_factor, TRIGGLE_CURRENT_OUT_MAX);
    
    // 摩擦轮电机PID初始化
    static const fp32 fire1_speed_pid[3] = {FIRE1_SPEED_PID_KP, FIRE1_SPEED_PID_KI, FIRE1_SPEED_PID_KD};
    static const fp32 fire2_speed_pid[3] = {FIRE2_SPEED_PID_KP, FIRE2_SPEED_PID_KI, FIRE2_SPEED_PID_KD};
    static const fp32 fire3_speed_pid[3] = {FIRE3_SPEED_PID_KP, FIRE3_SPEED_PID_KI, FIRE3_SPEED_PID_KD};
    
    pid_init(&shoot_init->shoot_motor_1.pid_shoot_speed, PID_DELTA, fire1_speed_pid, 
             FIRE_FIRE_PID_MAX_OUT, FIRE_FIRE_PID_MAX_IOUT);
    pid_init(&shoot_init->shoot_motor_2.pid_shoot_speed, PID_DELTA, fire2_speed_pid, 
             FIRE_FIRE_PID_MAX_OUT, FIRE_FIRE_PID_MAX_IOUT);
    pid_init(&shoot_init->shoot_motor_3.pid_shoot_speed, PID_DELTA, fire3_speed_pid, 
             FIRE_FIRE_PID_MAX_OUT, FIRE_FIRE_PID_MAX_IOUT);
		
}

/**
  * @brief 射击模式设置
  * @param shoot_mode: 射击系统结构体指针
  */
static void Shoot_Mode_Set(hero_shoot_t *shoot_mode)
{
    // 比赛模式（右拨杆上）
    if(switch_is_up(shoot_mode->shoot_dt7_remote_control->rc.s[RIGHT]))
    {
        // 按键触发射击模式切换（有防抖延时）
        if(shoot_mode->shoot_dt7_remote_control->key.v & FIRE_ON && 
           shoot_mode->fire_flag_change == 0 && 
           shoot_mode->fire_flag_time >= 500)
        {
            shoot_mode->fire_flag_change = 1;
            shoot_mode->shoot_mode = SHOOT_START;  // 启动射击
            shoot_mode->fire_flag_time = 0;
        }
        else if(shoot_mode->shoot_dt7_remote_control->key.v & FIRE_ON && 
                shoot_mode->fire_flag_change == 1 && 
                shoot_mode->fire_flag_time >= 500)
        {
            shoot_mode->fire_flag_change = 0;
            shoot_mode->fire_flag_time = 0;
            shoot_mode->shoot_mode = SHOOT_STOP;  // 停止射击
        }
        
        // 防抖计时器更新（防止溢出）
        if(shoot_mode->fire_flag_time < 1000)
        {
            shoot_mode->fire_flag_time++;
        }
        else
        {
            shoot_mode->fire_flag_time = 1000;
        }
    }
    else  // 调试模式
    {
        // 左拨杆上为启动射击
        if(switch_is_up(shoot_mode->shoot_dt7_remote_control->rc.s[LEFT])  ||  (switch_is_down(shoot_mode->shoot_dt7_remote_control->rc.s[LEFT])&&(switch_is_down(shoot_mode->shoot_dt7_remote_control->rc.s[RIGHT]))))
        {  
            shoot_mode->shoot_mode = SHOOT_START;
        }
				
        else
        {
            shoot_mode->shoot_mode = SHOOT_STOP;
        }
    }
}

/**
  * @brief 反馈数据更新
  * @param shoot_feedback_updata: 射击系统结构体指针
  */
static void Shoot_Feedback_Updata(hero_shoot_t *shoot_feedback_updata)
{
    // 拨弹电机反馈数据
    shoot_feedback_updata->trigger_motor.feedback_current = 
        shoot_feedback_updata->trigger_motor.trigger_motor_measure->given_current * MOTOR_A;
    shoot_feedback_updata->trigger_motor.feedback_speed = 
        shoot_feedback_updata->trigger_motor.trigger_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED;
    
    // 摩擦轮电机速度反馈
    shoot_feedback_updata->shoot_motor_1.feedback_speed = 
        shoot_feedback_updata->shoot_motor_1.shoot_motor_measure->speed_rpm;
    shoot_feedback_updata->shoot_motor_2.feedback_speed = 
        shoot_feedback_updata->shoot_motor_2.shoot_motor_measure->speed_rpm;
    shoot_feedback_updata->shoot_motor_3.feedback_speed = 
        shoot_feedback_updata->shoot_motor_3.shoot_motor_measure->speed_rpm;
    
    // 鼠标按键状态
    shoot_feedback_updata->last_press_r = shoot_feedback_updata->last_press_l;
    shoot_feedback_updata->press_l = shoot_feedback_updata->shoot_dt7_remote_control->mouse.press_l;
    
    // 拨弹盘编码器数据处理
    float fix_dials_ecd = (float)shoot_feedback_updata->trigger_motor.trigger_motor_measure->ecd;
    float fix_last_dials_ecd = (float)shoot_feedback_updata->trigger_motor.trigger_motor_measure->last_ecd;
    shoot_feedback_updata->trigger_motor.dials_ecd_count = fix_dials_ecd - fix_last_dials_ecd;
    
    // 编码器溢出处理
    if(shoot_feedback_updata->trigger_motor.dials_ecd_count > (0.5 * 8192))
    {
        shoot_feedback_updata->trigger_motor.dials_ecd_count -= 8192;
    }
    else if(shoot_feedback_updata->trigger_motor.dials_ecd_count < (-0.5 * 8192))
    {
        shoot_feedback_updata->trigger_motor.dials_ecd_count += 8192;
    }
    
    // 累加角度计算（编码器值转换为角度）
    shoot_feedback_updata->trigger_motor.add_angle += (shoot_feedback_updata->trigger_motor.dials_ecd_count / 8192) * 18.947368 / 3.16;
}

/**
  * @brief 控制量设置
  * @param shoot_control_set: 射击系统结构体指针
  */
static void Shoot_Control_Set(hero_shoot_t *shoot_control_set)
{
    // 射击停止模式
    if(shoot_control_set->shoot_mode == SHOOT_STOP)
    {
        // 摩擦轮速度设为0
        shoot_control_set->shoot_motor_1.speed_set = 0;
        shoot_control_set->shoot_motor_2.speed_set = 0;
        shoot_control_set->shoot_motor_3.speed_set = 0;
        
        // 拨弹电机速度控制
        shoot_control_set->trigger_motor.give_current = 
            pid_calc(&shoot_control_set->trigger_motor.speed_pid, 
                    shoot_control_set->trigger_motor.feedback_speed,
                    shoot_control_set->trigger_motor.trigger_motor_speed_set);
    }
    else  // 射击启动模式
    {
        // 执行射击控制逻辑
        shoot_power_set(shoot_control_set);         // 热量控制
        shoot_speed_set(shoot_control_set);         // 速度控制
        shoot_bullet_control(shoot_control_set);    // 射击频率控制
        trigger_motor_turn_back(shoot_control_set); // 堵转检测控制  临时关闭，调试
        
        // LQR控制器计算拨弹电机电流
        fp32 feed_back_triiger[3] = {
            shoot_control_set->trigger_motor.add_angle,
            shoot_control_set->trigger_motor.feedback_speed,
            shoot_control_set->trigger_motor.feedback_current
        };
        fp32 set_trigger[3] = {
            shoot_control_set->trigger_motor.trigger_motor_angle_set,
            0,
            0
        };
        shoot_control_set->trigger_motor.give_current = 
            lqr_cal(&shoot_control_set->trigger_motor.lqr, feed_back_triiger, set_trigger);
    }
    
    // 摩擦轮电机PID控制
    shoot_control_set->shoot_motor_1.give_current = 
        pid_calc(&shoot_control_set->shoot_motor_1.pid_shoot_speed,
                shoot_control_set->shoot_motor_1.feedback_speed,
                shoot_control_set->shoot_motor_1.speed_set);
    shoot_control_set->shoot_motor_2.give_current = 
        pid_calc(&shoot_control_set->shoot_motor_2.pid_shoot_speed,
                shoot_control_set->shoot_motor_2.feedback_speed,
                shoot_control_set->shoot_motor_2.speed_set);
    shoot_control_set->shoot_motor_3.give_current = 
        pid_calc(&shoot_control_set->shoot_motor_3.pid_shoot_speed,
                shoot_control_set->shoot_motor_3.feedback_speed,
                shoot_control_set->shoot_motor_3.speed_set);
}

/**
  * @brief 射击计算循环（当前为空函数）
  * @param shoot_cacl: 射击系统结构体指针
  */
static void Shoot_Cacl_Loop(hero_shoot_t *shoot_cacl)
{
    // 预留的计算函数，可根据需要添加其他计算逻辑
}

/**
  * @brief 电机编码值转换为角度
  * @param ecd: 当前编码值
  * @param ecd_zero_angle_set: 零点编码值
  * @return 计算得到的角度值
  */
static fp32 Motor_Ecd_To_Angle(uint16_t ecd, uint16_t ecd_zero_angle_set)
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
    
    return relative_ecd * MOTOR_ECD_TO_RAD;
}

/**
  * @brief 射击速度设置
  * @param shoot_speed_set: 射击系统结构体指针
  */
static void shoot_speed_set(hero_shoot_t *shoot_speed_set)
{
  if((switch_is_up(shoot_speed_set->shoot_dt7_remote_control->rc.s[LEFT])))  // 设置摩擦轮电机目标速度
	{shoot_speed_set->shoot_motor_1.speed_set =4200;  // 右摩擦轮
    shoot_speed_set->shoot_motor_2.speed_set = -4200;   // 左摩擦轮
    shoot_speed_set->shoot_motor_3.speed_set = 4100;   // 下摩擦轮
	}
	else
	{
		 shoot_speed_set->shoot_motor_1.speed_set = 0;
       shoot_speed_set->shoot_motor_2.speed_set = 0;
       shoot_speed_set->shoot_motor_3.speed_set = 0;
		
	}
}

/**
  * @brief 射击热量控制
  * @param shoot_speed_set: 射击系统结构体指针
  */
static void shoot_power_set(hero_shoot_t *shoot_speed_set)
{
    // 热量限制标志位（此处暂时设为1，实际应有热量计算逻辑）
    shoot_speed_set->trigger_motor.fire_power_flag = 1;
    
    // 根据按键设置射击频率
    if(switch_is_up(shoot_speed_set->shoot_dt7_remote_control->rc.s[LEFT]))
    {
        shoot_speed_set->trigger_motor.shoot_ban_hz = 1000; // 普通射击模式
    }
    else
    {
            shoot_speed_set->trigger_motor.shoot_ban_hz = 300;
    }
}

  float need_angle = 40.3f;
/**
  * @brief 射击频率控制
  * @param shooot_bullet_control: 射击系统结构体指针
  */
static void shoot_bullet_control(hero_shoot_t *shooot_bullet_control)
{
    // 射击触发条件判断：鼠标左键或遥控器通道4或自动射击模式
    if(((shooot_bullet_control->press_l == 1) || 
        shooot_bullet_control->shoot_dt7_remote_control->rc.ch[4] >= 300 || 
        (shooot_bullet_control->gimbal->hero_pc->pc_fire_mode == 2 && 
         shooot_bullet_control->gimbal->gimbal_mode == GIMBAL_AUTO)) && 
       (shooot_bullet_control->trigger_motor.shoot_ban_time_hz == 0) && 
       shooot_bullet_control->trigger_motor.fire_power_flag == 1)
    {
        if(shooot_bullet_control->trigger_motor.shoot_ban_time_hz == 0)
        {
            // 启动射击计时
            shooot_bullet_control->trigger_motor.shoot_ban_time_hz = 1;
            shooot_bullet_control->trigger_motor.fire_ban_hz = 1;
            //shoot_ban_time_hz：开始记录两次拨弹之间的间隔
            //fire_ban_hz：标记本轮需要更新一次目标角度
        }
    }
    
    // 射击计时器管理
    if(shooot_bullet_control->trigger_motor.shoot_ban_time_hz != 0)
    {
        shooot_bullet_control->trigger_motor.shoot_ban_time_hz++;
        if(shooot_bullet_control->trigger_motor.shoot_ban_time_hz > 
           shooot_bullet_control->trigger_motor.shoot_ban_hz)
        {
            shooot_bullet_control->trigger_motor.shoot_ban_time_hz = 0;
        }
    }
    
    // 设置拨弹电机目标角度
    if(shooot_bullet_control->trigger_motor.fire_ban_hz == 1)
    {
         if(switch_is_up(shooot_bullet_control->shoot_dt7_remote_control->rc.s[LEFT]))
        // 控制拨弹盘旋转一个弹仓位置（40度）
       { 
      
        shooot_bullet_control->trigger_motor.trigger_motor_angle_set -= need_angle;
        // if(__fabs(shooot_bullet_control->trigger_motor.trigger_motor_angle_set - 
        //          shooot_bullet_control->trigger_motor.add_angle) < need_angle)
        //     {
        //         shooot_bullet_control->trigger_motor.trigger_motor_angle_set -= need_angle;
        //     }
        //     else
        //     {
        //         shooot_bullet_control->trigger_motor.trigger_motor_angle_set = 
        //             shooot_bullet_control->trigger_motor.add_angle - need_angle;
        //     }
		}
		else if (switch_is_down(shooot_bullet_control->shoot_dt7_remote_control->rc.s[LEFT]))
		{
					
            shooot_bullet_control->trigger_motor.trigger_motor_angle_set += need_angle;
            // if(__fabs(shooot_bullet_control->trigger_motor.trigger_motor_angle_set - 
            //     shooot_bullet_control->trigger_motor.add_angle) > need_angle)
            // {
            //     shooot_bullet_control->trigger_motor.trigger_motor_angle_set += need_angle;
            // }
            // else
            // {
            //     shooot_bullet_control->trigger_motor.trigger_motor_angle_set = 
            //         shooot_bullet_control->trigger_motor.add_angle + need_angle;
            // }
					
		}
				
				
			
        shooot_bullet_control->trigger_motor.fire_ban_hz = 0;
    }
}

// 调试用计数器
int num = 0;
float block_need_angle=30;

/**
  * @brief 拨弹电机堵转控制
  * @param shoot_block: 射击系统结构体指针
  */
static void trigger_motor_turn_back(hero_shoot_t *shoot_block)
{
    // 未达到堵转时间，正常旋转
    if(shoot_block->trigger_motor.tigger_bolck.block_time < BLOCK_TIME)
    {
        shoot_block->trigger_motor.trigger_motor_angle_set = 
            shoot_block->trigger_motor.trigger_motor_angle_set;
    }
    // 堵转后反转阶段
    else if(shoot_block->trigger_motor.tigger_bolck.reverse_time <= BLOCK_REVERSE_TIME)
    {
        if(shoot_block->trigger_motor.tigger_bolck.reverse_time == 1)
        {  
            if(switch_is_up(shoot_block->shoot_dt7_remote_control->rc.s[LEFT]))
            // 设置反转角度
           { shoot_block->trigger_motor.trigger_motor_angle_set = 
                shoot_block->trigger_motor.add_angle + block_need_angle;}
					 else
					 {
						 
						 shoot_block->trigger_motor.trigger_motor_angle_set = 
                shoot_block->trigger_motor.add_angle -block_need_angle;
					 }
        }
    }
    
    // 堵转检测条件：电流过大且角度误差较大
    if(__fabs(shoot_block->trigger_motor.feedback_current) > BLOCK_CURRENT && 
       __fabs(shoot_block->trigger_motor.trigger_motor_angle_set - 
              shoot_block->trigger_motor.add_angle) > 5 && 
       shoot_block->trigger_motor.tigger_bolck.block_time < BLOCK_TIME)
    {
        // 堵转计时
        shoot_block->trigger_motor.tigger_bolck.block_time++;
        shoot_block->trigger_motor.tigger_bolck.reverse_time = 0;
    }
    // 堵转计时达到阈值，开始反转计时
    else if(shoot_block->trigger_motor.tigger_bolck.block_time == BLOCK_TIME && 
            shoot_block->trigger_motor.tigger_bolck.reverse_time < BLOCK_REVERSE_TIME)
    {
        shoot_block->trigger_motor.tigger_bolck.reverse_time++;
    }
    else  // 无堵转情况
    {
        shoot_block->trigger_motor.tigger_bolck.block_time = 0;
    }
}



