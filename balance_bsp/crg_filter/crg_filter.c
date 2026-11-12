#include "crg_filter.h"
#include "cmsis_os2.h"

/**
 * @brief   crg_filter.c —— 一阶低通滤波器 + 斜坡（速率限制）规划模块
 *
 * 作用：对底盘/云台控制链路中的参考量与反馈量做平滑，抑制阶跃跳变与高频噪声。
 * 本文件包含两类算法：
 *   1) 一阶惯性低通滤波（First_Lpf_Init / First_Lpf_Calc / Low_Pass_Nomal）
 *      把输入量平滑地逼近目标，相当于“惯性平均”；
 *   2) 斜坡规划（slope_init / slope_plan）
 *      把目标值按设定的加速/减速限速斜坡过渡，限制变化速率，保护执行机构。
 *
 * 时间单位约定（本模块统一）：
 *   - 系统 tick 由 osKernelGetTickCount() 获取，单位 ms（毫秒）；
 *   - 秒级时间变量一律以 Seconds 后缀标识，如 lastUpdateTimeSeconds / elapsedTimeSeconds；
 *   - 毫秒级计数变量以 Ms 后缀标识，如 tickCountMs。
 */
/**
 * @brief   初始化一阶低通滤波器（惯性环节）
 * @param   lpf            滤波器结构体指针（存放滤波参数与状态）
 * @param   time_constant  滤波时间常数 τ，单位 s（秒）
 *                         越大滤波越“迟钝”（截止频率越低），越小越跟手
 * @param   frame_time     控制周期 T，单位 s（秒），即两次计算的时间间隔
 * @note    一阶低通截止频率：fc = 1 / (2·π·τ)
 */
void First_Lpf_Init(lpf_t *lpf,fp32 time_constant,fp32 frame_time)
{
    lpf->time_constant=time_constant;  //截至频率
    lpf->frame_time=frame_time;        //采样间隔时间
    lpf->output=0;
    lpf->input=0;

}
/**
 * @brief   一阶低通滤波的单步递推（输入新数据，输出平滑结果）
 * @param   lpf        滤波器结构体指针
 * @param   msg_input  本次原始输入量
 * @return  本次滤波后的输出量
 * @note    递推公式：y(k) = τ/(τ+T)·y(k-1) + T/(τ+T)·x(k)
 *          其中 τ 为时间常数，T 为控制周期 frame_time。
 *          直观理解：本次结果 = 新输入按 T/(τ+T) 占比加权 + 历史输出按 τ/(τ+T) 占比加权，
 *          即“越新的数据占比越小，输出越平滑”。
 */
fp32 First_Lpf_Calc(lpf_t *lpf,fp32 msg_input)
{
    lpf->input=msg_input;
    lpf->output=lpf->time_constant/(lpf->time_constant+lpf->frame_time)*lpf->output+lpf->frame_time/(lpf->time_constant+lpf->frame_time)*lpf->input;
    return lpf->output; 
}
/**
 * @brief   直通式一阶低通（带输入异常/断连保护）
 * @param   input_msg   本次输入量（int32 类型）
 * @param   parameter   平滑系数，取值范围 (0,1)；越接近 1 越平滑、越迟钝
 * @return  本次滤波输出量
 * @note    设计流程：
 *          1. 记录“上一次输出 lastOutputValue”与“上一次更新时刻 lastUpdateTimeSeconds”；
 *          2. 计算本次与上次更新的时间间隔 elapsedTimeSeconds-lastUpdateTimeSeconds；
 *             若间隔 ≥ 150ms，判定输入异常/断连，直接返回上一次输出（不滤波，防污染）；
 *          3. 正常时按  输出 = parameter·上次输出 + (1-parameter)·本次输入  递推。
 *
 *          【注意】此函数在原工程中即存在逻辑瑕疵：lastUpdateTimeSeconds 在判断前
 *          已被赋值为当前时刻（lastUpdateTimeSeconds = elapsedTimeSeconds 在前），
 *          导致 150ms 保护分支恒不触发。此处忠实保留原逻辑；如需修复请另行处理。
 */
fp32 Low_Pass_Nomal( int32_t input_msg,fp32 parameter)  //直接形式低通滤波
{
    // 上一次的输出值
    static fp32 lastOutputValue=0;
    // 上一次的时间戳
    static fp32 lastUpdateTimeSeconds=0;
    // 获取当前时间
    uint32_t tickCountMs=osKernelGetTickCount();//获取当前时间
    // 将时间戳转换为秒（假设系统时钟频率为1000Hz）
    fp32 elapsedTimeSeconds=(fp32)tickCountMs /1000;//默认配置1000tick
    // 更新上一次的时间戳
    lastUpdateTimeSeconds=elapsedTimeSeconds;
    // 如果时间差大于等于150ms（即频率不达标），则直接返回上一次的输出值，不进行滤波
    if(elapsedTimeSeconds-lastUpdateTimeSeconds>=150) //频率不达标
    {
        return lastOutputValue;
    }
    // 根据滤波参数计算新的输出值
    fp32 out=parameter*lastOutputValue + (1.0f-parameter)*input_msg;
    // 更新上一次的输出值
    lastOutputValue=out;
    // 返回新的输出值
    return out;
}

/**
 * @brief   斜坡规划初始化
 * @param   slope_plan        斜坡规划结构体指针
 * @param   accl_velocity     加速斜坡限速：每个控制周期允许增加的最大量
 * @param   retarded_velocity 减速斜坡限速：每个控制周期允许减少的最大量
 * @param   real_data         指向“实际/当前值”的指针（实时反馈）
 * @param   target            指向“目标值”的指针
 */
void slope_init(slope_plan_t *slope_plan ,fp32 accl_velocity,fp32 retarded_velocity,fp32 *real_data,fp32 *target)
{
	slope_plan->accl_velocity=accl_velocity;
	slope_plan->retarded_velocity=retarded_velocity;
	slope_plan->real_data=real_data;
	slope_plan->target=target;
	slope_plan->out=0;
	slope_plan->now_plan=0;
}

/**
 * @brief   斜坡规划单步计算：把输出向目标值按限速斜坡逐步逼近
 * @param   slope_plan  结构体指针（内部读写 now_plan 与 out）
 * @return  本次规划输出量
 * @note    按当前位置分三种情形：
 *          1) now_plan == 0：起步段，按加速限速 accl_velocity 逼近目标；
 *          2) now_plan >  0：正向段，加速用 accl_velocity、减速用 retarded_velocity；
 *          3) now_plan <  0：反向段，加速/减速限速同理。
 *          每个控制周期输出至多变化一个“限速”量，避免目标突变造成执行器冲击；
 *          若与目标差值小于单步限速，则直接输出目标值（到位收敛）。
 */
fp32 slope_plan(slope_plan_t *slope_plan)
{
	if((((*(slope_plan->target)>=(*(slope_plan->real_data)))&&((*(slope_plan->real_data))>=slope_plan->now_plan))||((*(slope_plan->target)<=(*(slope_plan->real_data)))&&((*(slope_plan->real_data))<=slope_plan->now_plan))))
	{
		slope_plan->out=*(slope_plan->real_data);
	}
	if(slope_plan->now_plan==0.0f)//零点两个方向加速情况
	{
		if(*(slope_plan->target)>slope_plan->now_plan)//加速环节
		{
			if(__fabs(*(slope_plan->target)-slope_plan->now_plan)>slope_plan->accl_velocity)//还差一个加速度距离
			{
				slope_plan->out+=slope_plan->accl_velocity;
			}
			else
			{
				slope_plan->out=*(slope_plan->target);
			}
		}
		else if(*(slope_plan->target)<slope_plan->now_plan) //零负加速
		{
			if(__fabs(*(slope_plan->target)>slope_plan->now_plan)>slope_plan->accl_velocity)
			{
				slope_plan->out-=slope_plan->accl_velocity;
			}
			else
			{
				slope_plan->out=*(slope_plan->target);
			}
		}
		
	}
	else if(slope_plan->now_plan>0.0f)  //正加速情况
	{
		if(*(slope_plan->target)>slope_plan->now_plan)//加速环节
		{
			if(__fabs(*(slope_plan->target)-slope_plan->now_plan)>slope_plan->accl_velocity)
			{
				slope_plan->out+=slope_plan->accl_velocity;
			}
			else
			{
				slope_plan->out=*(slope_plan->target);
			}
		}
		else if(*(slope_plan->target)<slope_plan->now_plan)
		{
			if(__fabs(*(slope_plan->target)-slope_plan->now_plan)>slope_plan->retarded_velocity)
			{
				slope_plan->out-=slope_plan->retarded_velocity;
			}
			else
			{
				slope_plan->out=*(slope_plan->target);
			}
		}
	}
	else if(slope_plan->now_plan<0.0f)//负加速情况
	{
		if(*(slope_plan->target)<slope_plan->now_plan)//加速环节
		{
			if(__fabs(*(slope_plan->target)-slope_plan->now_plan)>slope_plan->accl_velocity)
			{
				slope_plan->out-=slope_plan->accl_velocity;
			}
			else
			{
				slope_plan->out=*(slope_plan->target);
			}
		}
		else if(*(slope_plan->target)>slope_plan->now_plan)
		{
			if(__fabs(*(slope_plan->target)-slope_plan->now_plan)>slope_plan->retarded_velocity)
			{
				slope_plan->out+=slope_plan->retarded_velocity;
			}
			else
			{
				slope_plan->out=*(slope_plan->target);
			}
		}
	}
	slope_plan->now_plan=slope_plan->out;
	return slope_plan->out;
}
