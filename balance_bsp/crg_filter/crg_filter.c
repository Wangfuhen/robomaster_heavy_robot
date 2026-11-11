#include "crg_filter.h"
#include "FreeRTOS.h"
#include "task.h"
void First_Lpf_Init(lpf_t *lpf,fp32 time_constant,fp32 frame_time)
{
    lpf->time_constant=time_constant;  //截至频率
    lpf->frame_time=frame_time;        //采样间隔时间
    lpf->output=0;
    lpf->input=0;

}
fp32 First_Lpf_Calc(lpf_t *lpf,fp32 msg_input)
{
    lpf->input=msg_input;
    lpf->output=lpf->time_constant/(lpf->time_constant+lpf->frame_time)*lpf->output+lpf->frame_time/(lpf->time_constant+lpf->frame_time)*lpf->input;
    return lpf->output; 
}
fp32 Low_Pass_Nomal( int32_t input_msg,fp32 parameter)  //直接形式低通滤波
{
    // 上一次的输出值
    static fp32 out_last=0;
    // 上一次的时间戳
    static fp32 time_last=0;
    // 获取当前时间
    TickType_t tickCount=xTaskGetTickCount();//获取当前时间
    // 将时间戳转换为秒（假设系统时钟频率为1000Hz）
    fp32 time=(fp32)tickCount /1000;//默认配置1000tick
    // 更新上一次的时间戳
    time_last=time;
    // 如果时间差大于等于150ms（即频率不达标），则直接返回上一次的输出值，不进行滤波
    if(time-time_last>=150) //频率不达标
    {
        return out_last;
    }
    // 根据滤波参数计算新的输出值
    fp32 out=parameter*out_last + (1.0f-parameter)*input_msg;
    // 更新上一次的输出值
    out_last=out;
    // 返回新的输出值
    return out;
}
