#ifndef PID_H
#define PID_H
#include "c_data_typedef.h"

enum PID_MODE
{
    PID_POSITION=0,  //位置式
    PID_DELTA,
};

typedef struct 
{
    uint8_t mode;
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 max_out;  //计算输出最大限幅
    fp32 max_iout; //积分最大限幅

    fp32 set;  //expect
    fp32 feed_back; //feedback

    fp32 out;  //最后结果
    fp32 pout;
    fp32 iout;
    fp32 dout;
    fp32 dbuf[3];
    fp32 error[3];

    
}pid_type_def;

extern void pid_init(pid_type_def *user_pid,uint8_t mode ,const fp32 PID[3],fp32 max_out,fp32 max_iout); 

extern void pid_clear(pid_type_def *user_pid);

extern fp32 pid_calc(pid_type_def *user_pid,fp32 feed_back,fp32 set);

extern fp32 pid_calc_gimbal(pid_type_def *user_pid,fp32 feed_back,fp32 set);//对云台进行特殊处理 慎用

#endif
