#include "pid.h"
#include "main.h"

#define LimitMax(user_data,max)  \
{                                \
    if((user_data)>(max))        \
    {                            \
        (user_data)=(max);       \
    }                            \
    else if((user_data)<-(max))  \
    {                            \
        (user_data)=-(max);      \
    }                            \
}                                
                         
void pid_init(pid_type_def *user_pid,uint8_t mode ,const fp32 PID[3],fp32 max_out,fp32 max_iout)  //完成对pid结构体的初始化传递参数，进而简化代码
{
    if(user_pid==NULL||PID==NULL)
    {
        return;
    }
    else
    {
    user_pid->kp=PID[0];
    user_pid->ki=PID[1];
    user_pid->kd=PID[2];
    user_pid->mode=mode;
    user_pid->max_out=max_out;
    user_pid->max_iout=max_iout;
    user_pid->dbuf[0]=user_pid->dbuf[1]=user_pid->dbuf[2]=0.0f;
    user_pid->error[0]=user_pid->error[0]=user_pid->error[0]=0.0f;
    user_pid->dout=user_pid->iout=user_pid->pout=user_pid->out=0.0f;
    }

}

void pid_clear(pid_type_def *user_pid)  //对pid结构体清算归零 
{
    if(user_pid==NULL)
    {
        return;
    }
    else
    {
        user_pid->error[0]=user_pid->error[1]=user_pid->error[2]=0.0f;
        user_pid->dbuf[0]=user_pid->dbuf[1]=user_pid->dbuf[2]=0.0f;
        user_pid->dout=user_pid->iout=user_pid->pout=user_pid->out=0.0f;
        user_pid->feed_back=user_pid->set=0.0f;
    }
}

fp32 pid_calc(pid_type_def *user_pid,fp32 feed_back,fp32 set) //进行pid计算 pid结构体 反馈 期望
{
    if(user_pid==NULL)
    {
        return 0.0f;
    }
    else
    {
        user_pid->error[2]=user_pid->error[1];
        user_pid->error[1]=user_pid->error[0];
        user_pid->set=set;
        user_pid->feed_back=feed_back;
        user_pid->error[0]=set-feed_back;
        if(user_pid->mode==PID_POSITION) //位置式
        {
            user_pid->pout=user_pid->kp*user_pid->error[0];
            user_pid->iout+=user_pid->ki*user_pid->error[0];
            user_pid->dout=user_pid->kd*(user_pid->error[0]-user_pid->error[1]);
            if(user_pid->max_iout!=0)
            {
                LimitMax(user_pid->iout,user_pid->max_iout);//积分限度
            }
            user_pid->out=user_pid->pout+user_pid->iout+user_pid->dout;
            LimitMax(user_pid->out,user_pid->max_out); //输出限幅
        }
        else if(user_pid->mode==PID_DELTA)
        {
            user_pid->pout=user_pid->kp*(user_pid->error[0]-user_pid->error[1]); //位置d
            user_pid->iout=user_pid->ki*user_pid->error[0];  //位置p
			user_pid->dbuf[2]=user_pid->dbuf[1];
			user_pid->dbuf[1]=user_pid->dbuf[0];
			user_pid->dbuf[0]=(user_pid->error[0]-2*user_pid->error[1]+user_pid->error[2]);
            user_pid->dout=user_pid->kd*user_pid->dbuf[0];  //误差微分
            user_pid->out+=(user_pid->pout+user_pid->iout+user_pid->dout);
            LimitMax(user_pid->out,user_pid->max_out);  //输出限幅
        }
        return user_pid->out;
    }
    

}

fp32 pid_calc_gimbal(pid_type_def *user_pid,fp32 feed_back,fp32 set)
{
    if(user_pid==NULL)
    {
        return 0.0f;
    }
    else
    {
        user_pid->error[2]=user_pid->error[1];
        user_pid->error[1]=user_pid->error[0];
        user_pid->set=set;
        user_pid->feed_back=feed_back;
        user_pid->error[0]=set-feed_back;
        if(user_pid->error[0]>3.14159265358979f)
		{
			user_pid->error[0]=user_pid->error[0]-2*3.14159265358979f;
		}
		if(user_pid->error[0]<-3.14159265358979f)
		{
			user_pid->error[0]=user_pid->error[0]+2*3.14159265358979f;
		}
        if(user_pid->mode==PID_POSITION) //位置式
        {
            user_pid->pout=user_pid->kp*user_pid->error[0];
            user_pid->iout+=user_pid->ki*user_pid->error[0];
            user_pid->dout=user_pid->kd*(user_pid->error[0]-user_pid->error[1]);
            if(user_pid->max_iout!=0)
            {
                LimitMax(user_pid->iout,user_pid->max_iout);//积分限度
            }
            user_pid->out=user_pid->pout+user_pid->iout+user_pid->dout;
            LimitMax(user_pid->out,user_pid->max_out); //输出限幅
        }
        else if(user_pid->mode==PID_DELTA)
        {
            user_pid->pout=user_pid->kp*(user_pid->error[0]-user_pid->error[1]); //位置d
            user_pid->iout=user_pid->ki*user_pid->error[0];  //位置p
			user_pid->dbuf[2]=user_pid->dbuf[1];
			user_pid->dbuf[1]=user_pid->dbuf[0];
			user_pid->dbuf[0]=(user_pid->error[0]-2*user_pid->error[1]+user_pid->error[2]);
            user_pid->dout=user_pid->kd*user_pid->dbuf[0];  //误差微分
            user_pid->out+=(user_pid->pout+user_pid->iout+user_pid->dout);
            LimitMax(user_pid->out,user_pid->max_out);  //输出限幅
        }		
    }	
}