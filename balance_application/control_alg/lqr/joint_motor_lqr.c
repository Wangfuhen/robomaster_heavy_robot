#include "joint_motor_lqr.h"
#include "user_lib.h"
/**
  ************************************* Copyright ******************************
  *
  *                 (C) Copyright {2022},ZQJ,China, Ares.
  *                            All Rights Reserved
  *
  ******************************************************************************
  *
  * FileName   : lqr.c
  * Version    : v1.0
  * Author     : ZQJ
  * Date       : 2023-02-22
  * Description: LQR 控制器
  *
  ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/


#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

// VALUE



// FUNCTIONs


/**
  * @Name    lqr_init
  * @brief   参数初始化
  * @param   lqr: [输入/出]
	**			 speed_factor[4]: [输入/出]
	**			 angle_factor[4]: [输入/出]
	**			 current_factor[4]: [输入/出]
	**			 max_out: [输入/出]
  * @retval
  * @author  ZQJ
  * @Data    2023-02-22
 **/


void lqr_init(lqr_type_def *lqr, fp32 speed_factor[4],fp32 angle_factor[4],fp32 current_factor[4],fp32 max_out)
{
    lqr->speed_lqr.mode = BACKWORD;
    lqr->speed_lqr.Kp = speed_factor[0];
    lqr->speed_lqr.Ki = speed_factor[1];
    lqr->speed_lqr.I_max_out = speed_factor[2];
    lqr->speed_lqr.max_out = speed_factor[3];
    lqr->speed_lqr.out = lqr->speed_lqr.Pout = lqr->speed_lqr.Iout = 0.0f;

    lqr->angle_lqr.mode = BACKWORD;
    lqr->angle_lqr.Kp = angle_factor[0];
    lqr->angle_lqr.Ki = angle_factor[1];
    lqr->angle_lqr.I_max_out = angle_factor[2];
    lqr->angle_lqr.max_out = angle_factor[3];
    lqr->angle_lqr.out = lqr->angle_lqr.Pout = lqr->angle_lqr.Iout = 0.0f;

    // 注意电流是正反馈
    lqr->current_lqr.mode = FORWORD;
    lqr->current_lqr.Kp = current_factor[0];
    lqr->current_lqr.Ki = current_factor[1];
    lqr->current_lqr.I_max_out = current_factor[2];
    lqr->current_lqr.max_out = current_factor[3];
    lqr->current_lqr.out = lqr->current_lqr.Pout = lqr->current_lqr.Iout = 0.0f;

    lqr->max_out = max_out;
    lqr->out = 0;
}


/**
  * @Name    lqr_element_cal
  * @brief   子因素计算
  * @param   lqr_element: [输入/出]
  **			 fd: [输入/出]
  **			 set: [输入/出]
  * @retval
  * @author  ZQJ
  * @Data    2023-02-22
 **/


fp32 lqr_element_cal(lqr_element_type_def *lqr_element,fp32 fd,fp32 set)
{
    if(lqr_element == NULL)
    {
        return 0;
    }

    static fp32 error = 0;
    lqr_element->set = set;
    lqr_element->fdb = fd;
    if(lqr_element->mode == BACKWORD) // 负反馈
    {
        error = lqr_element->set - lqr_element->fdb;
        lqr_element->Pout = lqr_element->Kp * error;
        lqr_element->Iout += lqr_element->Ki * error;
        // KI 限幅
        LimitMax(lqr_element->Iout, lqr_element->I_max_out);
    }
    else  // 正反馈
    {
        lqr_element->Iout = 0;
        error = lqr_element->fdb /*- lqr_element->set*/ ;
        lqr_element->Pout = lqr_element->Kp * error;
        lqr_element->Iout = 0;
    }
    // OUT
    lqr_element->out = lqr_element->Pout + lqr_element->Iout;
    LimitMax(lqr_element->out,lqr_element->max_out );
    return lqr_element->out;
}


/**
  * @Name    lqr_cal
  * @brief   lqr计算
  * @param   lqr: [输入/出]
  **			 fd[3]: [输入/出]
  **			 set[3]: [输入/出]
  * @retval
  * @author  ZQJ
  * @Data    2023-02-22
 **/


fp32 lqr_cal(lqr_type_def *lqr,fp32 fd[3],fp32 set[3])
{
    if(lqr == NULL)
    {
        return 0;
    }
    lqr_element_cal(&lqr->speed_lqr,fd[0],set[0]);
    lqr_element_cal(&lqr->angle_lqr,fd[1],set[1]);
    lqr_element_cal(&lqr->current_lqr,fd[2],set[2]);
    lqr->out = lqr->speed_lqr.out + lqr->angle_lqr.out +lqr->current_lqr .out;      //分为LQR 速度 角度 电流 三值相加为输出
    LimitMax(lqr->out, lqr->max_out );
    return lqr->out;
}

int16_t shoot_PI_LQR(fp32 KP,fp32 KI,fp32 K1,fp32 K2,fp32 K3,//PI_LQR Serise
                     fp32 angle_get,fp32 speed_get,fp32 current_get,
                     fp32 angle_set)
{
    fp32 current=0,first_out=0;
    fp32 KP_angle;
    static fp32 KI_angle=0;
    KP_angle=(angle_set-angle_get);
    KI_angle+=(angle_set-angle_get);
    abs_limit(&KI_angle,2.0f);
    first_out = KP*KP_angle+KI*KI_angle;

    current=(first_out - (K2*speed_get + K3*(-current_get)));//通过角度和速度求出理论要求输入的角加速度
    abs_limit(&current,8000.0f);
    return (int16_t)current;
}

fp32  forward_feddback_cacl(forward_feed *forward_fedd)
{
	fp32 out=0;
	out=(forward_fedd->in-forward_fedd->last_in)/forward_fedd->T+forward_fedd->in;
	forward_fedd->last_in=forward_fedd->in;
	return out;
}

/************************ (C) COPYRIGHT ZQJ *****END OF FILE****/
