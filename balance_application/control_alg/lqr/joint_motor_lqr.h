#ifndef JOINT_MOTOR_LQR_H
#define JOINT_MOTOR_LQR_H
/*
                                                                                                                                                                    
                                                                                                                                                                                   
                         ||                  |||                                                                                              |||                   |||||||||      
                         ||           ||||||||||||||       ||||||||||||||||||        |||||||                             ||||||||      ||||||||||||||            |||||||||||||||   
                        ||||           ||||||||||||||       |||||||||||||||||      ||||||||||                          ||||||||||||     ||||||||||||||          |||||      |||||   
                        ||||             ||||   ||||||        ||||        |||     ||||   ||||                         |||||    ||||       ||||   ||||||        ||||          |||   
                        ||||             ||||    |||||        ||||         ||     |||      ||                        ||||        |||      ||||    |||||       ||||            ||   
                       ||||||            ||||     ||||        ||||         ||    ||||      ||                       ||||         |||      ||||     ||||      ||||             ||   
                       || |||            ||||     ||||        ||||               ||||      ||                      ||||           ||      ||||     ||||      ||||                  
                      ||| |||            ||||     ||||        ||||       ||       ||||                             ||||                   ||||     ||||     |||||                  
                      ||  ||||           ||||    ||||         ||||       ||       ||||||                           ||||                   ||||    ||||      ||||                   
                      ||   |||           ||||   ||||          ||||       ||        |||||||                        ||||                    ||||   ||||       ||||                   
                     |||   ||||          ||||||||||           |||||||||||||         ||||||||                      ||||                    ||||||||||        ||||                   
                     ||||||||||          ||||  ||||           |||||||||||||           |||||||                     ||||                    ||||  ||||        ||||        |||||||||||
                    |||||||||||          ||||   ||||          ||||       ||             ||||||                    ||||                    ||||   ||||       ||||        |||||||||||
                    |||     ||||         ||||   ||||          ||||       ||               ||||                    ||||                    ||||   ||||       |||||           ||||   
                    ||      ||||         ||||    ||||         ||||       ||      ||        |||                    ||||                    ||||    ||||       ||||           ||||   
                   |||       |||         ||||    |||||        ||||        |  ||  |||       |||                     ||||                   ||||    |||||      |||||          ||||   
                   ||        ||||        ||||     ||||        ||||          |||  |||       |||                     ||||           ||      ||||     ||||      |||||          ||||   
                   ||         |||        ||||     |||||       ||||          |||  ||||      |||                      ||||          ||      ||||     |||||      |||||         ||||   
                  |||         ||||       |||||     ||||||     ||||         |||   ||||||  ||||                       ||||         |||      |||||     ||||||     ||||||      |||||   
               |||||||||   |||||||||  ||||||||||   ||||||| |||||||||||||||||||    ||||||||||                         |||||     ||||    ||||||||||   |||||||      ||||||||||||||      
               |||||||||   |||||||||  ||||||||||    |||||| |||||||||||||||||||      ||||||                             ||||||||||||    ||||||||||    ||||||        |||||||||       
                                                                                                                         ||||||||                                                  
                                                                                                                                                                                   
                                                                                                ||||||||||||||||                                                                   
                                                                                                ||||||||||||||||                                                                                                                                                                   
 
*/

#include "main.h"
#include "c_data_typedef.h"

typedef enum
{
    BACKWORD = 0,  // 负反馈
    FORWORD        // 正反馈

} lqr_element_m;

typedef struct
{
    fp32 Kp;
    fp32 Ki;
    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 I_max_out;
    fp32 max_out;
    lqr_element_m mode;

} lqr_element_type_def;


typedef struct
{
    lqr_element_type_def speed_lqr;
    lqr_element_type_def angle_lqr;
    lqr_element_type_def current_lqr;
    // YOU CAN ADD YOUR OWN ELEMENT

    fp32 out;
    fp32 max_out;

} lqr_type_def;

typedef struct
{
	fp32 T;
	fp32 last_in;
	fp32 in;
}forward_feed;

extern void lqr_init(lqr_type_def *lqr,fp32 speed_factor[3],fp32 angle_factor[3],fp32 current_factor[3],fp32 max_out);
extern fp32 lqr_element_cal(lqr_element_type_def *lqr_element,fp32 fd,fp32 set);
extern fp32 lqr_cal(lqr_type_def *lqr,fp32 fd[3],fp32 set[3]);
extern int16_t shoot_PI_LQR(fp32 KP,fp32 KI,fp32 K1,fp32 K2,fp32 K3,//PI_LQR Serise
                     fp32 angle_get,fp32 speed_get,fp32 current_get,
                     fp32 angle_set);
extern fp32  forward_feddback_cacl(forward_feed *forward_fedd);
#endif
