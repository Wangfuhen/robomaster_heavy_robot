#include <stdio.h>
#include "crg_kalman_filter.h"
#include "crg_matrix_cacl.h"

void kalman_filter_init(kalman_filter_t *mode,fp32 Q[2][2],fp32 R[2][2],fp32 A[2][2],fp32 B[2][2],fp32 H[2][2],fp32 x_l_0[2],fp32 P_l_0[2][2])
{
	mode->I[0][0]=1;
	mode->I[0][1]=0;
	mode->I[1][0]=0;
	mode->I[1][1]=1;
    for (uint8_t i = 0; i < 2; i++)
    {
		mode->x_f[i]=0;//先验估算
		mode->Z[i]=0;  //暂时忽略测量量
		mode->U[i]=0;  //暂时忽略控制量
        mode->x_l[i]=x_l_0[i];//后验估算初始化
        for (uint8_t j = 0; i < 2; i++)
        {
            mode->Q[i][j]=Q[i][j];
            mode->R[i][j]=R[i][j];
            mode->A[i][j]=A[i][j];
            mode->B[i][j]=B[i][j];
            mode->H[i][j]=H[i][j];
            mode->P_l[i][j]=P_l_0[i][j]; //后验协方差初始化
			mode->K[i][j]=0;  //卡尔曼增益初始化
			mode->P_f[i][j]=0;  //先验协方差初始化
			
        } 
    }
}

void kalman_filter_calc(kalman_filter_t *kalman_filter,fp32 z_msg[2],fp32 u_msg[2])
{
    //先验估算  (step1)  *//
    matrix_cacl_2_1(kalman_filter->x_f,kalman_filter->A,kalman_filter->x_l);
    fp32 tem_1[2]={0,0};
    matrix_cacl_2_1(tem_1,kalman_filter->B,u_msg);
    matrix_add_2_1(kalman_filter->x_f,kalman_filter->x_f,tem_1,1);
    //先验协方差  (step2)  **//
    matrix_cacl_2_2(kalman_filter->P_f,kalman_filter->A,kalman_filter->P_l);
    fp32 A_T[2][2]={0,0,0,0};
    matrix_transpose_2_2(A_T,kalman_filter->A);
    matrix_cacl_2_2(kalman_filter->P_f,A_T,kalman_filter->P_f);
    matrix_add_2_2(kalman_filter->P_f,kalman_filter->P_f,kalman_filter->Q,1);
    //卡尔曼增益  (step3)  ***//
    fp32 H_T[2][2]={0,0,0,0};
    matrix_transpose_2_2(H_T,kalman_filter->H);
    //分母begin
    fp32 tem_2[2][2]={0,0,0,0};
    matrix_cacl_2_2(tem_2,kalman_filter->H,kalman_filter->P_f);
    matrix_cacl_2_2(tem_2,tem_2,H_T);
    matrix_add_2_2(tem_2,tem_2,kalman_filter->R,1);  
    matrix_inverse_2_2(tem_2,tem_2);//分母end
    matrix_cacl_2_2(kalman_filter->K,kalman_filter->P_f,H_T);  //分子
    matrix_cacl_2_2(kalman_filter->K,kalman_filter->K,tem_2);  //最终结果
    //后验估算  (step4)  ****  //
    matrix_cacl_2_1(kalman_filter->x_l,kalman_filter->H,kalman_filter->x_f);
    matrix_add_2_1(kalman_filter->x_l,z_msg,kalman_filter->x_l,0);
    matrix_cacl_2_1(kalman_filter->x_l,kalman_filter->K,kalman_filter->x_l);
    matrix_add_2_1(kalman_filter->x_l,kalman_filter->x_f,kalman_filter->x_l,1);
    //后验协方差  (step5)  *****//
    matrix_cacl_2_2(kalman_filter->P_l,kalman_filter->K,kalman_filter->H);
    matrix_add_2_2(kalman_filter->P_l,kalman_filter->I,kalman_filter->P_l,0);
    matrix_cacl_2_2(kalman_filter->P_l,kalman_filter->P_l,kalman_filter->P_f);
}

// fp32 kaman_filter_1d_calc(fp32 ResrcData,fp32 ProcessNoise_Q,fp32 MeasureNoise_R)
// {
//     fp32 R = MeasureNoise_R;
//     fp32 Q = ProcessNoise_Q;
//     fp32 x_mid;
//     fp32 x_now;
//     fp32 p_mid ;
//     fp32 p_now;
//     fp32 kg;
//     x_mid=x_last;
//     p_mid=p_last+Q;
//     kg=p_mid/(p_mid+R);
//     x_now=x_mid+kg*(ResrcData-x_mid);
 
//     p_now=(1-kg)*p_mid;
//     p_last = p_now;
//     x_last = x_now;
//     return x_now;

// }