#ifndef CRG_MATRIX_CACL_H
#define CRG_MATRIX_CACL_H

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

#include "c_data_typedef.h"
/**
 * @brief 2X2矩阵乘法
 * @return none
 * @note 默认按形参顺序乘法，即结果矩阵=矩阵1*矩阵2
 * @example matrix_cacl_2_2(a[2][2],b[2][2],c[2][2])
 * @author  Ares_茶荣果
 * @version 1.0
 * @date    2025-01-30
 */
extern void matrix_cacl_2_2(fp32 result[2][2], fp32 matrix_1[2][2], fp32 matrix_2[2][2]);

/**
 * @brief 2X1矩阵乘法
 * @return none
 * @note 默认按形参顺序乘法，即结果矩阵=矩阵1*矩阵2
 * @example matrix_cacl_2_2(a[2],b[2][2],c[2])
 * @author  Ares_茶荣果
 * @version 1.0
 * @date    2025-01-30
 */
extern void matrix_cacl_2_1(fp32 result[2], fp32 matrix_1[2][2], fp32 matrix_2[2]);

/**
 * @brief 3X3矩阵乘法
 * @return none
 * @note 默认按形参顺序乘法，即结果矩阵=矩阵1*矩阵2
 * @example matrix_cacl_3_3(a[3][3],b[3][3],c[3][3])
 * @author  Ares_茶荣果
 * @version 1.0
 * @date    2025-01-30
 */
extern void matrix_cacl_3_3(fp32 result[3][3], fp32 matrix_1[3][3], fp32 matrix_2[3][3]);

/**
 * @brief 2X2矩阵加法
 * @return none 
 * * @note 默认按形参顺序加法，即结果矩阵=矩阵1+-矩阵2
 * @param flag 0:默认按形参顺序减法，即结果矩阵=矩阵1-矩阵2
 * @param flag 1:按形参顺序加法，即结果矩阵=矩阵1+矩阵2
 * @example matrix_add_2_2(a[2][2],b[2][2],c[2][2])
 * @author  Ares_茶荣果
 * @version 1.0
 * @date    2025-01-30
 */
extern void matrix_add_2_2(fp32 result[2][2], fp32 matrix_1[2][2], fp32 matrix_2[2][2], fp32 flag);

/**
 * @brief 2X1矩阵加法
 * @return none 
 * * @note 默认按形参顺序加法，即结果矩阵=矩阵1+-矩阵2
 * @param flag 0:默认按形参顺序减法，即结果矩阵=矩阵1-矩阵2
 * @param flag 1:按形参顺序加法，即结果矩阵=矩阵1+矩阵2
 * @example matrix_add_2_1(a[2],b[2],c[2])
 * @author  Ares_茶荣果
 * @version 1.0
 * @date    2025-01-30
 */
extern void matrix_add_2_1(fp32 result[2], fp32 matrix_1[2], fp32 matrix_2[2], fp32 flag);

/**
 * @brief 3X3矩阵加法
 * @return none
 * @note 默认按形参顺序加法，即结果矩阵=矩阵1+矩阵2
 * @example matrix_add_3_3(a[3][3],b[3][3],c[3][3])
 * @author  Ares_茶荣果
 * @version 1.0
 * @date    2025-01-30
 */
extern void matrix_add_3_3(fp32 result[3][3], fp32 matrix_1[3][3], fp32 matrix_2[3][3]);

/**
 * @brief 2X2矩阵减法
 * @return none
 * @note 默认按形参顺序减法，即结果矩阵=矩阵1-矩阵2
 * @example matrix_sub_2_2(a[2][2],b[2][2],c[2][2])
 * @author  Ares_茶荣果
 * @version 1.0
 * @date    2025-01-30
*/
extern void matrix_sub_2_2(fp32 result[2][2], fp32 matrix_1[2][2], fp32 matrix_2[2][2]);

/**
 * @brief 3X3矩阵减法
 * @return none
 * @note 默认按形参顺序减法，即结果矩阵=矩阵1-矩阵2
 * @example matrix_sub_3_3(a[3][3],b[3][3],c[3][3])
 * @author  Ares_茶荣果
 * @version 1.0
 * @date    2025-01-30
*/
extern void matrix_sub_3_3(fp32 result[3][3], fp32 matrix_1[3][3], fp32 matrix_2[3][3]);

/**
 * @brief 2X2矩阵转置
 * @return none
 * @note 默认按形参顺序转置，即结果矩阵=矩阵1转置
 * @example matrix_transpose_2_2(a[2][2],b[2][2])
 * @author  Ares_茶荣果
 * @version 1.0
 * @date    2025-01-30
*/
extern void matrix_transpose_2_2(fp32 result[2][2], fp32 matrix[2][2]);

/**
 * @brief 3X3矩阵转置
 * @return none
 * @note 默认按形参顺序转置，即结果矩阵=矩阵1转置
 * @example matrix_transpose_3_3(a[3][3],b[3][3])
 * @author  Ares_茶荣果
 * @version 1.0
 * @date    2025-01-30
*/
extern void matrix_transpose_3_3(fp32 result[3][3], fp32 matrix[3][3]);

/**
 * @brief 2X2矩阵求逆
 * @return none
 * @note 默认按形参顺序求逆，即结果矩阵=矩阵1逆
 * @example matrix_inverse_2_2(a[2][2],b[2][2])
 * @author  Ares_茶荣果
 * @version 1.0
 * @date    2025-01-30
*/
extern void matrix_inverse_2_2(fp32 result[2][2], fp32 matrix[2][2]);

#endif
