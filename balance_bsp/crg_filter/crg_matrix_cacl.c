#include "crg_matrix_cacl.h"

void matrix_cacl_2_2(fp32 result[2][2], fp32 matrix_1[2][2], fp32 matrix_2[2][2])
{
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            result[i][j] = matrix_1[i][0] * matrix_2[0][j] + matrix_1[i][1] * matrix_2[1][j];
        }

    }
}

void matrix_cacl_2_1(fp32 result[2], fp32 matrix_1[2][2], fp32 matrix_2[2])
{
    for (int i = 0; i < 2; i++)
    {
        result[i] = matrix_1[i][0] * matrix_2[0] + matrix_1[i][1] * matrix_2[1];
    }
}

void matrix_cacl_3_3(fp32 result[3][3], fp32 matrix_1[3][3], fp32 matrix_2[3][3])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            result[i][j] = matrix_1[i][0] * matrix_2[0][j] + matrix_1[i][1] * matrix_2[1][j] + matrix_1[i][2] * matrix_2[2][j];
        }
    }
}

void matrix_add_2_2(fp32 result[2][2], fp32 matrix_1[2][2], fp32 matrix_2[2][2], fp32 flag)
{
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            if(flag==1)
            {
                result[i][j] = matrix_1[i][j] + matrix_2[i][j];
            }
            else if(flag==0)
            {
                result[i][j] = matrix_1[i][j] - matrix_2[i][j];
            }
            else
            {
               return;
            }  
        }
    }
}

void matrix_add_2_1(fp32 result[2], fp32 matrix_1[2], fp32 matrix_2[2], fp32 flag)
{
    for (int i = 0; i < 2; i++)
    {
        if(flag==1)
        {
            result[i] = matrix_1[i] + matrix_2[i];
       
        }
        else if(flag==0)
        {
            result[i] = matrix_1[i] - matrix_2[i];
        }
        else
        {
            return;
        } 
    } 
}
void matrix_add_3_3(fp32 result[3][3], fp32 matrix_1[3][3], fp32 matrix_2[3][3])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            result[i][j] = matrix_1[i][j] + matrix_2[i][j];
        }
    }
}

void matrix_sub_2_2(fp32 result[2][2], fp32 matrix_1[2][2], fp32 matrix_2[2][2])
{
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            result[i][j] = matrix_1[i][j] - matrix_2[i][j];
        }
    }
}

void matrix_sub_3_3(fp32 result[3][3], fp32 matrix_1[3][3], fp32 matrix_2[3][3])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            result[i][j] = matrix_1[i][j] - matrix_2[i][j];
        }
    }
}

void matrix_transpose_2_2(fp32 result[2][2], fp32 matrix[2][2])
{
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            result[i][j] = matrix[j][i];
        }
    } 
}

void matrix_transpose_3_3(fp32 result[3][3], fp32 matrix[3][3])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            result[i][j] = matrix[j][i];
        }
    }
}

void matrix_inverse_2_2(fp32 result[2][2], fp32 matrix[2][2])
{
    fp32 det = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
    result[0][0] = matrix[1][1] / det;
    result[0][1] = -matrix[0][1] / det; 
    result[1][0] = -matrix[1][0] / det;
    result[1][1] = matrix[0][0] / det;
}


