#include <math.h>
#include "misc.h"





/*-------------------------------------------------------------------------*/
/**
  @brief    
  @param    
  @return   
  

 */
/*--------------------------------------------------------------------------*/
U32 computeMeanStd(FLT* const mean, FLT* const std, const FLT array[][CHN], U32 count)
{
    U32 i = 0;
    FLT sum = 0;
    
    if (count < 2)
    {
        return -1;
    }

    for (i = 0; i < count; i++)
    {
        FLT det = 0;

        det = sqrtf(array[i][X]*array[i][X] + array[i][Y]*array[i][Y] + array[i][Z]*array[i][Z]);
        sum += det;
    }
    *mean = sum / count;

    sum = 0;
    for (i = 0; i < count; i++)
    {
        FLT det = 0;

        det = sqrtf(array[i][X]*array[i][X] + array[i][Y]*array[i][Y] + array[i][Z]*array[i][Z]);
        sum += (det - *mean) * (det - *mean);
    }
    *std = sqrtf(sum / (count - 1.0F));

    return 0;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    
  @param    
  @return   
  

 */
/*--------------------------------------------------------------------------*/
void dcm2euler(const FLT cbn[3][3], FLT* const pyaw, FLT* const ppitch, FLT* const proll)
{
    *pyaw = atan2f(cbn[Y][X], cbn[X][X]);
    *ppitch = asinf(-cbn[Z][X]);
    *proll = atan2f(cbn[Z][Y], cbn[Z][Z]);
}

/*-------------------------------------------------------------------------*/
/**
  @brief    
  @param    
  @return   
  

 */
/*--------------------------------------------------------------------------*/
void euler2dcm(FLT cbn[3][3], FLT fyaw, FLT fpitch, FLT froll)
{
    cbn[0][0] = cosf(fpitch) * cosf(fyaw);
    cbn[0][1] = sinf(froll) * sinf(fpitch) * cosf(fyaw) - cosf(froll) * sinf(fyaw);
    cbn[0][2] = cosf(froll) * sinf(fpitch) * cosf(fyaw) + sinf(froll) * sinf(fyaw);

    cbn[1][0] = cosf(fpitch) * sinf(fyaw);
    cbn[1][1] = sinf(froll) * sinf(fpitch) * sinf(fyaw) + cosf(froll) * cosf(fyaw);
    cbn[1][2] = cosf(froll) * sinf(fpitch) * sinf(fyaw) - sinf(froll) * cosf(fyaw);

    cbn[2][0] = -sinf(fpitch);
    cbn[2][1] = sinf(froll) * cosf(fpitch);
    cbn[2][2] = cosf(froll) * cosf(fpitch);
}

/*-------------------------------------------------------------------------*/
/**
  @brief    
  @param    
  @return   
  

 */
/*--------------------------------------------------------------------------*/
void euler2q(FLT q[4], FLT fyaw, FLT fpitch, FLT froll)
{
    q[0] = cosf(froll / 2) * cosf(fpitch / 2) * cosf(fyaw / 2) + sinf(froll / 2) * sinf(fpitch / 2) * sinf(fyaw / 2);
    q[1] = sinf(froll / 2) * cosf(fpitch / 2) * cosf(fyaw / 2) - cosf(froll / 2) * sinf(fpitch / 2) * sinf(fyaw / 2);
    q[2] = cosf(froll / 2) * sinf(fpitch / 2) * cosf(fyaw / 2) + sinf(froll / 2) * cosf(fpitch / 2) * sinf(fyaw / 2);
    q[3] = cosf(froll / 2) * cosf(fpitch / 2) * sinf(fyaw / 2) - sinf(froll / 2) * sinf(fpitch / 2) * cosf(fyaw / 2);
}

/*-------------------------------------------------------------------------*/
/**
  @brief    
  @param    
  @return   
  

 */
/*--------------------------------------------------------------------------*/
void q2dcm(FLT q[4], FLT cbn[3][3])
{
    cbn[0][0] = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
    cbn[0][1] = 2 * (q[1] * q[2] - q[0] * q[3]);
    cbn[0][2] = 2 * (q[1] * q[3] + q[0] * q[2]);

    cbn[1][0] = 2 * (q[1] * q[2] + q[0] * q[3]);
    cbn[1][1] = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
    cbn[1][2] = 2 * (q[2] * q[3] - q[0] * q[1]);

    cbn[2][0] = 2 * (q[1] * q[3] - q[0] * q[2]);
    cbn[2][1] = 2 * (q[2] * q[3] + q[0] * q[1]);
    cbn[2][2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
}

/*-------------------------------------------------------------------------*/
/**
  @brief    
  @param    
  @return   
  

 */
/*--------------------------------------------------------------------------*/
void qNorm(FLT q[4])
{
    FLT fnorm;

    fnorm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (fnorm > 0.001F)
    {
        fnorm = 1.0F / fnorm;
        q[0] *= fnorm;
        q[1] *= fnorm;
        q[2] *= fnorm;
        q[3] *= fnorm;
    }
    else
    {
        // return with identity quaternion since the quaternion is corrupted
        q[0] = 1.0F;
        q[1] = 0.0F;
        q[2] = 0.0F;
        q[3] = 0.0F;
    }

    // correct a negative scalar component if the function was called with negative q0
    if (q[0] < 0.0F)
    {
        q[0] = -q[0];
        q[1] = -q[1];
        q[2] = -q[2];
        q[3] = -q[3];
    }


}

/*-------------------------------------------------------------------------*/
/**
  @brief    
  @param    
  @return   
  

 */
/*--------------------------------------------------------------------------*/
void f3x3matrixTranspose(FLT matrix[3][3])
{
    U32 i = 0;
    U32 j = 0;
    FLT temp;

    for (i = 1; i < 3; i++)
    {
        for (j = 0; j < i; j++)
        {
            temp = matrix[i][j];
            matrix[i][j] = matrix[j][i];
            matrix[j][i] = temp;
        }
    }
}

/*-------------------------------------------------------------------------*/
/**
  @brief    
  @param    
  @return   
  

 */
/*--------------------------------------------------------------------------*/
void f3x3matrixEqI(FLT matrix[3][3])
{
    U32 i = 0;
    U32 j = 0;

    for (i = 0; i < 3; i++)
    {
        for (j = 0; j < 3; j++)
        {
            matrix[i][j] = 0.0F;
        }
        matrix[i][i] = 1.0F;
    }
}

/*-------------------------------------------------------------------------*/
/**
  @brief    
  @param    
  @return   
  

 */
/*--------------------------------------------------------------------------*/
void f3x3matrixEqScalar(FLT matrix[3][3], FLT scalar)
{
    U32 i = 0;
    U32 j = 0;

    for (i = 0; i < 3; i++)
    {
        for (j = 0; j < 3; j++)
        {
            matrix[i][j] = scalar;
        }
    }
}

