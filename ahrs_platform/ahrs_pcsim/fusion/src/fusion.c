#include <stdio.h>
#include <memory.h>
#include <math.h>
#include "fusion.h"
#include "kalman.h"
#include "misc.h"

#define     ALIGN_NUM       (100)

enum
{
    Initial = 0,
    Aligment = 1,
    Attitude = 2,
    Navigation = 3,
};

typedef struct fusionFixData
{
    U32   uTime;                    // time tag (ms)
    FLT   fPsiPl;					// yaw (deg)
    FLT   fThePl;					// pitch (deg)
    FLT   fPhiPl;					// roll (deg)    
    FLT   fCnb[3][3];               // a posteriori orientation matrix (Cnb)
    FLT   fCbn[3][3];               // a posteriori orientation matrix (Cbn)
    quaternion_t fqPl;              // a posteriori orientation quaternion
    FLT   fGyroBias[CHN];           // gyro bias (rad/s)
    FLT   fAccBias[CHN];            // acc bias (m/s2)
    FLT   fV[3];                      // current hard iron offset x, y, z, (uT)
    FLT   finvW[3][3];              // current inverse soft iron matrix
    FLT   fB;                       // current geomagnetic field magnitude (uT)
    FLT   fFitError;                // value of fit error (%)
    FLT   fVelN;                    // velocity toward the North (m/s)
    FLT   fVelE;                    // velocity toward the East (m/s)
    FLT   fVelD;                    // velocity toward the Down (m/s)
    FLT   fPosN;                    // position in the North (m)
    FLT   fPosE;                    // position in the East (m)
    FLT   fPosD;                    // position in the Down (m)
} fusionFixData_t;

typedef struct fusionFixCtrl
{
    U32   uStaticFlag;              // indicate the device is static now
    U32   uAlignFlag;               // indicate alignment between body frame and navigation frame is completed
    U32   uMagCaliFlag;             // indicate mag calibration process is completed
    U32   uMechanizationFlag;       // indicate INS strapdown mechanization process
} fusionFixCtrl_t;

static fusionFixData_t FusionFix;
static fusionFixCtrl_t FusionCtrl;
static kalmanInfo_t KalmanInfo;
static FLT AlignGyroArray[ALIGN_NUM][CHN];
static FLT AlignAccArray[ALIGN_NUM][CHN];

const DBL INIT_RMS[] = {SIG_PHI_E, SIG_PHI_N, SIG_PHI_U, SIG_GYRO, SIG_GYRO, SIG_GYRO, SIG_ACC, SIG_ACC, SIG_ACC};


static U32 sensorDataCorrection(FLT gyro[], FLT acc[], FLT mag[], const fusionInputData_t* const pinputData, const fusionFixData_t* const pfusionFix, const fusionFixCtrl_t* const pfuisonCtrl);
static U32 staticDectect(const FLT gyro[], FLT gyroArray[][CHN], const FLT acc[], FLT accArray[][CHN]);
static U32 quaternionIntegration(U32 utime, const FLT gyro[], fusionFixData_t* const pfusionFix);
static void ouputResult(const fusionFixData_t* const pfusionFix, fusionOutputData_t* const poutputData);

/*-------------------------------------------------------------------------*/
/**
  @brief    
  @param    
  @return   
  

 */
/*--------------------------------------------------------------------------*/
static U32 sensorDataCorrection(FLT gyro[], FLT acc[], FLT mag[], const fusionInputData_t * const pinputData, const fusionFixData_t * const pfusionFix, const fusionFixCtrl_t * const pfuisonCtrl)
{
    U32 i = 0;

    // gyro data correction
    for (i = X; i <= Z; i++)
    {
        gyro[i] = pinputData->fGyro[i] - pfusionFix->fGyroBias[i];
    }

    // acc data correction
    for (i = X; i <= Z; i++)
    {
        acc[i] = pinputData->fAcc[i] - pfusionFix->fAccBias[i];
    }

    // mag data correction
    if (pfuisonCtrl->uMagCaliFlag != 0)
    {
        FLT ftemp[CHN] = {0};

        // remove the computed hard iron offsets (uT): ftmp[] = fmag[] - fV[]
        for (i = X; i <= Z; i++)
        {
            ftemp[i] = pinputData->fMag[i] - pfusionFix->fV[i];
        }
        // remove the computed soft iron offsets (uT): fmag = inv(W)*(fmag[] - fV[])
        for (i = X; i <= Z; i++)
        {
            mag[i] = pfusionFix->finvW[i][X]*ftemp[X] + pfusionFix->finvW[i][Y]*ftemp[Y] + pfusionFix->finvW[i][Z]*ftemp[Z];
        }
    }

    return 0;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    
  @param    
  @return   
  

 */
/*--------------------------------------------------------------------------*/
#define     ACC_STATIC      (0.1)
#define     GYRO_STATIC     (0.01)
static U32 staticDectect(const FLT gyro[], FLT gyroArray[][CHN], const FLT acc[], FLT accArray[][CHN])
{
    FLT gyro_mean = 0;
    FLT gyro_std = 0;
    FLT acc_mean = 0;
    FLT acc_std = 0;
    U32 i = 0;
    U32 j = 0;
    static U32 uCount = 0;

    if (uCount < ALIGN_NUM)
    {
        for (i = X; i <= Z; i++)
        {
            gyroArray[uCount][i] = gyro[i];
            accArray[uCount][i] = acc[i];
        }
    }
    else
    {
        for(i = 0; i < uCount - 1; i++)
        {
            for (j = X; j <= Z; j++)
            {
                gyroArray[i][j] = gyroArray[i+1][j];
                accArray[i][j] = accArray[i+1][j];
            }
        }
        for (i = X; i <= Z; i++)
        {
            gyroArray[uCount-1][i] = gyro[i];
            accArray[uCount-1][i] = acc[i];
        }
    }

    uCount++;
    if (uCount >= ALIGN_NUM)
    {
        uCount = ALIGN_NUM;
        if (computeMeanStd(&gyro_mean, &gyro_std, gyroArray, uCount))
        {
            return -1;
        }

        if (computeMeanStd(&acc_mean, &acc_std, accArray, uCount))
        {
            return -1;
        }

        if (acc_std < ACC_STATIC && gyro_std < GYRO_STATIC)
        {
            // indicate static condition
            return 1;
        }
    }

    return 0;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    
  @param    
  @return   
  

 */
/*--------------------------------------------------------------------------*/
static U32 accTiltInit(const FLT accArray[][CHN], U32 count, fusionFixData_t *pfusionFix)
{
    U32 i = 0;
    FLT fyaw = 0;
    FLT fpitch = 0;
    FLT froll = 0;
    FLT fmodGxyz;			// modulus of the x, y, z accelerometer readings
    FLT fmodGyz;			// modulus of the y, z accelerometer readings
    FLT frecipmodGxyz;	    // reciprocal of modulus
    FLT ftmp;
    FLT fg[CHN] = {0};
    FLT fq[4] = {0};
    FLT cnb[3][3] = {0};
    FLT cbn[3][3] = {0};

    for (i = 0; i < count; i++)
    {
        fg[X] -= accArray[i][X];
        fg[Y] -= accArray[i][Y];
        fg[Z] -= accArray[i][Z];
    }
    for (i = X; i <= Z; i++)
    {
        fg[i] = fg[i] / count;
    }
    
    //fyaw = 0.0F;
    //fpitch = -asinf(fg[X]/GRAVITY);
    //froll = atan2f(fg[Y]/GRAVITY, fg[Z]/GRAVITY);

    //euler2q(fq, fyaw, fpitch, froll);
    //q2dcm(fq, cbn);
    
    // compute the accelerometer squared magnitudes
    fmodGyz = fg[Y] * fg[Y] + fg[Z] * fg[Z];
    fmodGxyz = fmodGyz + fg[X] * fg[X];

    // check for free fall special case where no solution is possible
    if (fmodGxyz == 0.0F)
    {
        f3x3matrixEqI(pfusionFix->fCnb);
        f3x3matrixEqI(pfusionFix->fCbn);

        return -1;
    }

    // check for vertical up or down gimbal lock case
    if (fmodGyz == 0.0F)
    {
        f3x3matrixEqScalar(pfusionFix->fCnb, 0.0F);
        pfusionFix->fCnb[Y][Y] = 1.0F;
        if (fg[X] >= 0.0F)
        {
            pfusionFix->fCnb[X][Z] = 1.0F;
            pfusionFix->fCnb[Z][X] = -1.0F;
        }
        else
        {
            pfusionFix->fCnb[X][Z] = -1.0F;
            pfusionFix->fCnb[Z][X] = 1.0F;
        }
        memcpy(pfusionFix->fCbn, pfusionFix->fCnb, sizeof(pfusionFix->fCnb));
        f3x3matrixTranspose(pfusionFix->fCbn);

        return -1;
    }

    // compute moduli for the general case
    fmodGyz = sqrtf(fmodGyz);
    fmodGxyz = sqrtf(fmodGxyz);
    frecipmodGxyz = 1.0F / fmodGxyz;
    ftmp = fmodGxyz / fmodGyz;

    // normalize the accelerometer reading into the z column
    for (i = X; i <= Z; i++)
    {
        cnb[i][Z] = fg[i] * frecipmodGxyz;
    }

    // construct x column of orientation matrix
    cnb[X][X] = fmodGyz * frecipmodGxyz;
    cnb[Y][X] = -cnb[X][Z] * cnb[Y][Z] * ftmp;
    cnb[Z][X] = -cnb[X][Z] * cnb[Z][Z] * ftmp;

    // // construct y column of orientation matrix
    cnb[X][Y] = 0.0F;
    cnb[Y][Y] = cnb[Z][Z] * ftmp;
    cnb[Z][Y] = -cnb[Y][Z] * ftmp;

    memcpy(cbn, cnb, sizeof(cnb));
    memcpy(pfusionFix->fCnb, cnb, sizeof(cnb));
    f3x3matrixTranspose(cbn);
    memcpy(pfusionFix->fCbn, cbn, sizeof(cbn));

    dcm2euler(cbn, &fyaw, &fpitch, &froll);
    pfusionFix->fPsiPl = fyaw;
    pfusionFix->fThePl = fpitch;
    pfusionFix->fPhiPl = froll;

    euler2q(fq, fyaw, fpitch, froll);
    pfusionFix->fqPl.q0 = fq[0];
    pfusionFix->fqPl.q1 = fq[1];
    pfusionFix->fqPl.q2 = fq[2];
    pfusionFix->fqPl.q3 = fq[3];

    return 0;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    
  @param    
  @return   
  

 */
/*--------------------------------------------------------------------------*/
static U32 quaternionIntegration(U32 utime, const FLT gyro[], fusionFixData_t * const pfusionFix)
{
    //I: inertial frame
    //E: earth frame (ECEF)
    //P: platform frame (geographic frame)
    //B: body frame
    //eg. EPP means E->P in P
    FLT omegaEPP[CHN] = {0};
    FLT omegaIEP[CHN] = {0};
    FLT omegaIPP[CHN] = {0};
    FLT omegaIPB[CHN] = {0};
    FLT omegaPBB[CHN] = {0};
    FLT fCnb[3][3] = {0};
    FLT fq[4] = {0};
    FLT fdq[4] = {0};
    FLT fdt = 0;
    U32 i = 0;
    
    //no latitude information in computing latitude and longitude rate
    omegaEPP[X] = 0.0F;
    omegaEPP[Y] = 0.0F;
    omegaEPP[Z] = 0.0F;

    //no latitude information in computing earth rate in the navigation frame
    omegaIEP[X] = 0.0F;
    omegaIEP[Y] = 0.0F;
    omegaIEP[Z] = 0.0F;

    for (i = X; i <= Z; i++)
    {
        omegaIPP[i] = omegaIEP[i] + omegaEPP[i];
    }

    memcpy(fCnb, pfusionFix->fCnb, sizeof(fCnb));
    for (i = X; i <= Z; i++)
    {
        omegaIPB[i] = fCnb[i][X]*omegaIPP[X] + fCnb[i][Y]*omegaIPP[Y] + fCnb[i][Z]*omegaIPP[Z];
    }

    for (i = X; i <= Z; i++)
    {
        omegaPBB[i] = gyro[i] - omegaIPB[i];
    }

    fq[0] = pfusionFix->fqPl.q0;
    fq[1] = pfusionFix->fqPl.q1;
    fq[2] = pfusionFix->fqPl.q2;
    fq[3] = pfusionFix->fqPl.q3;

    fdq[0] = -(omegaPBB[0] * fq[1] + omegaPBB[1] * fq[2] + omegaPBB[2] * fq[3]) / 2.0F;
    fdq[1] =  (omegaPBB[0] * fq[0] + omegaPBB[2] * fq[2] - omegaPBB[1] * fq[3]) / 2.0F;
    fdq[2] =  (omegaPBB[1] * fq[0] - omegaPBB[2] * fq[1] + omegaPBB[0] * fq[3]) / 2.0F;
    fdq[3] =  (omegaPBB[2] * fq[0] + omegaPBB[1] * fq[1] - omegaPBB[0] * fq[2]) / 2.0F;

    //fdt = (utime - pfusionFix->uTime) / 1.0F;
    fdt = 1.0F / 100.0F; //100Hz
    for (i = 0; i < 4; i++)
    {
        fq[i] += fdq[i] * fdt;
    }
    qNorm(fq);

    //restore integration result
    pfusionFix->fqPl.q0 = fq[0];
    pfusionFix->fqPl.q1 = fq[1];
    pfusionFix->fqPl.q2 = fq[2];
    pfusionFix->fqPl.q3 = fq[3];
    q2dcm(fq, pfusionFix->fCbn);
    dcm2euler(pfusionFix->fCbn, &pfusionFix->fPsiPl, &pfusionFix->fThePl, &pfusionFix->fPhiPl);
    memcpy(pfusionFix->fCnb, pfusionFix->fCbn, sizeof(pfusionFix->fCnb));
    f3x3matrixTranspose(pfusionFix->fCnb);

    return 0;
}

static void ouputResult(const fusionFixData_t* const pfusionFix, fusionOutputData_t* const poutputData)
{
    poutputData->uTime = pfusionFix->uTime;
    poutputData->fPsiPl = pfusionFix->fPsiPl;
    poutputData->fThePl = pfusionFix->fThePl;
    poutputData->fPhiPl = pfusionFix->fPhiPl;
    memcpy(poutputData->fRPl, pfusionFix->fCnb, sizeof(poutputData->fRPl));
    memcpy(&poutputData->fqPl, &pfusionFix->fqPl, sizeof(poutputData->fqPl));
    poutputData->fVelN = pfusionFix->fVelN;
    poutputData->fVelE = pfusionFix->fVelE;
    poutputData->fVelD = pfusionFix->fVelD;
    poutputData->fPosN = pfusionFix->fPosN;
    poutputData->fPosE = pfusionFix->fPosE;
    poutputData->fPosD = pfusionFix->fPosD;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    
  @param    
  @return   
  

 */
/*--------------------------------------------------------------------------*/
U32 sensorFusionInit(void)
{
    if (kalmanInit(&KalmanInfo, STATE_NUM))
    {
        printf("kalman init failed!\r\n");
        return -1;
    }

    if (UDInit(KalmanInfo.pUd, KalmanInfo.uUdNum, INIT_RMS, KalmanInfo.uStateNum))
    {
        printf("UD init failed!\r\n");
        return -1;
    }

    memset(&FusionFix, 0, sizeof(fusionFixData_t));
    memset(&FusionCtrl, 0, sizeof(fusionFixCtrl_t));

    return 0;
};

/*-------------------------------------------------------------------------*/
/**
  @brief    
  @param    
  @return   
  

 */
/*--------------------------------------------------------------------------*/
U32 sensorFusionExec(const fusionInputData_t* const pinputData, fusionOutputData_t* const poutputData)
{
    U32 retvel = Initial;
    U32 utime = 0;
    U32 i = 0;
    FLT fgyro[CHN] = {0};
    FLT facc[CHN] = {0};
    FLT fmag[CHN] = {0};

    utime = pinputData->uTime;

    sensorDataCorrection(fgyro, facc, fmag, pinputData, &FusionFix, &FusionCtrl);

    if (FusionCtrl.uAlignFlag == 0)
    {
        // static detection
        FusionCtrl.uStaticFlag = staticDectect(fgyro, AlignGyroArray, facc, AlignAccArray);

        if (FusionCtrl.uStaticFlag == 1)
        {
            // start initial alignment
            if (!accTiltInit(AlignAccArray, ALIGN_NUM, &FusionFix))
            {
                retvel = Aligment;
                FusionCtrl.uAlignFlag = 1;
            }
        }

        return retvel;
    }

    retvel = Attitude;
    // quaternion integration
    quaternionIntegration(utime, fgyro, &FusionFix);

    // restore time tag for next epoch
    FusionFix.uTime = utime;
    // output result
    ouputResult(&FusionFix, poutputData);

    return retvel;
};


