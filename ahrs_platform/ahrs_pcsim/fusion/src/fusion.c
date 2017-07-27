#include <stdio.h>
#include <memory.h>
#include <math.h>
#include "fusion.h"
#include "kalman.h"
#include "misc.h"
#include "magcal.h"

#define     ALIGN_NUM               (100)

#define     STATE_NUM               (9)
#define     UD_NUM                  (STATE_NUM*(STATE_NUM+1)/2)
#define     MEAS_ACC_NUM            (3)
#define     MEAS_MAG_NUM            (3)

#define     SIG_PHI_E               (1.0*PI/180)                /* rms of pitch and roll */
#define     SIG_PHI_N               (1.0*PI/180)                /* rms of pitch and roll */
#define     SIG_PHI_U               (1.0*PI/180)                /* (rad)0.001 rms of heading */
#define     SIG_ACC                 (0.3)                       /* rms of acc error(m/(s.s)) */
#define     SIG_GYRO                (1000.0*DEG2RAD/3600.0)     /* rms of gyro error  */

#define     GYRO_TIME_CONSTANT      (100.0F)
#define     ACC_TIME_CONSTANT       (100.0F)
#define     SIGMA_WIN               ((FLT)1.0e-6)
#define     SIGMA_ACC               ((FLT)((5.0e-4) * 9.78032667 * (5.0e-4) * 9.78032667))
#define     SIGMA_GYRO              ((FLT)(20.0 * PI / 180.0 / 3600 * 20.0 * PI / 180.0 / 3600))

typedef enum sensorFusionStatus
{
    Initial = 0,
    Aligment = 1,
    Attitude = 2,
    Navigation = 3,
} sensorFusionStatus_t;

typedef enum kalmanFilterStatus
{
    OK = 0,
    InnovationCheckFailed = 1,
} kalmanFilterStatus_t;

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
    FLT   fV[3];                    // current hard iron offset x, y, z, (uT)
    FLT   finvW[3][3];              // current inverse soft iron matrix
    FLT   fB;                       // current geomagnetic field magnitude (uT)
    FLT   fFitError;                // value of fit error (%)
    FLT   fDelta;                   // inclination angle (rad)
    FLT   fLinerAccN;               // liner accelerate toward North (m/s2)
    FLT   fLinerAccE;               // liner accelerate toward East (m/s2)
    FLT   fLinerAccD;               // liner accelerate toward Down (m/s2)
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
	U32   uKalmanFusionFlag;		// indicate kalman fusion can be executed
    U32   uMechanizationFlag;       // indicate INS strapdown mechanization process
    U32   uActionStartFlag;         // indicate high dynamic action start
    U32   uActionEndFlag;           // indicate high dynamic action end
} fusionFixCtrl_t;

static fusionFixData_t FusionFix;
static fusionFixCtrl_t FusionCtrl;
static kalmanInfo_t KalmanInfo;
static FLT AlignGyroArray[ALIGN_NUM][CHN];
static FLT AlignAccArray[ALIGN_NUM][CHN];
static FLT AlignMagArray[ALIGN_NUM][CHN];
static magneticBuffer_t MagBuffer;
static magCalibration_t MagCalibration;

const DBL INIT_RMS[] = {SIG_PHI_E, SIG_PHI_N, SIG_PHI_U, SIG_GYRO, SIG_GYRO, SIG_GYRO, SIG_ACC, SIG_ACC, SIG_ACC};

static U32 sensorDataCorrection(FLT gyro[], FLT acc[], FLT mag[], const fusionInputData_t * const pinputData, const fusionFixData_t * const pfusionFix, const fusionFixCtrl_t * const pfuisonCtrl);
static U32 staticDectect(const FLT gyro[], FLT gyroArray[][CHN], const FLT acc[], FLT accArray[][CHN], const FLT mag[], FLT magArray[][CHN]);
static U32 accTiltInit(const FLT accArray[][CHN], const U32 count, fusionFixData_t* const pfusionFix);
static U32 compassAlignment(const FLT accArray[][CHN], const FLT magArray[][CHN], const U32 count, fusionFixData_t *pfusionFix);
static U32 quaternionIntegration(U32 utime, const FLT gyro[], fusionFixData_t * const pfusionFix);
static U32 quaternionIntegration(U32 utime, const FLT gyro[], fusionFixData_t * const pfusionFix);
static void ouputResult(const fusionFixData_t* const pfusionFix, fusionOutputData_t* const poutputData);
static FLT dtCalculate(U32 timeNow, U32 timeLast);
static U32 actionDetect(U32 utime, FLT gyro[], FLT acc[], fusionFixCtrl_t* const pfusionCtrl);
static U32 resetInsData(fusionFixData_t* const pfusionFix);
static void setPhimQd(U32 utime, kalmanInfo_t* const pkalmanInfo, fusionFixData_t* const pfusionFix);
static U32 accMeasUpdate(const FLT acc[], kalmanInfo_t* const pkalmanInfo, fusionFixData_t* const pfusionFix);
static U32 magMeasUpdate(const FLT mag[], kalmanInfo_t* const pkalmanInfo, fusionFixData_t* const pfusionFix);
static void errCorrection(kalmanInfo_t* const pkalmanInfo, fusionFixData_t* const pfusionFix);
static kalmanFilterStatus_t sensorFusionKalman(U32 utime, const FLT acc[], const FLT mag[], kalmanInfo_t* const pkalmanInfo, fusionFixData_t* const pfusionFix);
static void insStrapdownMechanization(U32 utime, const FLT acc[], fusionFixData_t* const pfusionFix);

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
    FLT ftemp[CHN] = {0};

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
static U32 staticDectect(const FLT gyro[], FLT gyroArray[][CHN], const FLT acc[], FLT accArray[][CHN], const FLT mag[], FLT magArray[][CHN])
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
            magArray[uCount][i] = mag[i];
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
                magArray[i][j] = magArray[i+1][j];
            }
        }
        for (i = X; i <= Z; i++)
        {
            gyroArray[uCount-1][i] = gyro[i];
            accArray[uCount-1][i] = acc[i];
            magArray[uCount-1][i] = mag[i];
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
static U32 accTiltInit(const FLT accArray[][CHN], const U32 count, fusionFixData_t* const pfusionFix)
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

    // construct y column of orientation matrix
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
static U32 compassAlignment(const FLT accArray[][CHN], const FLT magArray[][CHN], const U32 count, fusionFixData_t *pfusionFix)
{
    FLT fg[CHN] = {0.0F};
    FLT fmag[CHN] = {0.0F};
    FLT fR[3][3] = {0.0F};
    FLT fmod[3] = {0.0F};
    FLT ftmp = 0.0F;
    FLT fmodMag = 0.0F;
    FLT fGdotMag = 0.0F;
    FLT fyaw = 0.0F;
    FLT fpitch = 0.0F;
    FLT froll = 0.0F;
    FLT fq[4] = {0.0F};
    U32 i, j, retval;

    retval = 0;

    for (i = 0; i < count; i++)
    {
        fg[X] -= accArray[i][X];
        fg[Y] -= accArray[i][Y];
        fg[Z] -= accArray[i][Z];
        fmag[X] += magArray[i][X];
        fmag[Y] += magArray[i][Y];
        fmag[Z] += magArray[i][Z];
    }
    for (i = X; i <= Z; i++)
    {
        fg[i] = fg[i] / count;
        fmag[i] = fmag[i] / count;
        fR[i][Z] = fg[i];
        fR[i][X] = fmag[i];
    }

    // set y vector to vector product of z and x vectors
    fR[X][Y] = fR[Y][Z] * fR[Z][X] - fR[Z][Z] * fR[Y][X];
    fR[Y][Y] = fR[Z][Z] * fR[X][X] - fR[X][Z] * fR[Z][X];
    fR[Z][Y] = fR[X][Z] * fR[Y][X] - fR[Y][Z] * fR[X][X];

    // set x vector to vector product of y and z vectors
    fR[X][X] = fR[Y][Y] * fR[Z][Z] - fR[Z][Y] * fR[Y][Z];
    fR[Y][X] = fR[Z][Y] * fR[X][Z] - fR[X][Y] * fR[Z][Z];
    fR[Z][X] = fR[X][Y] * fR[Y][Z] - fR[Y][Y] * fR[X][Z];

    for (i = X; i <= Z; i++)
    {
        fmod[i] = sqrtf(fR[X][i] * fR[X][i] + fR[Y][i] * fR[Y][i] + fR[Z][i] * fR[Z][i]);
    }

    if (!((fmod[X] == 0.0F) || (fmod[Y] == 0.0F) || (fmod[Z] == 0.0F)))
    {
        for (j = X; j <= Z; j++)
        {
            ftmp = 1.0F / fmod[j];
            for (i = X; i <= Z; i++)
            {
                fR[i][j] *= ftmp;
            }
        }
    }
    else
    {
        // no solution is possible so set rotation to identity matrix
        f3x3matrixEqI(fR);
        retval = -1;
    }
    memcpy(pfusionFix->fCnb, fR, sizeof(pfusionFix->fCnb));
    memcpy(pfusionFix->fCbn, fR, sizeof(pfusionFix->fCbn));
    f3x3matrixTranspose(pfusionFix->fCbn);

    fmodMag = sqrtf(fmag[X] * fmag[X] + fmag[Y] * fmag[Y] + fmag[Z] * fmag[Z]);
    fGdotMag = fg[X] * fmag[X] + fg[Y] * fmag[Y] + fg[Z] * fmag[Z];
    if (!((fmod[Z] == 0.0F) || (fmodMag == 0.0F)))
    {
        pfusionFix->fDelta = asinf(fGdotMag / (fmod[Z] * fmodMag));
    }
    dcm2euler(pfusionFix->fCbn, &fyaw, &fpitch, &froll);
    pfusionFix->fPsiPl = fyaw;
    pfusionFix->fThePl = fpitch;
    pfusionFix->fPhiPl = froll;

    euler2q(fq, fyaw, fpitch, froll);
    pfusionFix->fqPl.q0 = fq[0];
    pfusionFix->fqPl.q1 = fq[1];
    pfusionFix->fqPl.q2 = fq[2];
    pfusionFix->fqPl.q3 = fq[3];

    // only for test mag measure update in kalman.
    // magnetic strength update in mag calibration process.
    // this code need to be removed.
    pfusionFix->fB = fmodMag;

    return retval;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    
  @param    
  @return   
  

 */
/*--------------------------------------------------------------------------*/
static void gyroCalibration(const FLT gyroArray[][CHN], const U32 count, fusionFixData_t *pfusionFix)
{
    FLT fgyro[CHN] = {0.0F};
    U32 i = 0;
    
    for (i = 0; i < count; i++)
    {
        fgyro[X] += gyroArray[i][X];
        fgyro[Y] += gyroArray[i][Y];
        fgyro[Z] += gyroArray[i][Z];
    }
    pfusionFix->fGyroBias[X] += (FLT)(fgyro[X] * 1.0 / count);
    pfusionFix->fGyroBias[Y] += (FLT)(fgyro[Y] * 1.0 / count);
    pfusionFix->fGyroBias[Z] += (FLT)(fgyro[Z] * 1.0 / count);
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

    fdt = dtCalculate(utime, pfusionFix->uTime);
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

/*-------------------------------------------------------------------------*/
/**
  @brief    
  @param    
  @return   
  

 */
/*--------------------------------------------------------------------------*/
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
static FLT dtCalculate(U32 timeNow, U32 timeLast)
{
    //if (timeLast > timeNow)
    //{
    //    return (0xFFFFFFFF - timeLast + timeNow) / 1000.0F;
    //}
    //else
    //{
    //    return (timeNow - timeLast) / 1000.0F;
    //}

    return 1.0F / 100.0F;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    
  @param    
  @return   
  

 */
/*--------------------------------------------------------------------------*/
#define     ACC_MAX     (8*GRAVITY)
#define     ACC_START   (0.9)
#define     ACC_END     (0.2)
static U32 actionDetect(U32 utime, FLT gyro[], FLT acc[], fusionFixCtrl_t* const pfusionCtrl)
{
    FLT acc_det = 0.0F;

    acc_det = sqrtf(acc[X]*acc[X] + acc[Y]*acc[Y] + acc[Z]*acc[Z]);

    if (acc_det > ACC_START*ACC_MAX)
    {
        pfusionCtrl->uActionStartFlag = 1;
    }

    if (pfusionCtrl->uActionStartFlag == 1 && acc_det < ACC_END * ACC_MAX)
    {
        pfusionCtrl->uActionEndFlag = 1;
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
static U32 resetInsData(fusionFixData_t* const pfusionFix)
{
    pfusionFix->fLinerAccN = 0;
    pfusionFix->fLinerAccE = 0;
    pfusionFix->fLinerAccD = 0;
    pfusionFix->fVelN = 0;
    pfusionFix->fVelE = 0;
    pfusionFix->fVelD = 0;
    pfusionFix->fPosN = 0;
    pfusionFix->fPosE = 0;
    pfusionFix->fPosD = 0;

    return 0;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    
  @param    
  @return   
  

 */
/*--------------------------------------------------------------------------*/
static void setPhimQd(U32 utime, kalmanInfo_t* const pkalmanInfo, fusionFixData_t* const pfusionFix)
{
    U32 i = 0;
    U32 j = 0;
    U32 stateNum = pkalmanInfo->uStateNum;
    DBL **phim = pkalmanInfo->pPhim;
    DBL **qdt = pkalmanInfo->pQd;
    FLT (*fCbn)[3] = pfusionFix->fCbn;
    DBL G[STATE_NUM][STATE_NUM] = {0};         // the row and col of shaping matrix are related with model rather than fixed.
    DBL GT[STATE_NUM][STATE_NUM] = {0};        // the transpose of G matrix
    DBL M2[STATE_NUM][STATE_NUM] = {0};
    DBL temp[STATE_NUM][STATE_NUM] = {0};
    DBL *pRowA[STATE_NUM] = {0};
    DBL *pRowB[STATE_NUM] = {0};
    FLT fdt = 0.0F;

    for (i = 0; i <stateNum; i++)
    {
        for (j = 0; j < stateNum; j++)
        {
            phim[i][j] = 0.0F;
            qdt[i][j] = 0.0F;
        }
    }

    //set PHI matrix
    phim[0][3] = (DBL) -fCbn[0][0];
    phim[0][4] = (DBL) -fCbn[0][1];
    phim[0][5] = (DBL) -fCbn[0][2];

    phim[1][3] = (DBL) -fCbn[1][0];
    phim[1][4] = (DBL) -fCbn[1][1];
    phim[1][5] = (DBL) -fCbn[1][2];

    phim[2][3] = (DBL) -fCbn[2][0];
    phim[2][4] = (DBL) -fCbn[2][1];
    phim[2][5] = (DBL) -fCbn[2][2];

    phim[3][3] = (DBL) - 1.0F / GYRO_TIME_CONSTANT;
    phim[4][4] = (DBL) - 1.0F / GYRO_TIME_CONSTANT;
    phim[5][5] = (DBL) - 1.0F / GYRO_TIME_CONSTANT;

    phim[6][6] = (DBL) - 1.0F / ACC_TIME_CONSTANT;
    phim[7][7] = (DBL) - 1.0F / ACC_TIME_CONSTANT;
    phim[8][8] = (DBL) - 1.0F / ACC_TIME_CONSTANT;

    //set Q matrix
    qdt[0][0] = (DBL)SIGMA_WIN;
    qdt[1][1] = (DBL)SIGMA_WIN;
    qdt[2][2] = (DBL)SIGMA_WIN;

    qdt[3][3] = (DBL)SIGMA_GYRO;
    qdt[4][4] = (DBL)SIGMA_GYRO;
    qdt[5][5] = (DBL)SIGMA_GYRO;

    qdt[6][6] = (DBL)SIGMA_ACC;
    qdt[7][7] = (DBL)SIGMA_ACC;
    qdt[8][8] = (DBL)SIGMA_ACC;

    // set G matrix
    G[0][0] = (DBL) -fCbn[0][0];
    G[0][1] = (DBL) -fCbn[0][1];
    G[0][2] = (DBL) -fCbn[0][2];

    G[1][0] = (DBL) -fCbn[1][0];
    G[1][1] = (DBL) -fCbn[1][1];
    G[1][2] = (DBL) -fCbn[1][2];

    G[2][0] = (DBL) -fCbn[2][0];
    G[2][1] = (DBL) -fCbn[2][1];
    G[2][2] = (DBL) -fCbn[2][2];

    for (i = 3; i < STATE_NUM; i++)
    {
        G[i][i] = 1.0;
    }

    // qdt = G*w*G'
    for (i = 0; i < stateNum; i++)
    {
        pRowA[i] = G[i];
    }
    for (i = 0; i < stateNum; i++)
    {
        pRowB[i] = temp[i];
    }
    matrixMult(pRowA, qdt, stateNum, stateNum, stateNum, stateNum, pRowB);
    for (i = 0; i < stateNum; i++)
    {
        pRowA[i] = GT[i];
    }
    matrixMult(pRowB, pRowA, stateNum, stateNum, stateNum, stateNum, qdt);

    // Q matrix discretization-2 order
    // M2=phi¡ÁM1£¬M1£½Q
    for (i = 0; i < stateNum; i++)
    {
        pRowA[i] = M2[i];
    }
    matrixMult(phim, qdt, stateNum, stateNum, stateNum, stateNum, pRowA);

    fdt = dtCalculate(utime, pfusionFix->uTime);
    for (i = 0; i < stateNum; i++)
    {
        for (j = 0; j < stateNum; j++)

        {
            qdt[i][j] = qdt[i][j] * fdt + (M2[i][j] + M2[j][i]) * fdt * fdt / 2.0;
        }
    }
    
    // ud decompose for Q matrix
    udDecompose(qdt, stateNum);

    // phi matrix discretization-2 order
    for (i = 0; i < stateNum; i++)
    {
        pRowA[i] = temp[i];
    }
    matrixMult(phim, phim, stateNum, stateNum, stateNum, stateNum, pRowA);

    for (i = 0; i < stateNum; i++)
    {
        for (j = 0; j < stateNum; j++)
        {
            phim[i][j] = phim[i][j] * fdt + temp[i][j] * fdt * fdt / 2.0; //second order phi matrix

            if (j == i)
            {
                phim[i][j] += 1.0;
            }
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
static U32 accMeasUpdate(const FLT acc[], kalmanInfo_t* const pkalmanInfo, fusionFixData_t* const pfusionFix)
{
    U32 i = 0;
    U32 j = 0;
    DBL zc = 0.0;
    DBL rc = 0.0;
    DBL hc[STATE_NUM] = {0.0};
    DBL z[MEAS_ACC_NUM] = {0.0};
    DBL h[MEAS_ACC_NUM][STATE_NUM] = {0.0};
    DBL r[MEAS_ACC_NUM] = {0.0};
    DBL xSave[STATE_NUM];
    DBL udSave[UD_NUM];
    DBL ion = 0.0;
    DBL res = 0.0;
    DBL gEstimate[3] = {0.0};
    DBL test = 0.0;

    h[0][1] = GRAVITY;
    h[1][0] = -GRAVITY;
    h[0][6] = pfusionFix->fCbn[0][0];
    h[0][7] = pfusionFix->fCbn[0][1];
    h[0][8] = pfusionFix->fCbn[0][2];
    h[1][6] = pfusionFix->fCbn[1][0];
    h[1][7] = pfusionFix->fCbn[1][1];
    h[1][8] = pfusionFix->fCbn[1][2];
    h[2][6] = pfusionFix->fCbn[2][0];
    h[2][7] = pfusionFix->fCbn[2][1];
    h[2][8] = pfusionFix->fCbn[2][2];

    r[0] = 0.5 * 0.5;
    r[1] = 0.5 * 0.5;
    r[2] = 0.5 * 0.5;

    for (i = X; i <= Z; i++)
    {
        gEstimate[i] = -(pfusionFix->fCbn[i][X] * acc[X] + pfusionFix->fCbn[i][Y] * acc[Y] + pfusionFix->fCbn[i][Z] * acc[Z]);
    }

    z[X] = 0 - gEstimate[X];
    z[Y] = 0 - gEstimate[Y];
    z[Z] = GRAVITY - gEstimate[Z];

    for (i = 0; i < MEAS_ACC_NUM; i++)
    {
        zc = z[i];
        rc = r[i];

        for (j = 0; j < STATE_NUM; j++)
        {
            hc[j] = h[i][j];
        }

        // save x,p in case the measurement is rejected
        memcpy(xSave, pkalmanInfo->pStateX, sizeof(xSave));
        memcpy(udSave, pkalmanInfo->pUd, sizeof(udSave));

        // scalar measurement update
        udMeasUpdate(pkalmanInfo->pUd, pkalmanInfo->pStateX, pkalmanInfo->uStateNum, rc, hc, zc, &ion, &res);
        test = fabs(res) / sqrt(ion);

        // reject this measurement
        // 1. innovation test > 5, generally it is around 3.24
        if (test > 5)
        {
            memcpy(pkalmanInfo->pStateX, xSave, sizeof(xSave));
            memcpy(pkalmanInfo->pUd, udSave, sizeof(udSave));
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
static U32 magMeasUpdate(const FLT mag[], kalmanInfo_t* const pkalmanInfo, fusionFixData_t* const pfusionFix)
{
    U32 i = 0;
    U32 j = 0;
    DBL zc = 0.0;
    DBL rc = 0.0;
    DBL hc[STATE_NUM] = {0.0};
    DBL z[MEAS_MAG_NUM] = {0.0};
    DBL h[MEAS_MAG_NUM][STATE_NUM] = {0.0};
    DBL r[MEAS_MAG_NUM] = {0.0};
    DBL xSave[STATE_NUM];
    DBL udSave[UD_NUM];
    DBL ion = 0.0;
    DBL res = 0.0;
    DBL magEstimate[3] = {0.0};
    DBL magVector[3] = {0.0};
    DBL test = 0.0;

    magVector[0] = pfusionFix->fB * cosf(pfusionFix->fDelta);
    magVector[2] = pfusionFix->fB * sinf(pfusionFix->fDelta);
    h[0][1] = magVector[2];
    h[1][0] = -magVector[2];
    h[1][2] = magVector[0];
    h[2][1] = -magVector[0];

    r[0] = 1 * 1;
    r[1] = 1 * 1;
    r[2] = 1 * 1;

    for (i = X; i <= Z; i++)
    {
        magEstimate[i] = pfusionFix->fCbn[i][X] * mag[X] + pfusionFix->fCbn[i][Y] * mag[Y] + pfusionFix->fCbn[i][Z] * mag[Z];
    }

    z[X] = magVector[X] - magEstimate[X];
    z[Y] = magVector[Y] - magEstimate[Y];
    z[Z] = magVector[Z] - magEstimate[Z];

    for (i = 0; i < MEAS_MAG_NUM; i++)
    {
        zc = z[i];
        rc = r[i];

        for (j = 0; j < STATE_NUM; j++)
        {
            hc[j] = h[i][j];
        }

        // save x,p in case the measurement is rejected
        memcpy(xSave, pkalmanInfo->pStateX, sizeof(xSave));
        memcpy(udSave, pkalmanInfo->pUd, sizeof(udSave));

        // scalar measurement update
        udMeasUpdate(pkalmanInfo->pUd, pkalmanInfo->pStateX, pkalmanInfo->uStateNum, rc, hc, zc, &ion, &res);
        test = fabs(res) / sqrt(ion);

        // reject this measurement
        // 1. innovation test > 5, generally it is around 3.24
        if (test > 5)
        {
            memcpy(pkalmanInfo->pStateX, xSave, sizeof(xSave));
            memcpy(pkalmanInfo->pUd, udSave, sizeof(udSave));
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
static void errCorrection(kalmanInfo_t* const pkalmanInfo, fusionFixData_t* const pfusionFix)
{
    U32 i = 0;
    U32 j = 0;
    U32 k = 0;
    FLT fq[4] = {0.0};
    FLT deltaCbn[3][3] = {0.0};
    FLT temp = 0.0;
    FLT tempMatrix[3][3] = {0.0};

    euler2dcm(deltaCbn, (FLT)pkalmanInfo->pStateX[2], (FLT)pkalmanInfo->pStateX[1], (FLT)pkalmanInfo->pStateX[0]);
    for (i = X; i <= Z; i++)
    {
        for (j = X; j <= Z; j++)
        {
            temp = 0.0F;
            for (k = X; k <= Z; k++)
            {
                temp += deltaCbn[i][k] * pfusionFix->fCbn[k][j];
            }
            tempMatrix[i][j] = temp;
        }
    }
    memcpy(pfusionFix->fCbn, tempMatrix, sizeof(pfusionFix->fCbn));
    memcpy(pfusionFix->fCnb, tempMatrix, sizeof(pfusionFix->fCnb));
    f3x3matrixTranspose(pfusionFix->fCnb);

    for (i = X; i <= Z; i++)
    {
        pfusionFix->fGyroBias[i] += (FLT)pkalmanInfo->pStateX[i + 3];
        pfusionFix->fAccBias[i] += (FLT)pkalmanInfo->pStateX[i + 6];
    }

    dcm2euler(pfusionFix->fCbn, &pfusionFix->fPsiPl, &pfusionFix->fThePl, &pfusionFix->fPhiPl);
    euler2q(fq, pfusionFix->fPsiPl, pfusionFix->fThePl, pfusionFix->fPhiPl);
    qNorm(fq);
    pfusionFix->fqPl.q0 = fq[0];
    pfusionFix->fqPl.q1 = fq[1];
    pfusionFix->fqPl.q2 = fq[2];
    pfusionFix->fqPl.q3 = fq[3];

    // clear x
    for (i = 0; i < STATE_NUM; i++)
    {
        pkalmanInfo->pStateX[i] = 0.0;
    }
}

/*-------------------------------------------------------------------------*/
/**
  @brief    
  @param    
  @return   
  

 */
/*--------------------------------------------------------------------------*/
static kalmanFilterStatus_t sensorFusionKalman(U32 utime, const FLT acc[], const FLT mag[], kalmanInfo_t* const pkalmanInfo, fusionFixData_t* const pfusionFix)
{
    setPhimQd(utime, pkalmanInfo, pfusionFix);
    predict(pkalmanInfo);
    accMeasUpdate(acc, pkalmanInfo, pfusionFix);
    magMeasUpdate(mag, pkalmanInfo, pfusionFix);
    errCorrection(pkalmanInfo, pfusionFix);

    return OK;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    
  @param    
  @return   
  

 */
/*--------------------------------------------------------------------------*/
static void insStrapdownMechanization(U32 utime, const FLT acc[], fusionFixData_t* const pfusionFix)
{
    U32 i = 0;
    FLT dt = 0.0;
    DBL linerAccIBP[CHN] = {0.0};
    DBL velIBP[CHN] = {0.0};
    DBL linerAccAve[CHN] = {0.0};
    DBL velAve[CHN] = {0.0};

    dt = dtCalculate(utime, pfusionFix->uTime);
    
    for (i = X; i <= Z; i++)
    {
        linerAccIBP[i] = acc[X]*pfusionFix->fCbn[i][X] + acc[Y]*pfusionFix->fCbn[i][Y] + acc[Z]*pfusionFix->fCbn[i][Z];
    }
    linerAccIBP[Z] += GRAVITY;

    linerAccAve[X] = (linerAccIBP[X] + pfusionFix->fLinerAccN) / 2.0;
    linerAccAve[Y] = (linerAccIBP[Y] + pfusionFix->fLinerAccE) / 2.0;
    linerAccAve[Z] = (linerAccIBP[Z] + pfusionFix->fLinerAccD) / 2.0;
    velIBP[X] = pfusionFix->fVelN + linerAccAve[X] * dt;
    velIBP[Y] = pfusionFix->fVelE + linerAccAve[Y] * dt;
    velIBP[Z] = pfusionFix->fVelD + linerAccAve[Z] * dt;
    velAve[X] = pfusionFix->fVelN + linerAccAve[X] * dt / 2.0;
    velAve[Y] = pfusionFix->fVelE + linerAccAve[Y] * dt / 2.0;
    velAve[Z] = pfusionFix->fVelD + linerAccAve[Z] * dt / 2.0;

    pfusionFix->fLinerAccN = (FLT)linerAccIBP[X];
    pfusionFix->fLinerAccE = (FLT)linerAccIBP[Y];
    pfusionFix->fLinerAccD = (FLT)linerAccIBP[Z];
    pfusionFix->fVelN = (FLT)velIBP[X];
    pfusionFix->fVelE = (FLT)velIBP[Y];
    pfusionFix->fVelD = (FLT)velIBP[Z];
    pfusionFix->fPosN += (FLT)velAve[X] * dt;
    pfusionFix->fPosE += (FLT)velAve[Y] * dt;
    pfusionFix->fPosD += (FLT)velAve[Z] * dt;

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
    if (magCalibrationInit(&MagCalibration, &MagBuffer))
    {
        printf("mag calibration init failed!\r\n");
        return -1;
    }

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
    FusionCtrl.uKalmanFusionFlag = 1;

    f3x3matrixEqI(FusionFix.finvW);
    f3x3matrixEqI(FusionFix.fCbn);
    f3x3matrixEqI(FusionFix.fCnb);

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
    U32 retval = Initial;
    U32 utime = 0;
    U32 i = 0;
    FLT fgyro[CHN] = {0};
    FLT facc[CHN] = {0};
    FLT fmag[CHN] = {0};
    static U32 LoopCounter = 0;

    utime = pinputData->uTime;

    sensorDataCorrection(fgyro, facc, fmag, pinputData, &FusionFix, &FusionCtrl);

    magBufferUpdate(&MagBuffer, pinputData->fMag, fmag, LoopCounter);
    LoopCounter++;
    magCalibrationExec(&MagCalibration, &MagBuffer);
    if (MagCalibration.iValidMagCal != 0)
    {
        // indicate mag calibration is valid
    }

    // static detection
    FusionCtrl.uStaticFlag = staticDectect(fgyro, AlignGyroArray, facc, AlignAccArray, fmag, AlignMagArray);

    if (FusionCtrl.uAlignFlag == 0)
    {
        if (FusionCtrl.uStaticFlag == 1)
        {
            // start initial alignment
            if (!compassAlignment(AlignAccArray, AlignMagArray, ALIGN_NUM, &FusionFix))
            {
                retval = Aligment;
                FusionCtrl.uAlignFlag = 1;
            }
        }

        return retval;
    }

    retval = Attitude;

    if (FusionCtrl.uStaticFlag == 1)
    {
        // gyro bias calibration
        //gyroCalibration(AlignGyroArray, ALIGN_NUM, &FusionFix);

        // delta angle update
    }

    // quaternion integration
    quaternionIntegration(utime, fgyro, &FusionFix);

    // action detect for fusion and ins mechanization
    //actionDetect(utime, fgyro, facc, &FusionCtrl);
    if (FusionCtrl.uActionStartFlag == 1)
    {
        FusionCtrl.uMechanizationFlag = 1;
        FusionCtrl.uKalmanFusionFlag = 0;
    }
    else
    {
        FusionCtrl.uKalmanFusionFlag = 1;
    }

    if (FusionCtrl.uActionEndFlag == 1)
    {
        FusionCtrl.uMechanizationFlag = 0;
        FusionCtrl.uActionStartFlag = 0;
        FusionCtrl.uActionEndFlag = 0;
        // clear INS data
        resetInsData(&FusionFix);
    }

	// kalman filter fusion
	if (FusionCtrl.uKalmanFusionFlag == 1)
    {
        kalmanFilterStatus_t status = OK;

        status = sensorFusionKalman(utime, facc, fmag, &KalmanInfo, &FusionFix);

        if (status != OK)
        {
            // kalman filter failed
        }
    }

    // ins mechanization

    if (FusionCtrl.uMechanizationFlag == 1)
    {
        insStrapdownMechanization(utime, facc, &FusionFix);
    }

    // restore time tag for next epoch
    FusionFix.uTime = utime;
    // output result
    ouputResult(&FusionFix, poutputData);

    return retval;
};

