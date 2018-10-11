//
// Created by jiangtianyu on 2018/10/8.
//

#include <cmath>
#include "fusion.h"
#include "Eigen/Dense"

SensorFusion::SensorFusion(): ALIGN_NUM(100), GRAVITY(9.80665), SAMPLE_RATE(100)
{
    Matrix3d temp;

    uTime = 0;
    fPsiPl = 0;
    fThePl = 0;
    fPhiPl = 0;
    fPsiPlPlat = 0;
    fThePlPlat = 0;
    fPhiPlPlat = 0;
    euler2q(fqPl, fPsiPl, fThePl, fPhiPl);
    euler2dcm(fCbn, fPsiPl, fThePl, fPhiPl);
    temp << fCbn[0][0], fCbn[0][1], fCbn[0][2],
         fCbn[1][0], fCbn[1][1], fCbn[1][2],
         fCbn[2][0], fCbn[2][1], fCbn[2][2];
    temp.transposeInPlace();

    for (int i = CHX; i <= CHZ; i++)
    {
        for (int j = CHX; j <= CHZ; j++)
        {
            fCnb[i][j] = temp(i, j);
            fCbnPlat[i][j] = 0;
        }
    }

    memset(fqPl, 0, sizeof(fqPl));
    memset(fqPlPlat, 0, sizeof(fqPlPlat));
    memset(fGyroBias, 0, sizeof(fGyroBias));
    memset(fAccBias, 0, sizeof(fAccBias));
    memset(fMagBias, 0, sizeof(fMagBias));
    fLinerAccN = 0;
    fLinerAccE = 0;
    fLinerAccD = 0;
    fVelN = 0;
    fVelE = 0;
    fVelD = 0;
    fPosN = 0;
    fPosE = 0;
    fPosD = 0;
    uStaticFlag = -1;
    uAlignFlag = false;
    uKalmanFusionFlag = true;
    uMechanizationFlag = false;
    uActionStartFlag = false;
    uActionEndFlag = false;
    uActionComplete = false;
    sAttitude = "";
    iStatus = Calibration;
    iCurveCondition = Peace;
    CalibrationProgress = 0;
}

string SensorFusion::sensorFusionExec(int time, double gyro[], double acc[], double mag[], double audio)
{
    double dt = 1.0 / SAMPLE_RATE;
    double fGyroRaw[3];
    double fAccRaw[3];
    double fMagRaw[3];

    uTime = time;

    for (int i = CHX; i <= CHZ; i++)
    {
        fGyroRaw[i] = fOmegaB[i] = gyro[i];
        fAccRaw[i] = fAccelerate[i] = acc[i];
        fMagRaw[i] = fMagnetic[i] = mag[i];
    }

    fAudio = audio;

    if (uActionComplete == true)
    {
        uActionComplete = false;
        fLinerAccXLast = 0;
        fPlatformOmegaMaxZ = 0;
        fPlatformOmegaMinZ = 0;
        fRangeMax = 0.0;
        fVelocityMax = 0.0;
        fAudioMax = 0.0;
        strikeIndex = 0;

        cSampleDataArray.clear();
    }

    // data correction
    sensorDataCorrection(gyro, acc, mag);
    // static detection
    uStaticFlag = staticDetect(fGyroRaw, fAccRaw, fMagRaw);

}

int SensorFusion::staticDetect(double gyro[], double acc[], double mag[])
{
    double gyro_std = 0;
    double acc_std = 0;

    if (fAlignGyroArray.size() >= ALIGN_NUM)
    {
        fAlignAccArray.erase(fAlignAccArray.begin());
        fAlignGyroArray.erase(fAlignGyroArray.begin());
    }

    fAlignAccArray.push_back(shared_ptr<double[]> (new double[3] {acc[0], acc[1], acc[2]}, [](double *p) {delete[] p;}));
    fAlignGyroArray.push_back(shared_ptr<double[]> (new double[3] {gyro[0], gyro[1], gyro[2]}, [](double *p) {delete[] p;}));
#if MAG_SUPPORT
    fAlignMagArray.push_back(shared_ptr<double[]> (new double[3] {mag[0], mag[1], mag[2]}, [](double *p) {delete[] p;}));
#endif

    if (fAlignGyroArray.size() == ALIGN_NUM && fAlignAccArray.size() == ALIGN_NUM)
    {
        gyro_std = stdCal(fAlignGyroArray);
        acc_std = stdCal(fAlignAccArray);

        if (gyro_std < 0.01 && acc_std < 0.1)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }

    return -1;
}

double SensorFusion::stdCal(vector<shared_ptr<double[]>> numList)
{
    double det = 0;
    double value = 0; // the sum of Xi
    double square = 0; // the sum of xi^2
    double sigma = 0; // standard deviation
    double size = numList.size();

    for (auto val : numList)
    {
        double *p = val.get();
        det = sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
        value += det;
        square += det * det;
    }

    sigma = sqrt((square - value * value / size) / (size - 1));

    return sigma;
}

void SensorFusion::sensorDataCorrection(double gyro[], double acc[], double mag[])
{
    for (int i = CHX; i <= CHZ; i++)
    {
        gyro[i] = gyro[i] - fGyroBias[i];
        acc[i] = acc[i] - fAccBias[i];
#if MAG_SUPPORT

        if (mag[0] != 0 && mag[1] != 0 && mag[2] != 0)
        {
            mag[i] = mag[i] - fMagBias[i];
        }

#endif
    }
}

void SensorFusion::euler2q(double q[], double fyaw, double fpitch, double froll)
{
    q[0] = cos(froll / 2) * cos(fpitch / 2) * cos(fyaw / 2) + sin(froll / 2) * sin(fpitch / 2) * sin(fyaw / 2);
    q[1] = sin(froll / 2) * cos(fpitch / 2) * cos(fyaw / 2) - cos(froll / 2) * sin(fpitch / 2) * sin(fyaw / 2);
    q[2] = cos(froll / 2) * sin(fpitch / 2) * cos(fyaw / 2) + sin(froll / 2) * cos(fpitch / 2) * sin(fyaw / 2);
    q[3] = cos(froll / 2) * cos(fpitch / 2) * sin(fyaw / 2) - sin(froll / 2) * sin(fpitch / 2) * cos(fyaw / 2);
}

void SensorFusion::euler2dcm(double cbn[][3], double fyaw, double fpitch, double froll)
{
    cbn[0][0] = cos(fpitch) * cos(fyaw);
    cbn[0][1] = sin(froll) * sin(fpitch) * cos(fyaw) - cos(froll) * sin(fyaw);
    cbn[0][2] = cos(froll) * sin(fpitch) * cos(fyaw) + sin(froll) * sin(fyaw);

    cbn[1][0] = cos(fpitch) * sin(fyaw);
    cbn[1][1] = sin(froll) * sin(fpitch) * sin(fyaw) + cos(froll) * cos(fyaw);
    cbn[1][2] = cos(froll) * sin(fpitch) * sin(fyaw) - sin(froll) * cos(fyaw);

    cbn[2][0] = -sin(fpitch);
    cbn[2][1] = sin(froll) * cos(fpitch);
    cbn[2][2] = cos(froll) * cos(fpitch);
}

