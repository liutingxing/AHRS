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
    euler2q(fqPl, fPsiPl, fThePl, fPhiPl);
    euler2dcm(fCbn, fPsiPl, fThePl, fPhiPl);
    temp << fCbn[0][0], fCbn[0][1], fCbn[0][2],
         fCbn[1][0], fCbn[1][1], fCbn[1][2],
         fCbn[2][0], fCbn[2][1], fCbn[2][2];
    temp.transposeInPlace();

    for (int i = CHX; i <= CHZ; i++)
        for (int j = CHX; j <= CHZ; j++)
        {
            fCnb[i][j] = temp(i, j);
        }
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
    CalibrationProgress = 0;
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