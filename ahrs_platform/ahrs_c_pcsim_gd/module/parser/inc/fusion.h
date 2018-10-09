//
// Created by jiangtianyu on 2018/10/8.
//

#ifndef AHRS_C_PCSIM_GD_FUSION_H
#define AHRS_C_PCSIM_GD_FUSION_H

#include <string>
#include <vector>
#include "training.h"
#include "sample.h"
#include "Eigen/Dense"

using namespace std;
using namespace Eigen;

#define MAG_SUPPORT     1
#define CHX             0
#define CHY             1
#define CHZ             2

class SensorFusion
{
    private:
        int    uTime;
        double fPsiPl;
        double fThePl;
        double fPhiPl;
        double fPsiPlPlat;
        double fThePlPlat;
        double fPhiPlPlat;
        double fCnb[3][3];
        double fCbn[3][3];
        double fCnp[3][3];
        double fqPl[4];
        double fqPlPlat[4];
        double fCbnPlat[3][3];
        double fGyroBias[3];
        double fAccBias[3];
        double fMagBias[3];
        double fLinerAccN;
        double fLinerAccE;
        double fLinerAccD;
        double fVelN;
        double fVelE;
        double fVelD;
        double fPosN;
        double fPosE;
        double fPosD;
        double fOmegaB;
        double fAccelerate;
        double fMagnetic;
        double fAudio;

        int uStaticFlag;
        bool uAlignFlag;
        bool uKalmanFusionFlag;
        bool uMechanizationFlag;
        bool uActionStartFlag;
        bool uActionEndFlag;

        const int ALIGN_NUM;
        int CalibrationProgress;
        vector<double*> fAlignGyroArray;
        vector<double*> fAlignAccArray;
        vector<double*> fAlignMagArray;
        vector<SampleData> cSampleDataArray;

        enum
        {
            Calibration = 0,
            Alignment,
            Fusion,
        } iStatus;

        enum
        {
            Peace = 0,
            Step1,
            Step2,
            Step3
        } iCurveCondition;

        double fLinerAccXLast;
        double actionTime;
        double downTime;
        double peakValue;

        double fPlatformOmegaMaxZ;
        double fPlatformOmegaMinZ;
        double fRangeMax;
        double fVelocityMax;
        double fAudioMax;
        int strikeIndex;

        string sAttitude;

    public:
        bool uActionComplete;
        const double GRAVITY;
        const double SAMPLE_RATE;
        TrainData trainData;

        static void euler2q(double q[], double fyaw, double fpitch, double froll);
        static void euler2dcm(double cbn[][3], double fyaw, double fpitch, double froll);

        SensorFusion();
};

#endif //AHRS_C_PCSIM_GD_FUSION_H
