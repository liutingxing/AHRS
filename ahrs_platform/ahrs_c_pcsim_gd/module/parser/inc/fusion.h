//
// Created by jiangtianyu on 2018/10/8.
//

#ifndef AHRS_C_PCSIM_GD_FUSION_H
#define AHRS_C_PCSIM_GD_FUSION_H

#include <string>
#include <vector>
#include <memory>
#include "training.h"
#include "sample.h"
#include "Eigen/Dense"

using namespace std;
using namespace Eigen;

#define MAG_SUPPORT     1
#define CHX             0
#define CHY             1
#define CHZ             2

#ifndef     PI
#define     PI              ((float)3.14159265358979323846)
#endif

#ifndef     DEG2RAD
#define     DEG2RAD         ((float)PI/(float)180.0)
#endif

#ifndef     RAD2DEG
#define     RAD2DEG         ((float)180.0/(float)PI)
#endif

#define     MAGSENSITIVE    (0.01F)
#define     MAGBUFFSIZEX    (14)
#define     MAGBUFFSIZEY    (MAGBUFFSIZEX * 2)
#define     MAXMEASUREMENTS (240)
#define     MESHDELTAUT     (5)
#define     MAGFITERROR     (5)

class SensorFusion
{
    private:
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
        double fOmegaB[3];
        double fAccelerate[3];
        double fMagnetic[3];
        double fAudio;

        double fB;
        double ftrB;
        double ftrFitErrorpc;
        double fFitErrorpc;
        bool iValidMagCal;
        double ftrV[3];
        double fV[3];

        int uStaticFlag;
        bool uAlignFlag;
        bool uKalmanFusionFlag;
        bool uMechanizationFlag;
        bool uActionStartFlag;
        bool uActionEndFlag;

        const int ALIGN_NUM;
        int CalibrationProgress;
        vector<shared_ptr<double>> fAlignGyroArray;
        vector<shared_ptr<double>> fAlignAccArray;
        vector<shared_ptr<double>> fAlignMagArray;
        vector<shared_ptr<SampleData>> cSampleDataArray;

        float fuTPerCount;
        float fCountsPeruT;
        int iMagRawBuffer[3][MAGBUFFSIZEX][MAGBUFFSIZEY];
        int Index[MAGBUFFSIZEX][MAGBUFFSIZEY];
        int Tanarray[MAGBUFFSIZEX - 1];
        int iMagBufferCount;

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

        void magCalibrationInit();
        int magBufferUpdate(double magRaw[], double magCal[], int loopCounter);
        int magCalibrationExec();
        void calibration4INV();
        void sensorDataCorrection(double gyro[], double acc[], double mag[]);
        int staticDetectUpdate(double gyro[], double acc[], double mag[]);
        int staticDetectCheck();
        double stdCal(vector<shared_ptr<double>>& numList);
        bool sensorAlignment(vector<shared_ptr<double>>& accArray, vector<shared_ptr<double>>& magArray);
        void gyroCalibration(vector<shared_ptr<double>>& gyroArray);
        void ahrsProcess(double dt, double gyro[], double acc[], double mag[]);
        void quaternionIntegration(double dt, double gyro[]);
        void platformDataProcess();
        void actionDetect(double dt, double gyro[], double acc[]);
        int copyInSampleData(SensorFusion *src, SampleData *dst);
        void systemConditionSet();
        void insStrapdownMechanization(double dt, double acc[]);
        int processSampleData(vector<shared_ptr<SampleData>>& sampleDataArray, PingPongTrainData& data);
        int updateAudioInfo(PingPongTrainData& data);

    public:
        int uTime;
        bool uActionComplete;
        const double GRAVITY;
        const double SAMPLE_RATE;
        PingPongTrainData trainData;

        static void euler2q(double q[], double fyaw, double fpitch, double froll);
        static void euler2dcm(double cbn[][3], double fyaw, double fpitch, double froll);
        static void dcm2euler(double cbn[][3], double euler[]);
        static void qNorm(double fq[]);
        static void q2dcm(double q[], double cbn[][3]);

        SensorFusion();
        string sensorFusionExec(int time, double gyro[], double acc[], double mag[], double audio);
        int resetSensorFusion();
        int getCalibrationProgress() {return CalibrationProgress;};
};

#endif //AHRS_C_PCSIM_GD_FUSION_H
