//
// Created by jiangtianyu on 2018/10/8.
//

#include <cmath>
#include <algorithm>
#include "fusion.h"
#include "fitting.h"
#include "Eigen/Dense"

SensorFusion::SensorFusion(): ALIGN_NUM(100), GRAVITY(9.80665), SAMPLE_RATE(100), CALIBRATION_NUM(500), dt(1.0 / SAMPLE_RATE)
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
    magCalibrationInit();
}

int SensorFusion::resetSensorFusion()
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
    magCalibrationInit();
    fCalibrationMagArray.clear();
    fAlignGyroArray.clear();
    fAlignAccArray.clear();
    fAlignMagArray.clear();

    return 0;
}

string SensorFusion::sensorFusionExec(int time, double gyro[], double acc[], double mag[], double audio)
{
    double dt = 1.0 / SAMPLE_RATE;
    double fGyroRaw[3];
    double fAccRaw[3];
    double fMagRaw[3];

    uTime = time;
    // recording the raw data
    for (int i = CHX; i <= CHZ; i++)
    {
        fOmegaBRaw[i] = fGyroRaw[i] = gyro[i];
        fAccelerateRaw[i] = fAccRaw[i] = acc[i];
        fMagneticRaw[i] = fMagRaw[i] = mag[i];
    }

    if (uActionComplete == true)
    {
        uActionComplete = false;
        cSampleDataArray.clear();
    }

    // data filter
    gyroFilter(gyro); // 4Hz cutoff frequency
    accFilter(acc);   // 4Hz cutoff frequency

    // data correction
    sensorDataCorrection(gyro, acc, mag);

    // recording the calibrated data
    for (int i = CHX; i <= CHZ; i++)
    {
        fOmegaB[i] = gyro[i];
        fAccelerate[i] = acc[i];
        fMagnetic[i] = mag[i];
    }

    fAudio = audio;

    // static detection
    staticDetectUpdate(fGyroRaw, fAccRaw, fMagRaw);

    if (time % 100 == 0) // 1Hz static check frequency
    {
        uStaticFlag = staticDetectCheck();

        if (uStaticFlag == 1)
        {
            gyroCalibration(fAlignGyroArray);
        }
    }

    // mag buffer update
#if MAG_SUPPORT

    if (uStaticFlag == 0 && iStatus == Calibration)
    {
        if (magCalibration(mag) == true)
        {
            // mag calibration process complete
            if (fGeoB > 10 && fGeoB < 100 && fResidual < 10)
            {
                iStatus = Alignment;
                CalibrationProgress = 100;
            }
        }
        else
        {
            // mag calibration process not execute
            CalibrationProgress = (int)(fCalibrationMagArray.size() * 95 / CALIBRATION_NUM);
        }
    }

#else
    iStatus = Alignment;
#endif

    if (uAlignFlag == false)
    {
        if (uStaticFlag == 1 && iStatus == Alignment)
        {
            // initial alignment
            if (sensorAlignment(fAlignAccArray, fAlignMagArray) == true)
            {
                uAlignFlag = true;
                iStatus = Fusion;
            }
        }

        return "";
    }

#if MAG_SUPPORT

    if (uStaticFlag == 0)
    {
        magBufferUpdate(fMagRaw, mag, time);
    }

    if (time % 100 == 0) // 1Hz calibration frequency
    {
        magCalibrationExec();
    }

#endif

    // AHRS/INS process
    sAttitude.clear();

    // quaternion integration for attitude and heading
    if (true)
    {
        ahrsProcess(dt, gyro, acc, mag);
    }
    else
    {
        quaternionIntegration(dt, gyro);
    }

    // convert the ahrs data for MAG_SUPPORT == 0 and MAG_SUPPORT == 1
    platformDataProcess();

    // action detect
    actionDetect(dt, gyro, acc);

    // system condition change
    systemConditionSet();

    if (uMechanizationFlag == true)
    {
        shared_ptr<SampleData> pSampleData = make_shared<SampleData>();

        // copy sample data into array list
        copyInSampleData(this, pSampleData.get());
        cSampleDataArray.push_back(pSampleData);
    }

    // refine the sample data array
    if (uActionComplete == true)
    {
        outlierCompensate(cSampleDataArray, gyro);
        refineSampleData(cSampleDataArray);
    }

    // record the attitude and trajectory
    if (uActionComplete == true)
    {
        processSampleData(cSampleDataArray, trainData);

        // filter the invalid action
        if (trainData.fVelocityMax < 1.0 && trainData.fRangeMax < 0.05)
        {
            trainData.bValid = false;
            trainData.uActionCount--;
        }
    }

    sAttitude.append(to_string(uTime));
    sAttitude.append(" ");
    sAttitude.append(to_string(0));
    sAttitude.append(" ");
    sAttitude.append(to_string(0));
    sAttitude.append(" ");
    sAttitude.append(to_string(0));
    sAttitude.append(" ");
    sAttitude.append(to_string(fqPlPlat[0]));
    sAttitude.append(" ");
    sAttitude.append(to_string(-fqPlPlat[1]));
    sAttitude.append(" ");
    sAttitude.append(to_string(-fqPlPlat[2]));
    sAttitude.append(" ");
    sAttitude.append(to_string(fqPlPlat[3]));
    sAttitude.append(" ");
    sAttitude.append("x");

    return sAttitude;
}

int SensorFusion::updateAudioInfo(PingPongTrainData& data)
{

    if (data.uStrikePower > 70)
    {
        data.uAudioType = 2;
    }
    else if (data.fRangeMax > 1.5)
    {
        data.uAudioType = 3;
    }
    else if (data.fVelocityMax > 6)
    {
        data.uAudioType = 1;
    }
    else if ((data.fVelocityMax - data.fVelocityStrike) < data.fVelocityMax * 0.08)
    {
        data.uAudioType = 4;
    }
    else
    {
        data.uAudioType = 0;
    }

    return 0;
}

int SensorFusion::processSampleData(vector<shared_ptr<SampleData>>& sampleDataArray, PingPongTrainData& data)
{

    int count = 0;
    string trajectory;
    SampleData* value;

    fPlatformOmegaMaxZ = 0;
    fPlatformOmegaMinZ = 0;
    fRangeMax = 0.0;
    fVelocityMax = 0.0;
    fAudioMax = 0.0;
    strikeIndex = 0;

    if (sampleDataArray.size() == 0)
    {
        return -1;
    }

    insStrapdownMechanization(dt, sampleDataArray);

    for (auto p : sampleDataArray)
    {
        double deltaN = 0;
        double deltaE = 0;
        double deltaD = 0;
        SampleData* valLast = nullptr;
        SampleData* val = p.get();

        // platform omega
        if (val->fOmegaN[CHZ] > fPlatformOmegaMaxZ)
        {
            fPlatformOmegaMaxZ = val->fOmegaN[CHZ];
        }

        if (val->fOmegaN[CHZ]  < fPlatformOmegaMinZ)
        {
            fPlatformOmegaMinZ = val->fOmegaN[CHZ];
        }

        // max velocity
        if (val->fVel > fVelocityMax)
        {
            fVelocityMax = val->fVel;
        }

        // range
        auto i = find(sampleDataArray.begin(), sampleDataArray.end(), p) - sampleDataArray.begin();

        if (i >= 1)
        {
            valLast = sampleDataArray[i - 1].get();
            deltaN = val->fPosN - valLast->fPosN;
            deltaE = val->fPosE - valLast->fPosE;
            deltaD = val->fPosD - valLast->fPosD;
        }

        fRangeMax += sqrt(deltaN * deltaN + deltaE * deltaE + deltaD * deltaD);

        // max audio for strike timing
        if (val->fAudio > fAudioMax)
        {
            fAudioMax = val->fAudio;
            strikeIndex = int(find(sampleDataArray.begin(), sampleDataArray.end(), p) - sampleDataArray.begin());
        }
    }

    if (fAudioMax < 250) // no ping pong ball training
    {
        fAudioMax = 0;
        strikeIndex = -1;
    }
    else if (fAudioMax > 1000)
    {
        fAudioMax = 1000;
    }

    for (auto p : sampleDataArray)
    {
        SampleData* val = p.get();
        // trajectory
        trajectory.append(to_string(val->uTime));
        trajectory.append(" ");
        trajectory.append(to_string(val->fPosN));
        trajectory.append(" ");
        trajectory.append(to_string(-val->fPosD));
        trajectory.append(" ");
        trajectory.append(to_string(-val->fPosE));
        trajectory.append(" ");
        trajectory.append(to_string(val->fqPlPlat[0]));
        trajectory.append(" ");
        trajectory.append(to_string(-val->fqPlPlat[1]));
        trajectory.append(" ");
        trajectory.append(to_string(-val->fqPlPlat[2]));
        trajectory.append(" ");
        trajectory.append(to_string(val->fqPlPlat[3]));
        trajectory.append(" ");

        if (count == strikeIndex)
        {
            trajectory.append("s");
        }
        else
        {
            trajectory.append("x");
        }

        trajectory.append("\n");

        count++;
    }

    // delete the last enter character
    trajectory.erase(trajectory.end() - 1);

    // update train data
    data.bValid = true;
    data.uActionCount ++;
    data.fRangeMax = fRangeMax;
    data.fVelocityMax = fVelocityMax;
    data.fStrikeAudio = fAudioMax;

    if (strikeIndex != -1)
    {
        value = sampleDataArray[strikeIndex].get();
        data.fVelocityStrike = sqrt(value->fVelN * value->fVelN + value->fVelE * value->fVelE + value->fVelD * value->fVelD);

        if (data.fVelocityStrike > 9)
        {
            data.uStrikePower = 100;
        }
        else if (data.fVelocityStrike > 8)
        {
            data.uStrikePower = (int)((data.fVelocityStrike - 8) * 10.0 + 90);
        }
        else if (data.fVelocityStrike > 7)
        {
            data.uStrikePower = (int)((data.fVelocityStrike - 7) * 5.0 + 85);
        }
        else if (data.fVelocityStrike > 6)
        {
            data.uStrikePower = (int)((data.fVelocityStrike - 6) * 5.0 + 80);
        }
        else if (data.fVelocityStrike > 5)
        {
            data.uStrikePower = (int)((data.fVelocityStrike - 5) * 15.0 + 65);
        }
        else if (data.fVelocityStrike > 4)
        {
            data.uStrikePower = (int)((data.fVelocityStrike - 4) * 25.0 + 40);
        }
        else
        {
            data.uStrikePower = (int)(data.fVelocityStrike * 10);
        }
    }
    else
    {
        data.fVelocityStrike = 0;
        data.uStrikePower = 0;
    }


    if (abs(fPlatformOmegaMaxZ) > abs(fPlatformOmegaMinZ))
    {
        data.sActionType = "backhand";
    }
    else
    {
        data.sActionType = "forehand";
    }

    // replace the type character in trajectory
    while (trajectory.find('x') != string::npos)
    {
        auto typeIndex = trajectory.find('x');

        trajectory.replace(typeIndex, 1, data.sActionType, 0, 1);
    }

    data.sTrajectory = trajectory;

    // update audio index
    updateAudioInfo(data);

    return 0;
}

void SensorFusion::insStrapdownMechanization(double dt, vector<shared_ptr<SampleData>>& sampleDataArray)
{

    int i;
    double linerAccIBP[3] = {0, 0, 0};
    double velIBP[3] = {0, 0, 0};
    double linerAccAve[3] = {0, 0, 0};
    double velAve[3] = {0, 0, 0};
    double deltaN = 0.0;
    double deltaE = 0.0;
    double deltaD = 0.0;
    shared_ptr<SampleData> valLast = make_shared<SampleData>();
    int index = 0;

    for (auto p : sampleDataArray)
    {
        SampleData* val = p.get();

        for (i = 0; i < 3; i++)
        {
            linerAccIBP[i] = val->fAccelerate[0] * val->fCbnPlat[i][0] +
                             val->fAccelerate[1] * val->fCbnPlat[i][1] +
                             val->fAccelerate[2] * val->fCbnPlat[i][2];
        }

        linerAccIBP[2] += GRAVITY;

        // static constrain
        for (i = 0; i < 3; i++)
        {
            if (abs(linerAccIBP[i]) < 1)
            {
                linerAccIBP[i] = 0;
            }
        }

        if (index == 0)
        {
            valLast->fLinerAccN = 0;
            valLast->fLinerAccE = 0;
            valLast->fLinerAccD = 0;
            valLast->fVelN = 0;
            valLast->fVelE = 0;
            valLast->fVelD = 0;
            valLast->fPosN = 0;
            valLast->fPosE = 0;
            valLast->fPosD = 0;
        }
        else
        {
            valLast = sampleDataArray.at(index - 1);
        }

        index ++;

        linerAccAve[0] = (linerAccIBP[0] + valLast->fLinerAccN) / 2.0;
        linerAccAve[1] = (linerAccIBP[1] + valLast->fLinerAccE) / 2.0;
        linerAccAve[2] = (linerAccIBP[2] + valLast->fLinerAccD) / 2.0;
        velIBP[0] = valLast->fVelN + linerAccAve[0] * dt;
        velIBP[1] = valLast->fVelE + linerAccAve[1] * dt;
        velIBP[2] = valLast->fVelD + linerAccAve[2] * dt;
        velAve[0] = (valLast->fVelN + velIBP[0]) / 2.0;
        velAve[1] = (valLast->fVelE + velIBP[1]) / 2.0;
        velAve[2] = (valLast->fVelD + velIBP[2]) / 2.0;

        val->fLinerAccN = linerAccIBP[0];
        val->fLinerAccE = linerAccIBP[1];
        val->fLinerAccD = linerAccIBP[2];
        val->fVelN = velIBP[0];
        val->fVelE = velIBP[1];
        val->fVelD = velIBP[2];
        deltaN = velAve[0] * dt;
        deltaE = velAve[1] * dt;
        deltaD = velAve[2] * dt;
        val->fPosN = valLast->fPosN + deltaN;
        val->fPosE = valLast->fPosE + deltaE;
        val->fPosD = valLast->fPosD + deltaD;
    }
}

void SensorFusion::systemConditionSet()
{

    if (uActionStartFlag == true)
    {
        uMechanizationFlag = true;
        uKalmanFusionFlag = false;
    }
    else
    {
        uMechanizationFlag = false;
        uKalmanFusionFlag = true;

        // clear sample data
        cSampleDataArray.clear();
    }

    if (uActionEndFlag == true)
    {
        uActionStartFlag = false;
        uActionEndFlag = false;
        uActionComplete = true;

        // one action complete, report and reset ins data
        uMechanizationFlag = false;
        uKalmanFusionFlag = true;
    }
}

int SensorFusion::copyInSampleData(SensorFusion* src, SampleData* dst)
{

    dst->uTime = src->uTime;
    dst->fPsiPlPlat = src->fPsiPlPlat;
    dst->fThePlPlat = src->fThePlPlat;
    dst->fPhiPlPlat = src->fPhiPlPlat;

    for (int i = CHX; i <= CHZ; i++)
    {
        for (int j = CHX; j <= CHZ; j++)
        {
            dst->fCbnPlat[i][j] = src->fCbnPlat[i][j];
        }
    }

    for (int i = 0; i < 4; i++)
    {
        dst->fqPlPlat[i] = src->fqPlPlat[i];
    }

    dst->fLinerAccN = src->fAccelerate[0] * src->fCbnPlat[0][0] + src->fAccelerate[1] * src->fCbnPlat[0][1] + src->fAccelerate[2] * src->fCbnPlat[0][2];
    dst->fLinerAccE = src->fAccelerate[0] * src->fCbnPlat[1][0] + src->fAccelerate[1] * src->fCbnPlat[1][1] + src->fAccelerate[2] * src->fCbnPlat[1][2];
    dst->fLinerAccD = src->fAccelerate[0] * src->fCbnPlat[2][0] + src->fAccelerate[1] * src->fCbnPlat[2][1] + src->fAccelerate[2] * src->fCbnPlat[2][2];
    dst->fVelN = src->fVelN;
    dst->fVelE = src->fVelE;
    dst->fVelD = src->fVelD;
    dst->fPosN = src->fPosN;
    dst->fPosE = src->fPosE;
    dst->fPosD = src->fPosD;

    for (int i = CHX; i <= CHZ; i++)
    {
        dst->fOmegaB[i] = src->fOmegaB[i];
        dst->fAccelerate[i] = src->fAccelerate[i];
        dst->fMagnetic[i] = src->fMagnetic[i];
        dst->fOmegaBRaw[i] = src->fOmegaBRaw[i];
        dst->fAccelerateRaw[i] = src->fAccelerateRaw[i];
        dst->fMagneticRaw[i] = src->fMagneticRaw[i];
    }

    dst->fAudio = src->fAudio;

    for (int i = CHX; i <= CHZ; i++)
    {
        dst->fOmegaN[i] = src->fCbnPlat[i][0] * src->fOmegaB[0] + src->fCbnPlat[i][1] * src->fOmegaB[1] + src->fCbnPlat[i][2] * src->fOmegaB[2];
    }

    dst->fVel = sqrt(src->fVelN * src->fVelN +  src->fVelE * src->fVelE + src->fVelD * src->fVelD);

    return 0;
}

void SensorFusion::actionDetect(double dt, double gyro[], double acc[])
{

    int i;
    int slop = 0;
    double linerAccIBP[] = {0, 0, 0};
    double linerAccX = 0.0;

    // calculate the liner accelerate along the x axis
    for (i = 0; i < 3; i++)
    {
        linerAccIBP[i] = acc[0] * fCbnPlat[i][0] + acc[1] * fCbnPlat[i][1] + acc[2] * fCbnPlat[i][2];
    }

    linerAccIBP[2] += GRAVITY;

    // static constrain
    for (i = 0; i < 3; i++)
    {
        if (abs(linerAccIBP[i]) < 1)
        {
            linerAccIBP[i] = 0;
        }
    }

    linerAccX = linerAccIBP[0];

    switch (iCurveCondition)
    {
    case Peace:
        if (linerAccX > 3)
        {
            uActionStartFlag = true;
            iCurveCondition = Step1;
            actionTime = 0;
        }

        break;

    case Step1:
        if (actionTime == 0)
        {
            if (linerAccX < fLinerAccXLast)
            {
                // abnormal case
                iCurveCondition = Peace;
                uActionStartFlag = false;
                break;
            }
        }

        actionTime += dt;

        if (actionTime > 1.0)
        {
            iCurveCondition = Peace;
            uActionStartFlag = false;
            break;
        }

        if (linerAccX > fLinerAccXLast)
        {
            slop = 1;
        }
        else
        {
            slop = -1;

            // reach the up peak
            if (fLinerAccXLast < 6 && abs(gyro[CHZ]) < 10)
            {
                // false peak
                iCurveCondition = Peace;
                uActionStartFlag = false;
            }
            else
            {
                iCurveCondition = Step2;
                downTime = 0;
                peakValue = fLinerAccXLast;
                // maybe is a false peak since the prepare action
            }
        }

        break;

    case Step2:
        actionTime += dt;
        downTime += dt;

        if (actionTime > 1.5 || downTime > 1.0)
        {
            iCurveCondition = Peace;
            uActionStartFlag = false;
            break;
        }

        if (linerAccX > fLinerAccXLast)
        {
            slop = 1;

            // reach the trough
            if (fLinerAccXLast > 0.5 * peakValue)
            {
                // there are 2 kind of false peak:
                // 1. first peak is false peak
                // 2. the following peak is false peak
                // we recording the two kinds of false peak for the following refine
            }
            else if (fLinerAccXLast > -6)
            {
                // false trough
                // no action, because it is normal
            }
            else
            {
                iCurveCondition = Step3;
            }
        }
        else
        {
            slop = -1;
        }

        break;

    case Step3:
        actionTime += dt;

        if (actionTime > 2.0)
        {
            iCurveCondition = Peace;
            uActionStartFlag = false;
            break;
        }

        if (linerAccX > fLinerAccXLast)
        {
            slop = 1;
        }
        else
        {
            slop = -1;
        }

        if (linerAccX > -10 && linerAccX < 10 && abs(gyro[CHZ]) < 10)
        {
            uActionEndFlag = true;
            iCurveCondition = Peace;
        }

        break;
    }

    fLinerAccXLast = linerAccX;
}

void SensorFusion::platformDataProcess()
{
#if MAG_SUPPORT
    Matrix3d cbn;
    Matrix3d cnp;
    Matrix3d cbnPlatform;
    double euler[3];

    cbn << fCbn[0][0], fCbn[0][1], fCbn[0][2],
        fCbn[1][0], fCbn[1][1], fCbn[1][2],
        fCbn[2][0], fCbn[2][1], fCbn[2][2];

    cnp << fCnp[0][0], fCnp[0][1], fCnp[0][2],
        fCnp[1][0], fCnp[1][1], fCnp[1][2],
        fCnp[2][0], fCnp[2][1], fCnp[2][2];

    cbnPlatform = cnp * cbn;

    for (int i = CHX; i <= CHZ; i++)
    {
        for (int j = CHX; j <= CHZ; j++)
        {
            fCbnPlat[i][j] = cbnPlatform(i, j);
        }
    }

    dcm2euler(fCbnPlat, euler);
    fPsiPlPlat = euler[0];
    fThePlPlat = euler[1];
    fPhiPlPlat = euler[2];
    euler2q(fqPlPlat, fPsiPlPlat, fThePlPlat, fPhiPlPlat);
#else

    for (int i = 0; i < 4; i++)
    {
        fqPlPlat[i] = fqPl[i];
    }

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            fCbnPlat[i][j] = fCbn[i][j];
        }
    }

#endif
}

bool SensorFusion::magCalibration(double mag[])
{
    if (mag[2] == 0)
    {
        return false;
    }

    if (fCalibrationMagArray.size() >= CALIBRATION_NUM)
    {
        fCalibrationMagArray.erase(fCalibrationMagArray.begin());
    }

    fCalibrationMagArray.push_back(shared_ptr<double>(new double[3] {mag[0], mag[1], mag[2]}, [](double * p)
    {
        delete[] p;
    }));

    if (fCalibrationMagArray.size() == CALIBRATION_NUM)
    {
        calibration4InvRaw(fCalibrationMagArray);

        return true;
    }

    return false;
}

void SensorFusion::calibration4InvRaw(vector<shared_ptr<double>>& magArray)
{
    // local variables
    double fBs2;                            // fBs[CHX]^2+fBs[CHY]^2+fBs[CHZ]^2
    double fSumBs4;                         // sum of fBs2
    double fscaling;                        // set to FUTPERCOUNT * FMATRIXSCALING
    double fE;                              // error function = r^T.r
    double fOffset[3];                      // offset to remove large DC hard iron bias in matrix
    int iCount;                             // number of measurements counted
    int ierror;                             // matrix inversion error flag
    int i, j, k, l;                         // loop counters

    double ftrB;
    double ftrFitErrorpc;
    double DEFAULTB = 50.0;
    double fvecB[4];
    double fvecA[6];
    double fmatA[4][4];
    double fmatB[4][4];
    double ftrV[3];

    // compute fscaling to reduce multiplications later
    fscaling = 1.0 / DEFAULTB;

    // zero fSumBs4=Y^T.Y, fvecB=X^T.Y (4x1) and on and above diagonal elements of fmatA=X^T*X (4x4)
    fSumBs4 = 0.0;

    for (i = 0; i < 4; i++)
    {
        fvecB[i] = 0.0;

        for (j = i; j < 4; j++)
        {
            fmatA[i][j] = 0.0;
        }
    }

    // the offsets are guaranteed to be set from the first element but to avoid compiler error
    fOffset[0] = fOffset[1] = fOffset[2] = 0;

    // use entries from magnetic buffer to compute matrices
    iCount = 0;

    for (auto val : magArray)
    {
        double* pMag = val.get();

        // use first valid magnetic buffer entry as estimate (in counts) for offset
        if (iCount == 0)
        {
            for (l = 0; l <= 2; l++)
            {
                fOffset[l] = pMag[l];
            }
        }

        // store scaled and offset fBs[XYZ] in fvecA[0-2] and fBs[XYZ]^2 in fvecA[3-5]
        for (l = 0; l <= 2; l++)
        {
            fvecA[l] = (pMag[l] - fOffset[l]) * fscaling;
            fvecA[l + 3] = fvecA[l] * fvecA[l];
        }

        // calculate fBs2 = fBs[CHX]^2 + fBs[CHY]^2 + fBs[CHZ]^2 (scaled uT^2)
        fBs2 = fvecA[3] + fvecA[4] + fvecA[5];
        // accumulate fBs^4 over all measurements into fSumBs4=Y^T.Y
        fSumBs4 += fBs2 * fBs2;

        // now we have fBs2, accumulate fvecB[0-2] = X^T.Y =sum(fBs2.fBs[XYZ])
        for (l = 0; l <= 2; l++)
        {
            fvecB[l] += fvecA[l] * fBs2;
        }

        //accumulate fvecB[3] = X^T.Y =sum(fBs2)
        fvecB[3] += fBs2;
        // accumulate on and above-diagonal terms of fmatA = X^T.X ignoring fmatA[3][3]
        fmatA[0][0] += fvecA[0 + 3];
        fmatA[0][1] += fvecA[0] * fvecA[1];
        fmatA[0][2] += fvecA[0] * fvecA[2];
        fmatA[0][3] += fvecA[0];
        fmatA[1][1] += fvecA[1 + 3];
        fmatA[1][2] += fvecA[1] * fvecA[2];
        fmatA[1][3] += fvecA[1];
        fmatA[2][2] += fvecA[2 + 3];
        fmatA[2][3] += fvecA[2];
        // increment the counter for next iteration
        iCount++;
    }

    // set the last element of the measurement matrix to the number of buffer elements used
    fmatA[3][3] = (double) iCount;

    // use above diagonal elements of symmetric fmatA to set both fmatB and fmatA to X^T.X
    for (i = 0; i < 4; i++)
    {
        for (j = i; j < 4; j++)
        {
            fmatB[i][j] = fmatB[j][i] = fmatA[j][i] = fmatA[i][j];
        }
    }

    // calculate in situ inverse of fmatB = inv(X^T.X) (4x4) while fmatA still holds X^T.X
    Matrix4d temp;
    temp << fmatB[0][0], fmatB[0][1], fmatB[0][2], fmatB[0][3],
         fmatB[1][0], fmatB[1][1], fmatB[1][2], fmatB[1][3],
         fmatB[2][0], fmatB[2][1], fmatB[2][2], fmatB[2][3],
         fmatB[3][0], fmatB[3][1], fmatB[3][2], fmatB[3][3];
    temp = temp.inverse().eval();

    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            fmatB[i][j] = temp(i, j);
        }
    }

    // calculate fvecA = solution beta (4x1) = inv(X^T.X).X^T.Y = fmatB * fvecB
    for (i = 0; i < 4; i++)
    {
        fvecA[i] = 0.0F;

        for (k = 0; k < 4; k++)
        {
            fvecA[i] += fmatB[i][k] * fvecB[k];
        }
    }

    // calculate P = r^T.r = Y^T.Y - 2 * beta^T.(X^T.Y) + beta^T.(X^T.X).beta
    // = fSumBs4 - 2 * fvecA^T.fvecB + fvecA^T.fmatA.fvecA
    // first set P = Y^T.Y - 2 * beta^T.(X^T.Y) = fSumBs4 - 2 * fvecA^T.fvecB
    fE = 0.0F;

    for (i = 0; i < 4; i++)
    {
        fE += fvecA[i] * fvecB[i];
    }

    fE = fSumBs4 - 2.0F * fE;

    // set fvecB = (X^T.X).beta = fmatA.fvecA
    for (i = 0; i < 4; i++)
    {
        fvecB[i] = 0.0F;

        for (k = 0; k < 4; k++)
        {
            fvecB[i] += fmatA[i][k] * fvecA[k];
        }
    }

    // complete calculation of P by adding beta^T.(X^T.X).beta = fvecA^T * fvecB
    for (i = 0; i < 4; i++)
    {
        fE += fvecB[i] * fvecA[i];
    }

    // compute the hard iron vector (in uT but offset and scaled by FMATRIXSCALING)
    for (l = 0; l <= 2; l++)
    {
        ftrV[l] = 0.5F * fvecA[l];
    }

    // compute the scaled geomagnetic field strength B (in uT but scaled by FMATRIXSCALING)
    ftrB = sqrt(fvecA[3] + ftrV[0] * ftrV[0] + ftrV[1] * ftrV[1] + ftrV[2] * ftrV[2]);
    // calculate the trial fit error (percent) normalized to number of measurements and scaled geomagnetic field strength
    ftrFitErrorpc = sqrt(fE / 300) * 100.0F / (2.0F * ftrB * ftrB);

    // correct the hard iron estimate for FMATRIXSCALING and the offsets applied (result in uT)
    for (l = CHX; l <= CHZ; l++)
    {
        ftrV[l] = ftrV[l] * DEFAULTB + fOffset[l];
    }

    // correct the geomagnetic field strength B to correct scaling (result in uT)
    ftrB *= DEFAULTB;

    // assignment
    fGeoB = ftrB;
    fResidual = ftrFitErrorpc;

    for (l = 0; l <= 2; l++)
    {
        fMagBias[l] = ftrV[l];
    }
}

void SensorFusion::quaternionIntegration(double dt, double gyro[])
{
    int i = 0;
    double fdq[4] {0, 0, 0, 0};
    double euler[4];

    fdq[0] = -(gyro[0] * fqPl[1] + gyro[1] * fqPl[2] + gyro[2] * fqPl[3]) / 2.0;
    fdq[1] = (gyro[0] * fqPl[0] + gyro[2] * fqPl[2] - gyro[1] * fqPl[3]) / 2.0;
    fdq[2] = (gyro[1] * fqPl[0] - gyro[2] * fqPl[1] + gyro[0] * fqPl[3]) / 2.0;
    fdq[3] = (gyro[2] * fqPl[0] + gyro[1] * fqPl[1] - gyro[0] * fqPl[2]) / 2.0;

    for (i = 0; i < 4; i++)
    {
        fqPl[i] += fdq[i] * dt;
    }

    qNorm(fqPl);
    q2dcm(fqPl, fCbn);
    dcm2euler(fCbn, euler);
    fPsiPl = euler[0];
    fThePl = euler[1];
    fPhiPl = euler[2];
    Matrix3d temp;
    temp << fCbn[0][0], fCbn[0][1], fCbn[0][2],
         fCbn[1][0], fCbn[1][1], fCbn[1][2],
         fCbn[2][0], fCbn[2][1], fCbn[2][2];
    temp.transposeInPlace();

    for (int i = CHX; i <= CHZ; i++)
    {
        for (int j = CHX; j <= CHZ; j++)
        {
            fCnb[i][j] = temp(i, j);
        }
    }
}

void SensorFusion::ahrsProcess(double dt, double gyro[], double acc[], double mag[])
{
    int i = 0;
    double accNorm = 0;
    double qDot[4] {0, 0, 0, 0};
    double qDotError[4] {0, 0, 0, 0};
    double gyroMeasError = 10 * PI / 180; // gyroscope measurement error in rad/s (shown as 10 deg/s)
    double beta = sqrt(3.0 / 4.0) * gyroMeasError;
    double euler[3];

    qDot[0] = -(gyro[0] * fqPl[1] + gyro[1] * fqPl[2] + gyro[2] * fqPl[3]) / 2.0;
    qDot[1] = (gyro[0] * fqPl[0] + gyro[2] * fqPl[2] - gyro[1] * fqPl[3]) / 2.0;
    qDot[2] = (gyro[1] * fqPl[0] - gyro[2] * fqPl[1] + gyro[0] * fqPl[3]) / 2.0;
    qDot[3] = (gyro[2] * fqPl[0] + gyro[1] * fqPl[1] - gyro[0] * fqPl[2]) / 2.0;

    accNorm = sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);

    if (true)
    {
        // execute the acc aid process
        double diff = 0;
        double gEstimate[3];
        MatrixXd F(3, 1);
        MatrixXd J(3, 4);
        MatrixXd step(4, 1);

        gEstimate[0] = -acc[0] / accNorm;
        gEstimate[1] = -acc[1] / accNorm;
        gEstimate[2] = -acc[2] / accNorm;

        F(0, 0) = 2 * (fqPl[1] * fqPl[3] - fqPl[0] * fqPl[2]) - gEstimate[0];
        F(1, 0) = 2 * (fqPl[0] * fqPl[1] + fqPl[2] * fqPl[3]) - gEstimate[1];
        F(2, 0) = 2 * (0.5 - fqPl[1] * fqPl[1] - fqPl[2] * fqPl[2]) - gEstimate[2];

        J(0, 0) = -2 * fqPl[2];
        J(0, 1) = 2 * fqPl[3];
        J(0, 2) = -2 * fqPl[0];
        J(0, 3) = 2 * fqPl[1];

        J(1, 0) = 2 * fqPl[1];
        J(1, 1) = 2 * fqPl[0];
        J(1, 2) = 2 * fqPl[3];
        J(1, 3) = 2 * fqPl[2];

        J(2, 0) = 0;
        J(2, 1) = -4 * fqPl[1];
        J(2, 2) = -4 * fqPl[2];
        J(2, 3) = 0;

        step = J.transpose().eval() * F;

        qDotError[0] += step(0, 0);
        qDotError[1] += step(1, 0);
        qDotError[2] += step(2, 0);
        qDotError[3] += step(3, 0);
    }

#if MAG_SUPPORT
    double magNorm = sqrt(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);

    if (true)
    {
        // execute the acc aid process
        double diff = 0;
        double mEstimate[3];
        double b[4];
        MatrixXd F(3, 1);
        MatrixXd J(3, 4);
        MatrixXd h(3, 1);
        MatrixXd m(3, 1);
        MatrixXd step(4, 1);
        MatrixXd cbn(3, 3);

        cbn << fCbn[0][0], fCbn[0][1], fCbn[0][2],
            fCbn[1][0], fCbn[1][1], fCbn[1][2],
            fCbn[2][0], fCbn[2][1], fCbn[2][2];

        mEstimate[0] = mag[0] / magNorm;
        mEstimate[1] = mag[1] / magNorm;
        mEstimate[2] = mag[2] / magNorm;

        m(0, 0) = mEstimate[0];
        m(1, 0) = mEstimate[1];
        m(2, 0) = mEstimate[2];

        h = cbn * m;
        b[0] = 0;
        b[1] = sqrt(h(0, 0) * h(0, 0) + h(1, 0) * h(1, 0));
        b[2] = 0;
        b[3] = h(2, 0);

        F(0, 0) = 2 * b[1] * (0.5 - fqPl[2] * fqPl[2] - fqPl[3] * fqPl[3]) + 2 * b[3] * (fqPl[1] * fqPl[3] - fqPl[0] * fqPl[2]) - mEstimate[0];
        F(1, 0) = 2 * b[1] * (fqPl[1] * fqPl[2] - fqPl[0] * fqPl[3]) + 2 * b[3] * (fqPl[0] * fqPl[1] + fqPl[2] * fqPl[3]) - mEstimate[1];
        F(2, 0) = 2 * b[1] * (fqPl[0] * fqPl[2] + fqPl[1] * fqPl[3]) + 2 * b[3] * (0.5 - fqPl[1] * fqPl[1] - fqPl[2] * fqPl[2]) - mEstimate[2];

        J(0, 0) = -2 * b[3] * fqPl[2];
        J(0, 1) = 2 * b[3] * fqPl[3];
        J(0, 2) = -4 * b[1] * fqPl[2] - 2 * b[3] * fqPl[0];
        J(0, 3) = -4 * b[1] * fqPl[3] + 2 * b[3] * fqPl[1];

        J(1, 0) = -2 * b[1] * fqPl[3] + 2 * b[3] * fqPl[1];
        J(1, 1) = 2 * b[1] * fqPl[2] + 2 * b[3] * fqPl[0];
        J(1, 2) = 2 * b[1] * fqPl[1] + 2 * b[3] * fqPl[3];
        J(1, 3) = -2 * b[1] * fqPl[0] + 2 * b[3] * fqPl[2];

        J(2, 0) = 2 * b[1] * fqPl[2];
        J(2, 1) = 2 * b[1] * fqPl[3] - 4 * b[3] * fqPl[1];
        J(2, 2) = 2 * b[1] * fqPl[0] - 4 * b[3] * fqPl[2];
        J(2, 3) = 2 * b[1] * fqPl[1];

        diff = F.norm();
        step = J.transpose() * F;
        qDotError[0] += step(0, 0);
        qDotError[1] += step(1, 0);
        qDotError[2] += step(2, 0);
        qDotError[3] += step(3, 0);
    }

#endif

    if (accNorm > 8.0 && accNorm < 12.0)
    {
        gyroMeasError = 10.0 * PI / 180;
    }
    else
    {
        gyroMeasError = 5.0 * PI / 180;
    }

    beta = sqrt(3.0 / 4.0) * gyroMeasError;

    double qDotErrorNorm = sqrt(qDotError[0] * qDotError[0] + qDotError[1] * qDotError[1] + qDotError[2] * qDotError[2] + qDotError[3] * qDotError[3]);

    if (qDotErrorNorm > 0)
    {
        qDotError[0] /= qDotErrorNorm;
        qDotError[1] /= qDotErrorNorm;
        qDotError[2] /= qDotErrorNorm;
        qDotError[3] /= qDotErrorNorm;
    }

    qDot[0] -= beta * qDotError[0];
    qDot[1] -= beta * qDotError[1];
    qDot[2] -= beta * qDotError[2];
    qDot[3] -= beta * qDotError[3];

    for (i = 0; i < 4; i++)
    {
        fqPl[i] += qDot[i] * dt;
    }

    qNorm(fqPl);
    q2dcm(fqPl, fCbn);
    dcm2euler(fCbn, euler);
    fPsiPl = euler[0];
    fThePl = euler[1];
    fPhiPl = euler[2];
    Matrix3d temp;
    temp << fCbn[0][0], fCbn[0][1], fCbn[0][2],
         fCbn[1][0], fCbn[1][1], fCbn[1][2],
         fCbn[2][0], fCbn[2][1], fCbn[2][2];
    temp.transposeInPlace();

    for (int i = CHX; i <= CHZ; i++)
    {
        for (int j = CHX; j <= CHZ; j++)
        {
            fCnb[i][j] = temp(i, j);
        }
    }
}

void SensorFusion::gyroCalibration(vector<shared_ptr<double>>& gyroArray)
{
    int i;
    double bias[3] {0, 0, 0};

    // gyro calibration
    for (auto p : gyroArray)
    {
        double* value = p.get();

        bias[0] += value[0];
        bias[1] += value[1];
        bias[2] += value[2];
    }

    for (i = 0; i < 3; i++)
    {
        bias[i] /= gyroArray.size();
    }

    //if (abs(bias[0]) < 0.01 && abs(bias[1]) < 0.01 && abs(bias[2]) < 0.01)
    {
        for (i = 0; i < 3; i++)
        {
            fGyroBias[i] = bias[i];
        }
    }
}

bool SensorFusion::sensorAlignment(vector<shared_ptr<double>>& accArray, vector<shared_ptr<double>>& magArray)
{
#if MAG_SUPPORT
    int i = 0;
    int j = 0;
    int X = 0;
    int Y = 1;
    int Z = 2;
    double ftmp = 0;
    double fg[3] {0, 0, 0};
    double fm[3] {0, 0, 0};
    double fmod[3] {0, 0, 0};
    double fR[3][3];
    double euler[3];

    for (auto p : accArray)
    {
        double* value = p.get();

        fg[0] -= value[0];
        fg[1] -= value[1];
        fg[2] -= value[2];
    }

    for (i = 0; i < 3; i++)
    {
        fg[i] = fg[i] / accArray.size();
        fR[i][Z] = fg[i];;
    }

    for (auto p : magArray)
    {
        double* value = p.get();

        fm[0] += value[0];
        fm[1] += value[1];
        fm[2] += value[2];
    }

    for (i = 0; i < 3; i++)
    {
        fm[i] = fm[i] / magArray.size() - fMagBias[i];
        fR[i][X] = fm[i];
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
        fmod[i] = sqrt(fR[X][i] * fR[X][i] + fR[Y][i] * fR[Y][i] + fR[Z][i] * fR[Z][i]);
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
        return false;
    }

    for (i = 0; i < 3; i++)
    {
        for (j = 0; j < 3; j++)
        {
            fCnb[i][j] = fR[i][j];
        }
    }

    Matrix3d temp;
    temp << fCnb[0][0], fCnb[0][1], fCnb[0][2],
         fCnb[1][0], fCnb[1][1], fCnb[1][2],
         fCnb[2][0], fCnb[2][1], fCnb[2][2];
    temp.transposeInPlace();

    for (i = CHX; i <= CHZ; i++)
    {
        for (j = CHX; j <= CHZ; j++)
        {
            fCbn[i][j] = temp(i, j);
        }
    }

    dcm2euler(fCbn, euler);
    fPsiPl = euler[0];
    fThePl = euler[1];
    fPhiPl = euler[2];
    euler2q(fqPl, fPsiPl, fThePl, fPhiPl);
    euler2dcm(fCnp, fPsiPl, 0, 0);
    temp << fCnp[0][0], fCnp[0][1], fCnp[0][2],
         fCnp[1][0], fCnp[1][1], fCnp[1][2],
         fCnp[2][0], fCnp[2][1], fCnp[2][2];
    temp.transposeInPlace();

    for (i = CHX; i <= CHZ; i++)
    {
        for (j = CHX; j <= CHZ; j++)
        {
            fCnp[i][j] = temp(i, j);
        }
    }

    return true;
#else
    int i = 0;
    int j = 0;
    double fmodGxyz = 0;
    double fmodGyz = 0;
    double frecipmodGxyz = 0;
    double ftmp = 0;
    double fg[3] {0, 0, 0};
    double euler[3];

    // tilt alignment
    for (auto p : accArray)
    {
        double* value = p.get();

        fg[0] -= value[0];
        fg[1] -= value[1];
        fg[2] -= value[2];
    }

    for (i = 0; i < 3; i++)
    {
        fg[i] = fg[i] / accArray.size();
    }

    fmodGyz = fg[1] * fg[1] + fg[2] * fg[2];
    fmodGxyz = fmodGyz + fg[0] * fg[0];

    // check for free fall special case where no solution is possible
    if (fmodGxyz == 0.0F)
    {
        return false;
    }

    // check for vertical up or down gimbal lock case
    if (fmodGyz == 0.0F)
    {
        return false;
    }

    // compute moduli for the general case
    fmodGyz = sqrt(fmodGyz);
    fmodGxyz = sqrt(fmodGxyz);
    frecipmodGxyz = 1.0 / fmodGxyz;
    ftmp = fmodGxyz / fmodGyz;

    // normalize the accelerometer reading into the z column
    for (i = 0; i < 3; i++)
    {
        fCnb[i][2] = fg[i] * frecipmodGxyz;
    }

    // construct x column of orientation matrix
    fCnb[0][0] = fmodGyz * frecipmodGxyz;
    fCnb[1][0] = -fCnb[0][2] * fCnb[1][2] * ftmp;
    fCnb[2][0] = -fCnb[0][2] * fCnb[2][2] * ftmp;

    // construct y column of orientation matrix
    fCnb[0][1] = 0;
    fCnb[1][1] = fCnb[2][2] * ftmp;
    fCnb[2][1] = -fCnb[1][2] * ftmp;

    Matrix3d temp;
    temp << fCnb[0][0], fCnb[0][1], fCnb[0][2],
         fCnb[1][0], fCnb[1][1], fCnb[1][2],
         fCnb[2][0], fCnb[2][1], fCnb[2][2];
    temp.transposeInPlace();

    for (i = CHX; i <= CHZ; i++)
    {
        for (j = CHX; j <= CHZ; j++)
        {
            fCbn[i][j] = temp(i, j);
        }
    }

    dcm2euler(fCbn, euler);
    fPsiPl = euler[0];
    fThePl = euler[1];
    fPhiPl = euler[2];
    euler2q(fqPl, fPsiPl, fThePl, fPhiPl);

    return true;
#endif
}

int SensorFusion::staticDetectUpdate(double gyro[], double acc[], double mag[])
{
    if (fAlignGyroArray.size() >= ALIGN_NUM)
    {
        fAlignAccArray.erase(fAlignAccArray.begin());
        fAlignGyroArray.erase(fAlignGyroArray.begin());
        fAlignMagArray.erase(fAlignMagArray.begin());
    }

    fAlignAccArray.push_back(shared_ptr<double>(new double[3] {acc[0], acc[1], acc[2]}, [](double * p)
    {
        delete[] p;
    }));
    fAlignGyroArray.push_back(shared_ptr<double>(new double[3] {gyro[0], gyro[1], gyro[2]}, [](double * p)
    {
        delete[] p;
    }));
#if MAG_SUPPORT
    fAlignMagArray.push_back(shared_ptr<double>(new double[3] {mag[0], mag[1], mag[2]}, [](double * p)
    {
        delete[] p;
    }));
#endif

    return 0;
}

int SensorFusion::staticDetectCheck()
{
    double gyro_std = 0;
    double acc_std = 0;

    if (fAlignGyroArray.size() == ALIGN_NUM && fAlignAccArray.size() == ALIGN_NUM)
    {
        gyro_std = stdCal(fAlignGyroArray);
        acc_std = stdCal(fAlignAccArray);

        if (gyro_std < 0.01 && acc_std < 0.1)
        {
            return 1;
        }
        else if (gyro_std > 0.5 && acc_std > 1.0)
        {
            return 0;
        }
    }

    return -1;
}

double SensorFusion::stdCal(vector<shared_ptr<double>>& numList)
{
    double det = 0;
    double value = 0; // the sum of Xi
    double square = 0; // the sum of xi^2
    double sigma = 0; // standard deviation
    double size = numList.size();

    for (auto val : numList)
    {
        double* p = val.get();
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

void SensorFusion::magCalibrationInit()
{
    // initialize the mag variable
    fB = 0;
    ftrB = 0;
    ftrFitErrorpc = 0;
    fFitErrorpc = 0;
    iValidMagCal = false;

    for (int i = CHX; i <= CHZ; i++)
    {
        ftrV[i] = 0;
        fV[i] = 0;
    }

    // initialize the mag buffer
    iMagBufferCount = 0;

    for (int i = 0; i < MAGBUFFSIZEX; i++)
    {
        for (int j = 0; j < MAGBUFFSIZEY; j++)
        {
            Index[i][j] = -1;
        }
    }

    fuTPerCount = MAGSENSITIVE;
    fCountsPeruT = (float)(1.0 / fuTPerCount);

    // initialize the array of (MAGBUFFSIZEX - 1) elements of 100 * tangents used for buffer indexing
    // entries cover the range 100 * tan(-PI/2 + PI/MAGBUFFSIZEX), 100 * tan(-PI/2 + 2*PI/MAGBUFFSIZEX) to
    // 100 * tan(-PI/2 + (MAGBUFFSIZEX - 1) * PI/MAGBUFFSIZEX).
    // for MAGBUFFSIZEX=14, the entries range in value from -438 to +438
    for (int i = 0; i < (MAGBUFFSIZEX - 1); i++)
    {
        Tanarray[i] = (int)(100.0F * tan(PI * (-0.5F + (i + 1) * 1.0F / MAGBUFFSIZEX)));
    }
}

int SensorFusion::magBufferUpdate(double magRaw[], double magCal[], int loopCounter)
{
    int i = 0;
    int j = 0;
    int k = 0;
    int l = 0;
    int m = 0;
    int iMagRaw[3];
    int iMagCal[3];
    int itanj = 0;
    int itank = 0;
    int idelta = 0;
    int iclose = 0;

    // convert float mag data to int mag data to reduce multiplications
    for (i = CHX; i <= CHZ; i++)
    {
        iMagRaw[i] = (int)(magRaw[i] * fCountsPeruT);
        iMagCal[i] = (int)(magCal[i] * fCountsPeruT);
    }

    if (iMagCal[CHZ] == 0)
    {
        return -1;
    }

    itanj = 100 * iMagCal[CHX] / iMagCal[CHZ];
    itank = 100 * iMagCal[CHY] / iMagCal[CHZ];

    while ((j < (MAGBUFFSIZEX - 1) && (itanj >= Tanarray[j])))
    {
        j++;
    }

    while ((k < (MAGBUFFSIZEX - 1) && (itank >= Tanarray[k])))
    {
        k++;
    }

    if (iMagCal[CHX] < 0)
    {
        k += MAGBUFFSIZEX;
    }

    // case 1: buffer is full and this bin has a measurement: over-write without increasing number of measurements
    // this is the most common option at run time
    if ((iMagBufferCount == MAXMEASUREMENTS) && (Index[j][k] != -1))
    {
        for (i = CHX; i <= CHZ; i++)
        {
            iMagRawBuffer[i][j][k] = iMagRaw[i];
        }

        Index[j][k] = loopCounter;

        return 0;
    }

    // case 2: the buffer is full and this bin does not have a measurement: store and retire the oldest
    // this is the second most common option at run time
    if ((iMagBufferCount == MAXMEASUREMENTS) && (Index[j][k] == -1))
    {
        for (i = CHX; i <= CHZ; i++)
        {
            iMagRawBuffer[i][j][k] = iMagRaw[i];
        }

        Index[j][k] = loopCounter;

        // set l and m to the oldest active entry and disable it
        for (j = 0; j < MAGBUFFSIZEX; j++)
        {
            for (k = 0; k < MAGBUFFSIZEY; k++)
            {
                // check if the time stamp is older than the oldest found so far (normally fails this test)
                if (Index[j][k] < (int)loopCounter)
                {
                    // check if this bin is active (normally passes this test)
                    if (Index[j][k] != -1)
                    {
                        // set l and m to the indices of the oldest entry found so far
                        l = j;
                        m = k;
                        // set i to the time stamp of the oldest entry found so far
                        i = Index[l][m];
                    }
                }
            }
        }

        // deactivate the oldest measurement (no need to zero the measurement data)
        Index[l][m] = -1;

        return 0;
    }

    // case 3: buffer is not full and this bin is empty: store and increment number of measurements
    if ((iMagBufferCount < MAXMEASUREMENTS) && (Index[j][k] == -1))
    {
        for (i = CHX; i <= CHZ; i++)
        {
            iMagRawBuffer[i][j][k] = iMagRaw[i];
        }

        Index[j][k] = loopCounter;
        iMagBufferCount++;

        return 0;
    }

    // case 4: buffer is not full and this bin has a measurement: over-write if close or try to slot in
    // elsewhere if close to the other measurements so as to create a mesh at power up
    if ((iMagBufferCount < MAXMEASUREMENTS) && (Index[j][k] != -1))
    {
        // calculate the vector difference between current measurement and the buffer entry
        idelta = 0;

        for (i = CHX; i <= CHZ; i++)
        {
            idelta += abs(iMagRaw[i] - iMagRawBuffer[i][j][k]);
        }

        // check to see if the current reading is close to this existing magnetic buffer entry
        if (idelta < MESHDELTAUT * fCountsPeruT)
        {
            // simply over-write the measurement and return
            for (i = CHX; i <= CHZ; i++)
            {
                iMagRawBuffer[i][j][k] = iMagRaw[i];
            }

            Index[j][k] = loopCounter;
        }
        else
        {
            // reset the flag denoting that the current measurement is close to any measurement in the buffer
            iclose = 0;
            // to avoid compiler warning
            l = m = 0;
            // loop over the buffer j from 0 potentially up to MAGBUFFSIZEX - 1
            j = 0;

            while (iclose == 0 && (j < MAGBUFFSIZEX))
            {
                // loop over the buffer k from 0 potentially up to MAGBUFFSIZEY - 1
                k = 0;

                while (iclose == 0 && (k < MAGBUFFSIZEY))
                {
                    // check whether this buffer entry already has a measurement or not
                    if (Index[j][k] != -1)
                    {
                        // calculate the vector difference between current measurement and the buffer entry
                        idelta = 0;

                        for (i = CHX; i <= CHZ; i++)
                        {
                            idelta += abs(iMagRaw[i] - iMagRawBuffer[i][j][k]);
                        }

                        // check to see if the current reading is close to this existing magnetic buffer entry
                        if (idelta < MESHDELTAUT)
                        {
                            // set the flag to abort the search
                            iclose = 1;
                        }
                    }
                    else
                    {
                        // store the location of this empty bin for future use
                        l = j;
                        m = k;
                    } // end of test for valid measurement in this bin

                    k++;
                } // end of k loop

                j++;
            } // end of j loop

            // if none too close, store the measurement in the last empty bin found and return
            // l and m are guaranteed to be set if no entries too close are detected
            if (iclose == 0)
            {
                for (i = CHX; i <= CHZ; i++)
                {
                    iMagRawBuffer[i][l][m] = iMagRaw[i];
                }

                Index[l][m] = loopCounter;
                iMagBufferCount++;
            }
        }

        return 0;
    }

    return -1;
}

int SensorFusion::magCalibrationExec()
{
    int i = 0;
    int j = 0;
    int isolver = 0;

    // 4 element calibration case
    if (iMagBufferCount > 200)
    {
        isolver = 4;
        calibration4INV();
    }

    if (ftrFitErrorpc <= MAGFITERROR && ftrB > 10 && ftrB < 100)
    {
        if (iValidMagCal == false || ftrFitErrorpc <= fFitErrorpc || ftrFitErrorpc <= 2.0F)
        {
            iValidMagCal = true;
            fFitErrorpc = ftrFitErrorpc;
            fB = ftrB;

            for (i = CHX; i <= CHZ; i++)
            {
                fMagBias[i] = fV[i] = ftrV[i];
            }
        }
    }

    return isolver;
}

void SensorFusion::calibration4INV()
{
    // local variables
    double fBs2;                            // fBs[CHX]^2+fBs[CHY]^2+fBs[CHZ]^2
    double fSumBs4;                         // sum of fBs2
    double fscaling;                        // set to FUTPERCOUNT * FMATRIXSCALING
    double fE;                              // error function = r^T.r
    int iOffset[3];                         // offset to remove large DC hard iron bias in matrix
    int iCount;                             // number of measurements counted
    int ierror;                             // matrix inversion error flag
    int i, j, k, l;                         // loop counters

    double DEFAULTB = 50.0;
    double fvecB[4];
    double fvecA[6];
    double fmatA[4][4];
    double fmatB[4][4];

    // compute fscaling to reduce multiplications later
    fscaling = fuTPerCount / DEFAULTB;

    // zero fSumBs4=Y^T.Y, fvecB=X^T.Y (4x1) and on and above diagonal elements of fmatA=X^T*X (4x4)
    fSumBs4 = 0.0;

    for (i = 0; i < 4; i++)
    {
        fvecB[i] = 0.0;

        for (j = i; j < 4; j++)
        {
            fmatA[i][j] = 0.0;
        }
    }

    // the offsets are guaranteed to be set from the first element but to avoid compiler error
    iOffset[0] = iOffset[1] = iOffset[2] = 0;

    // use entries from magnetic buffer to compute matrices
    iCount = 0;

    for (j = 0; j < MAGBUFFSIZEX; j++)
    {
        for (k = 0; k < MAGBUFFSIZEY; k++)
        {
            if (Index[j][k] != -1)
            {
                // use first valid magnetic buffer entry as estimate (in counts) for offset
                if (iCount == 0)
                {
                    for (l = 0; l <= 2; l++)
                    {
                        iOffset[l] = iMagRawBuffer[l][j][k];
                    }
                }

                // store scaled and offset fBs[XYZ] in fvecA[0-2] and fBs[XYZ]^2 in fvecA[3-5]
                for (l = 0; l <= 2; l++)
                {
                    fvecA[l] = (iMagRawBuffer[l][j][k] - iOffset[l]) * fscaling;
                    fvecA[l + 3] = fvecA[l] * fvecA[l];
                }

                // calculate fBs2 = fBs[CHX]^2 + fBs[CHY]^2 + fBs[CHZ]^2 (scaled uT^2)
                fBs2 = fvecA[3] + fvecA[4] + fvecA[5];
                // accumulate fBs^4 over all measurements into fSumBs4=Y^T.Y
                fSumBs4 += fBs2 * fBs2;

                // now we have fBs2, accumulate fvecB[0-2] = X^T.Y =sum(fBs2.fBs[XYZ])
                for (l = 0; l <= 2; l++)
                {
                    fvecB[l] += fvecA[l] * fBs2;
                }

                //accumulate fvecB[3] = X^T.Y =sum(fBs2)
                fvecB[3] += fBs2;
                // accumulate on and above-diagonal terms of fmatA = X^T.X ignoring fmatA[3][3]
                fmatA[0][0] += fvecA[0 + 3];
                fmatA[0][1] += fvecA[0] * fvecA[1];
                fmatA[0][2] += fvecA[0] * fvecA[2];
                fmatA[0][3] += fvecA[0];
                fmatA[1][1] += fvecA[1 + 3];
                fmatA[1][2] += fvecA[1] * fvecA[2];
                fmatA[1][3] += fvecA[1];
                fmatA[2][2] += fvecA[2 + 3];
                fmatA[2][3] += fvecA[2];
                // increment the counter for next iteration
                iCount++;
            }
        }
    }

    // set the last element of the measurement matrix to the number of buffer elements used
    fmatA[3][3] = (double) iCount;

    // use above diagonal elements of symmetric fmatA to set both fmatB and fmatA to X^T.X
    for (i = 0; i < 4; i++)
    {
        for (j = i; j < 4; j++)
        {
            fmatB[i][j] = fmatB[j][i] = fmatA[j][i] = fmatA[i][j];
        }
    }

    // calculate in situ inverse of fmatB = inv(X^T.X) (4x4) while fmatA still holds X^T.X
    Matrix4d temp;
    temp << fmatB[0][0], fmatB[0][1], fmatB[0][2], fmatB[0][3],
         fmatB[1][0], fmatB[1][1], fmatB[1][2], fmatB[1][3],
         fmatB[2][0], fmatB[2][1], fmatB[2][2], fmatB[2][3],
         fmatB[3][0], fmatB[3][1], fmatB[3][2], fmatB[3][3];
    temp = temp.inverse().eval();

    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            fmatB[i][j] = temp(i, j);
        }
    }

    // calculate fvecA = solution beta (4x1) = inv(X^T.X).X^T.Y = fmatB * fvecB
    for (i = 0; i < 4; i++)
    {
        fvecA[i] = 0.0F;

        for (k = 0; k < 4; k++)
        {
            fvecA[i] += fmatB[i][k] * fvecB[k];
        }
    }

    // calculate P = r^T.r = Y^T.Y - 2 * beta^T.(X^T.Y) + beta^T.(X^T.X).beta
    // = fSumBs4 - 2 * fvecA^T.fvecB + fvecA^T.fmatA.fvecA
    // first set P = Y^T.Y - 2 * beta^T.(X^T.Y) = fSumBs4 - 2 * fvecA^T.fvecB
    fE = 0.0F;

    for (i = 0; i < 4; i++)
    {
        fE += fvecA[i] * fvecB[i];
    }

    fE = fSumBs4 - 2.0F * fE;

    // set fvecB = (X^T.X).beta = fmatA.fvecA
    for (i = 0; i < 4; i++)
    {
        fvecB[i] = 0.0F;

        for (k = 0; k < 4; k++)
        {
            fvecB[i] += fmatA[i][k] * fvecA[k];
        }
    }

    // complete calculation of P by adding beta^T.(X^T.X).beta = fvecA^T * fvecB
    for (i = 0; i < 4; i++)
    {
        fE += fvecB[i] * fvecA[i];
    }

    // compute the hard iron vector (in uT but offset and scaled by FMATRIXSCALING)
    for (l = 0; l <= 2; l++)
    {
        ftrV[l] = 0.5F * fvecA[l];
    }

    // compute the scaled geomagnetic field strength B (in uT but scaled by FMATRIXSCALING)
    ftrB = sqrt(fvecA[3] + ftrV[0] * ftrV[0] + ftrV[1] * ftrV[1] + ftrV[2] * ftrV[2]);
    // calculate the trial fit error (percent) normalized to number of measurements and scaled geomagnetic field strength
    ftrFitErrorpc = sqrt(fE / 300) * 100.0F / (2.0F * ftrB * ftrB);

    // correct the hard iron estimate for FMATRIXSCALING and the offsets applied (result in uT)
    for (l = CHX; l <= CHZ; l++)
    {
        ftrV[l] = ftrV[l] * DEFAULTB + iOffset[l] * 1.0 * fuTPerCount;
    }

    // correct the geomagnetic field strength B to correct scaling (result in uT)
    ftrB *= DEFAULTB;
}

void SensorFusion::gyroFilter(double gyro[])
{
    int i;
    double temp;

    for (i = CHX; i <= CHZ; i++)
    {
        temp = LpfGyroB[0] * gyro[i] + LpfGyroB[1] * LpfGyroX[i][0] + LpfGyroB[2] * LpfGyroX[i][1]
               - LpfGyroA[1] * LpfGyroY[i][0] - LpfGyroA[2] * LpfGyroY[i][1];
        LpfGyroX[i][1] = LpfGyroX[i][0];
        LpfGyroX[i][0] = gyro[i];
        LpfGyroY[i][1] = LpfGyroY[i][0];
        LpfGyroY[i][0] = temp;
        gyro[i] = temp;
    }
}

void SensorFusion::accFilter(double acc[])
{
    int i;
    double temp;

    for (i = CHX; i <= CHZ; i++)
    {
        temp = LpfAccB[0] * acc[i] + LpfAccB[1] * LpfAccX[i][0] + LpfAccB[2] * LpfAccX[i][1]
               - LpfAccA[1] * LpfAccY[i][0] - LpfAccA[2] * LpfAccY[i][1];
        LpfAccX[i][1] = LpfAccX[i][0];
        LpfAccX[i][0] = acc[i];
        LpfAccY[i][1] = LpfAccY[i][0];
        LpfAccY[i][0] = temp;
        acc[i] = temp;
    }
}

#define OMEGA_MARGIN (10)
void SensorFusion::outlierCompensate(vector<shared_ptr<SampleData>>& sampleDataArray, double gyro[])
{
    int left_index = -1;
    int right_index = -1;
    int index = 0;
    bool outlier_flag = false;

    index = 0;
    for (auto p : sampleDataArray)
    {
        SampleData *val = p.get();
        if (left_index == -1)
        {
            if (abs(val->fOmegaBRaw[CHZ]) > (MAX_OMEGA_DEG - OMEGA_MARGIN) * DEG2RAD)
            {
                left_index = index;
            }
        }
        else
        {
            if (right_index == -1)
            {
                if (abs(val->fOmegaBRaw[CHZ]) < (MAX_OMEGA_DEG - OMEGA_MARGIN) * DEG2RAD)
                {
                    right_index = index - 1;
                    outlier_flag = true;
                }
            }
        }

        if (outlier_flag)
        {
            break;
        }
        index++;
    }

    if (outlier_flag)
    {
        /* estimate the outlier value */
        Eigen::VectorXd xvals(6);
        Eigen::VectorXd yvals(xvals.rows());
        int seq_left = left_index - 1;
        int seq_right = right_index + 1;

        // check the index valid
        if (seq_left-2 < 0 || seq_right+2 > sampleDataArray.size()-1)
        {
            return;
        }

        xvals << seq_left-2, seq_left-1, seq_left, seq_right, seq_right+1, seq_right+2;
        yvals << sampleDataArray.at(seq_left-2)->fOmegaBRaw[CHZ],
                 sampleDataArray.at(seq_left-1)->fOmegaBRaw[CHZ],
                 sampleDataArray.at(seq_left)->fOmegaBRaw[CHZ],
                 sampleDataArray.at(seq_right)->fOmegaBRaw[CHZ],
                 sampleDataArray.at(seq_right+1)->fOmegaBRaw[CHZ],
                 sampleDataArray.at(seq_right+2)->fOmegaBRaw[CHZ];

        SplineFunction s(xvals, yvals);
        for (int i = left_index; i <= right_index; i++)
        {
            sampleDataArray.at(i)->fOmegaBRaw[CHZ] = s(i);
        }

        /* recompute the past filtered gyro value */
        double lpfGyroX[2];
        double lpfGyroY[2];
        //left_index = 2;
        lpfGyroX[0] = sampleDataArray.at(left_index-1)->fOmegaBRaw[CHZ];
        lpfGyroX[1] = sampleDataArray.at(left_index-2)->fOmegaBRaw[CHZ];
        lpfGyroY[0] = sampleDataArray.at(left_index-1)->fOmegaB[CHZ]+fGyroBias[CHZ];
        lpfGyroY[1] = sampleDataArray.at(left_index-2)->fOmegaB[CHZ]+fGyroBias[CHZ];

        for (int i = left_index; i < sampleDataArray.size(); i++)
        {
            double gyroZ = sampleDataArray.at(i)->fOmegaBRaw[CHZ];
            double temp = LpfGyroB[0] * gyroZ + LpfGyroB[1] * lpfGyroX[0] + LpfGyroB[2] * lpfGyroX[1]
                        - LpfGyroA[1] * lpfGyroY[0] - LpfGyroA[2] * lpfGyroY[1];
            lpfGyroX[1] = lpfGyroX[0];
            lpfGyroX[0] = gyroZ;
            lpfGyroY[1] = lpfGyroY[0];
            lpfGyroY[0] = temp;
            temp -= fGyroBias[CHZ];
            sampleDataArray.at(i)->fOmegaB[CHZ] = temp;
            //Todo: remove it if integrated in iOS
            extern FILE* fpGyroCali;
            extern FILE* fpGyroRaw;
            fprintf(fpGyroCali, "$%d, %f, %f, %f\n", sampleDataArray.at(i)->uTime, sampleDataArray.at(i)->fOmegaB[CHX], sampleDataArray.at(i)->fOmegaB[CHY], sampleDataArray.at(i)->fOmegaB[CHZ]);
            fprintf(fpGyroRaw, "$%d, %f, %f, %f\n", sampleDataArray.at(i)->uTime, sampleDataArray.at(i)->fOmegaBRaw[CHX], sampleDataArray.at(i)->fOmegaBRaw[CHY], sampleDataArray.at(i)->fOmegaBRaw[CHZ]);
            //Todo: remove it if integrated in iOS
        }

        /* recompute the current filtered gyro value */
        double gyroZ = fOmegaBRaw[CHZ];
        double temp = LpfGyroB[0] * gyroZ + LpfGyroB[1] * lpfGyroX[0] + LpfGyroB[2] * lpfGyroX[1]
                      - LpfGyroA[1] * lpfGyroY[0] - LpfGyroA[2] * lpfGyroY[1];
        lpfGyroX[1] = lpfGyroX[0];
        lpfGyroX[0] = gyroZ;
        lpfGyroY[1] = lpfGyroY[0];
        lpfGyroY[0] = temp;
        temp -= fGyroBias[CHZ];
        fOmegaB[CHZ] = gyro[CHZ] = temp;
        // store the new filter parameters
        LpfGyroX[CHZ][1] = lpfGyroX[1];
        LpfGyroX[CHZ][0] = lpfGyroX[0];
        LpfGyroY[CHZ][1] = lpfGyroY[1];
        LpfGyroY[CHZ][0] = lpfGyroY[0];

        /* recompute the past attitude */
        Matrix3d cbn;
        Matrix3d cnp;
        Matrix3d cbnPlatform;
        double fEuler[3];
        double fCbn[3][3];
        double fqBase[4];

        cbnPlatform << sampleDataArray.at(left_index-1)->fCbnPlat[0][0], sampleDataArray.at(left_index-1)->fCbnPlat[0][1], sampleDataArray.at(left_index-1)->fCbnPlat[0][2],
                       sampleDataArray.at(left_index-1)->fCbnPlat[1][0], sampleDataArray.at(left_index-1)->fCbnPlat[1][1], sampleDataArray.at(left_index-1)->fCbnPlat[1][2],
                       sampleDataArray.at(left_index-1)->fCbnPlat[2][0], sampleDataArray.at(left_index-1)->fCbnPlat[2][1], sampleDataArray.at(left_index-1)->fCbnPlat[2][2];

        cnp << fCnp[0][0], fCnp[0][1], fCnp[0][2],
                fCnp[1][0], fCnp[1][1], fCnp[1][2],
                fCnp[2][0], fCnp[2][1], fCnp[2][2];

        cnp.transposeInPlace();
        cbn = cnp * cbnPlatform;
        for (int i = CHX; i <= CHZ; i++)
        {
            for (int j = CHX; j <= CHZ; j++)
            {
                fCbn[i][j] = cbn(i, j);
            }
        }
        dcm2euler(fCbn, fEuler);
        euler2q(fqBase, fEuler[0], fEuler[1], fEuler[2]);
        for (int k = left_index; k < sampleDataArray.size(); k++)
        {
            double fGyro[3] = {sampleDataArray.at(k)->fOmegaB[CHX], sampleDataArray.at(k)->fOmegaB[CHY], sampleDataArray.at(k)->fOmegaB[CHZ]};
            double fdq[4] {0, 0, 0, 0};


            fdq[0] = -(fGyro[0] * fqBase[1] + fGyro[1] * fqBase[2] + fGyro[2] * fqBase[3]) / 2.0;
            fdq[1] = (fGyro[0] * fqBase[0] + fGyro[2] * fqBase[2] - fGyro[1] * fqBase[3]) / 2.0;
            fdq[2] = (fGyro[1] * fqBase[0] - fGyro[2] * fqBase[1] + fGyro[0] * fqBase[3]) / 2.0;
            fdq[3] = (fGyro[2] * fqBase[0] + fGyro[1] * fqBase[1] - fGyro[0] * fqBase[2]) / 2.0;

            for (int i = 0; i < 4; i++)
            {
                fqBase[i] += fdq[i] * dt;
            }
            qNorm(fqBase);
            q2dcm(fqBase, fCbn);
            cbn << fCbn[0][0], fCbn[0][1], fCbn[0][2],
                    fCbn[1][0], fCbn[1][1], fCbn[1][2],
                    fCbn[2][0], fCbn[2][1], fCbn[2][2];
            cnp << fCnp[0][0], fCnp[0][1], fCnp[0][2],
                    fCnp[1][0], fCnp[1][1], fCnp[1][2],
                    fCnp[2][0], fCnp[2][1], fCnp[2][2];
            cbnPlatform = cnp * cbn;
            for (int i = CHX; i <= CHZ; i++)
            {
                for (int j = CHX; j <= CHZ; j++)
                {
                    sampleDataArray.at(k)->fCbnPlat[i][j] = cbnPlatform(i, j);
                }
            }
            dcm2euler(sampleDataArray.at(k)->fCbnPlat, fEuler);
            euler2q(sampleDataArray.at(k)->fqPlPlat, fEuler[0], fEuler[1], fEuler[2]);
            sampleDataArray.at(k)->fPsiPlPlat = fEuler[0];
            sampleDataArray.at(k)->fThePlPlat = fEuler[1];
            sampleDataArray.at(k)->fPhiPlPlat = fEuler[2];
            sampleDataArray.at(k)->fLinerAccN = sampleDataArray.at(k)->fAccelerate[0] * sampleDataArray.at(k)->fCbnPlat[0][0] + sampleDataArray.at(k)->fAccelerate[1] * sampleDataArray.at(k)->fCbnPlat[0][1] + sampleDataArray.at(k)->fAccelerate[2] * sampleDataArray.at(k)->fCbnPlat[0][2];
            sampleDataArray.at(k)->fLinerAccE = sampleDataArray.at(k)->fAccelerate[0] * sampleDataArray.at(k)->fCbnPlat[1][0] + sampleDataArray.at(k)->fAccelerate[1] * sampleDataArray.at(k)->fCbnPlat[1][1] + sampleDataArray.at(k)->fAccelerate[2] * sampleDataArray.at(k)->fCbnPlat[1][2];
            sampleDataArray.at(k)->fLinerAccD = sampleDataArray.at(k)->fAccelerate[0] * sampleDataArray.at(k)->fCbnPlat[2][0] + sampleDataArray.at(k)->fAccelerate[1] * sampleDataArray.at(k)->fCbnPlat[2][1] + sampleDataArray.at(k)->fAccelerate[2] * sampleDataArray.at(k)->fCbnPlat[2][2];
            for (int i = CHX; i <= CHZ; i++)
            {
                sampleDataArray.at(k)->fOmegaN[i] = sampleDataArray.at(k)->fCbnPlat[i][0] * sampleDataArray.at(k)->fOmegaB[0] + sampleDataArray.at(k)->fCbnPlat[i][1] * sampleDataArray.at(k)->fOmegaB[1] + sampleDataArray.at(k)->fCbnPlat[i][2] * sampleDataArray.at(k)->fOmegaB[2];
            }
        }

        /* recompute the current attitude */
        double fGyro[3] = {fOmegaB[CHX], fOmegaB[CHY], fOmegaB[CHZ]};
        double fdq[4] {0, 0, 0, 0};
        Matrix3d cnbTemp;

        fdq[0] = -(fGyro[0] * fqBase[1] + fGyro[1] * fqBase[2] + fGyro[2] * fqBase[3]) / 2.0;
        fdq[1] = (fGyro[0] * fqBase[0] + fGyro[2] * fqBase[2] - fGyro[1] * fqBase[3]) / 2.0;
        fdq[2] = (fGyro[1] * fqBase[0] - fGyro[2] * fqBase[1] + fGyro[0] * fqBase[3]) / 2.0;
        fdq[3] = (fGyro[2] * fqBase[0] + fGyro[1] * fqBase[1] - fGyro[0] * fqBase[2]) / 2.0;

        for (int i = 0; i < 4; i++)
        {
            fqBase[i] += fdq[i] * dt;
        }
        qNorm(fqBase);
        q2dcm(fqBase, fCbn);
        cbn << fCbn[0][0], fCbn[0][1], fCbn[0][2],
                fCbn[1][0], fCbn[1][1], fCbn[1][2],
                fCbn[2][0], fCbn[2][1], fCbn[2][2];
        cnp << fCnp[0][0], fCnp[0][1], fCnp[0][2],
                fCnp[1][0], fCnp[1][1], fCnp[1][2],
                fCnp[2][0], fCnp[2][1], fCnp[2][2];
        cbnPlatform = cnp * cbn;
        cnbTemp << fCbn[0][0], fCbn[0][1], fCbn[0][2],
                fCbn[1][0], fCbn[1][1], fCbn[1][2],
                fCbn[2][0], fCbn[2][1], fCbn[2][2];
        cnbTemp.transposeInPlace();

        for (int i = CHX; i <= CHZ; i++)
        {
            for (int j = CHX; j <= CHZ; j++)
            {
                fCbnPlat[i][j] = cbnPlatform(i, j);
                this->fCbn[i][j] = fCbn[i][j];
                this->fCnb[i][j] = cnbTemp(i, j);
            }
        }
        dcm2euler(fCbn, fEuler);
        fPsiPl = fEuler[0];
        fThePl = fEuler[1];
        fPhiPl = fEuler[2];
        for (int i = 0; i < 4; i++)
        {
            this->fqPl[i] = fqBase[i];
        }
        dcm2euler(fCbnPlat, fEuler);
        fPsiPlPlat = fEuler[0];
        fThePlPlat = fEuler[1];
        fPhiPlPlat = fEuler[2];
        euler2q(fqPlPlat, fEuler[0], fEuler[1], fEuler[2]);
    }
}

void SensorFusion::refineSampleData(vector<shared_ptr<SampleData>>& sampleDataArray)
{
    double fOmegaMax = -100;
    double fOmegaMin = 100;
    double fOmegaPeak = 0;
    double fOmegaFirst = 0;
    double fOmegaLast = 0;
    double fOmegaLetter = 0;
    double fScale = 10;
    int startIndex = 0;
    int endIndex = 0;
    double fAccMaxX = 0;
    double fAccMinX = 0;
    int fAccMaxIndex = 0;
    int fAccMinIndex = 0;
    int arraySize = 0;
    int index = 0;

    if (sampleDataArray.size() == 0)
    {
        return;
    }

    // check the action type roughly
    fPlatformOmegaMaxZ = 0;
    fPlatformOmegaMinZ = 0;

    index = 0;

    for (auto p : sampleDataArray)
    {
        SampleData* val = p.get();
        double linerAccX = 0;

        // calculate the liner accelerate along the x axis
        linerAccX = val->fAccelerate[0] * val->fCbnPlat[0][0] + val->fAccelerate[1] * val->fCbnPlat[0][1] + val->fAccelerate[2] * val->fCbnPlat[0][2];

        if (linerAccX > fAccMaxX)
        {
            fAccMaxX = linerAccX;
            fAccMaxIndex = index;
        }

        if (linerAccX < fAccMinX)
        {
            fAccMinX = linerAccX;
            fAccMinIndex = index;
        }

        // platform omega
        if (val->fOmegaN[CHZ] > fPlatformOmegaMaxZ)
        {
            fPlatformOmegaMaxZ = val->fOmegaN[CHZ];
        }

        if (val->fOmegaN[CHZ] < fPlatformOmegaMinZ)
        {
            fPlatformOmegaMinZ = val->fOmegaN[CHZ];
        }

        // body omega
        if (val->fOmegaB[2] > fOmegaMax)
        {
            fOmegaMax = val->fOmegaB[2];
        }

        if (val->fOmegaB[2]  < fOmegaMin)
        {
            fOmegaMin = val->fOmegaB[2];
        }

        index++;
    }

    // general refine:
    // remove the false peak
    double fLastLinerAccX = 0;
    bool bCurveRising = true;
    int tempIndex = 0;
    index = 0;

    for (auto p : sampleDataArray)
    {
        SampleData* val = p.get();
        double linerAccX = val->fLinerAccN;

        if (linerAccX < fLastLinerAccX && bCurveRising)
        {
            // reach the peak and check the peak
            if (abs(fLastLinerAccX - fAccMaxX) < 0.01)
            {
                // the max peak
                startIndex = tempIndex;
            }
            else
            {
                // check the peak value
                if (fLastLinerAccX > 0.5 * fAccMaxX)
                {
                    startIndex = tempIndex;
                }
            }
            bCurveRising = false;
        }

        if (!bCurveRising)
        {
            if (linerAccX < 0)
            {
                break;
            }
            if (linerAccX > fLastLinerAccX)
            {
                tempIndex = index - 1;
                bCurveRising = true;
            }
        }

        fLastLinerAccX = linerAccX;
        index++;
    }

    // refine the sample data array
    if (startIndex > 0)
    {
        for (int i = 0; i < startIndex; i++)
        {
            sampleDataArray.erase(sampleDataArray.begin());
            endIndex--;
        }
    }

    // special refine:
    if (fPlatformOmegaMaxZ < 8 && abs(fPlatformOmegaMinZ) < 8 && fOmegaMax < 5 && abs(fOmegaMin) < 5)
    {
        // no rotation, it is push

        // calculate the ins info firstly
        insStrapdownMechanization(dt, sampleDataArray);
        // remove the backward action
        endIndex = 0;
        index = 0;

        for (auto p : sampleDataArray)
        {
            SampleData* val = p.get();
            int lastIndex = index - 1;

            if (lastIndex > 0 && val->fPosN < sampleDataArray.at(lastIndex)->fPosN)
            {
                endIndex = index;
                break;
            }

            index ++;
        }

        if (endIndex > 0)
        {
            arraySize = sampleDataArray.size();

            for (int i = 0; i < arraySize - endIndex; i++)
            {
                sampleDataArray.erase(sampleDataArray.end() - 1);
            }
        }
    }
    else if (abs(fPlatformOmegaMaxZ) > abs(fPlatformOmegaMinZ))
    {
        // backhand action
        fOmegaFirst = sampleDataArray.at(0)->fOmegaB[CHZ];
        fOmegaLast = sampleDataArray.at(sampleDataArray.size() - 1)->fOmegaB[CHZ];

        if (fOmegaMax > 0 && fOmegaMax > fOmegaFirst && fOmegaMax > fOmegaLast)
        {
            fOmegaPeak = fOmegaMax;
            fOmegaLetter = 1;
        }
        else if (fOmegaMin < 0 && fOmegaMin < fOmegaFirst && fOmegaMin < fOmegaLast)
        {
            fOmegaPeak = fOmegaMin;
            fOmegaLetter = -1;
        }
        else
        {
            // abnormal case
        }

        // remove the negative gyro Z actions
        for (int i = 0; i < sampleDataArray.size(); i++)
        {
            if (sampleDataArray.at(sampleDataArray.size() - 1 - i)->fOmegaB[CHZ] * fOmegaLetter > 0)
            {
                endIndex = sampleDataArray.size() - 1 - i;
                break;
            }
        }

        if (endIndex > 0)
        {
            arraySize = sampleDataArray.size();

            for (int i = 0; i < arraySize - endIndex; i++)
            {
                sampleDataArray.erase(sampleDataArray.end() - 1);
            }
        }

        // calculate the ins info firstly
        insStrapdownMechanization(dt, sampleDataArray);
        // remove the negative position action
        endIndex = 0;
        index = 0;

        for (auto p : sampleDataArray)
        {
            SampleData* val = p.get();
            int lastIndex = index - 1;

            if (lastIndex > 0 && val->fPosN < 0)
            {
                endIndex = index;
                break;
            }

            index ++;
        }

        if (endIndex > 0)
        {
            arraySize = sampleDataArray.size();

            for (int i = 0; i < arraySize - endIndex; i++)
            {
                sampleDataArray.erase(sampleDataArray.end() - 1);
            }
        }
    }
    else
    {
        // forehand action
        fOmegaFirst = sampleDataArray.at(0)->fOmegaB[CHZ];
        fOmegaLast = sampleDataArray.at(sampleDataArray.size() - 1)->fOmegaB[CHZ];

        if (fOmegaMax > 0 && fOmegaMax > fOmegaFirst && fOmegaMax > fOmegaLast)
        {
            fOmegaPeak = fOmegaMax;
            fOmegaLetter = 1;
        }
        else if (fOmegaMin < 0 && fOmegaMin < fOmegaFirst && fOmegaMin < fOmegaLast)
        {
            fOmegaPeak = fOmegaMin;
            fOmegaLetter = -1;
        }
        else
        {
            // abnormal case
        }

        double fGyroLastZ = 0;
        int slop = 0;
        const int START = 0;
        const int UP = 1;
        const int DOWN = 2;
        int condition = START;
        bool errorFlag = false;
        bool endFlag = false;

        index = 0;

        for (auto p : sampleDataArray)
        {
            SampleData* val = p.get();
            double gyroZ = val->fOmegaB[2] * fOmegaLetter * fScale;

            switch (condition)
            {
            case START:
                if (gyroZ > 10)
                {
                    condition = UP;
                    startIndex = index;
                }

                break;

            case UP:
                if (slop == 0)
                {
                    if (gyroZ < fGyroLastZ)
                    {
                        // it will happen: gyro z may decrease firstly and then increase.
                    }
                    else
                    {
                        slop = 1;
                    }
                }
                else
                {
                    if (gyroZ < fGyroLastZ)
                    {
                        // peak
                        if (fGyroLastZ > 0.9 * fOmegaPeak * fOmegaLetter * fScale)
                        {
                            // true peak
                            slop = -1;
                            condition = DOWN;
                        }
                        else
                        {
                            // normal case
                            slop = 1;
                        }
                    }
                }

                break;

            case DOWN:
                if (gyroZ > fGyroLastZ)
                {
                    // trough (never happen)
                    slop = 1;
                }
                else
                {
                    // normal case
                    slop = -1;

                    if (gyroZ < 0.5 * fOmegaPeak * fScale * fOmegaLetter)
                    {
                        endIndex = index;
                        endFlag = true;
                    }
                }

                break;
            }

            index ++;
            fGyroLastZ = gyroZ;

            if (errorFlag || endFlag)
            {
                break;
            }
        }

        if (errorFlag || condition != DOWN)
        {
            uActionComplete = false;
            sampleDataArray.clear();

            return;
        }

        // sample data array must including the max and min liner acc
        if (startIndex > fAccMaxIndex)
        {
            startIndex = fAccMaxIndex;
        }

        if (endIndex < fAccMinIndex)
        {
            endIndex = fAccMinIndex;
        }

        // refine the sample data array
        if (startIndex > 0)
        {
            for (int i = 0; i < startIndex; i++)
            {
                sampleDataArray.erase(sampleDataArray.begin());
                endIndex--;
            }
        }

        if (endIndex > 0)
        {
            arraySize = sampleDataArray.size();

            for (int i = 0; i < arraySize - endIndex - 1; i++)
            {
                sampleDataArray.erase(sampleDataArray.end() - 1);
            }
        }

        // calculate the ins info firstly
        insStrapdownMechanization(dt, sampleDataArray);
        // remove the backward action
        endIndex = 0;
        index = 0;

        for (auto p : sampleDataArray)
        {
            SampleData* val = p.get();
            int lastIndex = index - 1;

            if (lastIndex > 0 && val->fPosN < sampleDataArray.at(lastIndex)->fPosN)
            {
                endIndex = index;
                break;
            }

            index ++;
        }

        if (endIndex > 0)
        {
            arraySize = sampleDataArray.size();

            for (int i = 0; i < arraySize - endIndex; i++)
            {
                sampleDataArray.erase(sampleDataArray.end() - 1);
            }
        }
    }

    // check action time and action interval time
    double actionSustainedTime = sampleDataArray.size() * dt;
    double actionIntervalTime = (sampleDataArray.at(0)->uTime - iActionEndTimeLast) * dt;

    if (actionIntervalTime < 0)
    {
        actionIntervalTime = 1.0;
    }

    if (actionSustainedTime > 0.6 || actionSustainedTime < 0.1 || actionIntervalTime < 0.2)
    {
        // abnormal case:
        // 1. action sustained time < 0.1s
        // 2. action sustained time > 0.6s
        // 3. action interval time < 0.2s
        uActionComplete = false;
        sampleDataArray.clear();

        return;
    }

    // record the last action end time
    iActionEndTimeLast = sampleDataArray.at(sampleDataArray.size() - 1)->uTime;
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

void SensorFusion::dcm2euler(double cbn[][3], double euler[])
{
    euler[0] = atan2(cbn[1][0], cbn[0][0]);
    euler[1] = asin(-cbn[2][0]);
    euler[2] = atan2(cbn[2][1], cbn[2][2]);
}

void SensorFusion::qNorm(double fq[])
{
    double fnorm = 0;

    fnorm = sqrt(fq[0] * fq[0] + fq[1] * fq[1] + fq[2] * fq[2] + fq[3] * fq[3]);
    fnorm = 1.0 / fnorm;
    fq[0] *= fnorm;
    fq[1] *= fnorm;
    fq[2] *= fnorm;
    fq[3] *= fnorm;
}

void SensorFusion::q2dcm(double q[], double cbn[][3])
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

