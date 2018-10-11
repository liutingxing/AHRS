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
    magCalibrationInit();
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
    // mag buffer update
#if MAG_SUPPORT
    magBufferUpdate(fMagRaw, mag, time);

    if (time % 10 == 0) // 1Hz calibration frequency
    {
        magCalibrationExec();
    }

#endif

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

    fAlignAccArray.push_back(shared_ptr<double[]> (new double[3] {acc[0], acc[1], acc[2]}, [](double * p)
    {
        delete[] p;
    }));
    fAlignGyroArray.push_back(shared_ptr<double[]> (new double[3] {gyro[0], gyro[1], gyro[2]}, [](double * p)
    {
        delete[] p;
    }));
#if MAG_SUPPORT
    fAlignMagArray.push_back(shared_ptr<double[]> (new double[3] {mag[0], mag[1], mag[2]}, [](double * p)
    {
        delete[] p;
    }));
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
    if (iMagBufferCount > 150)
    {
        isolver = 4;
        calibration4INV();
    }

    if (ftrFitErrorpc <= MAGFITERROR && ftrB > 10 && ftrB < 300)
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
    temp = temp.inverse();
    for (i = 0; i < 4; i++)
    {
        for (j = i; j < 4; j++)
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
        ftrV[l] = (float)(ftrV[l] * DEFAULTB + iOffset[l] * 1.0 * fuTPerCount);
    }

    // correct the geomagnetic field strength B to correct scaling (result in uT)
    ftrB *= DEFAULTB;
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

