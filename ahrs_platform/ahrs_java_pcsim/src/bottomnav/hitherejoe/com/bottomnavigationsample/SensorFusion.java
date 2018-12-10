package bottomnav.hitherejoe.com.bottomnavigationsample;

/**
 * Created by Anakin on 2017/7/11.
 */

import java.util.ArrayList;
import java.util.Arrays;

import Jama.Matrix;

public class SensorFusion {
    public int           uTime;
    private double       fPsiPl;
    private double       fThePl;
    private double       fPhiPl;
    private double       fPsiPlPlat;
    private double       fThePlPlat;
    private double       fPhiPlPlat;
    private double[][]   fCnb = new double[3][3];
    private double[][]   fCbn = new double[3][3];
    private double[][]   fCnp = new double[3][3];
    private double[]     fqPl = new double[4];
    private double[]     fqPlPlat = new double[4];       // MAG_SUPPORT = 0: fqPlPlat = fqPl
    private double[][]   fCbnPlat = new double[3][3];    // MAG_SUPPORT = 0: fCbnPlat = fCbn
    private double[]    fGyroBias = new double[3];
    private double[]    fAccBias = new double[3];
    private double[]    fMagBias = new double[3];
    private double      fLinerAccN;
    private double      fLinerAccE;
    private double      fLinerAccD;
    private double       fVelN;
    private double       fVelE;
    private double       fVelD;
    private double       fPosN;
    private double       fPosE;
    private double       fPosD;
    private double[]     fOmegaB;
    private double[]     fAccelerate;
    private double[]     fMagnetic;
    private double       fAudio;

    private double       fB;
    private double       ftrB;
    private double       ftrFitErrorpc;
    private double       fFitErrorpc;
    private boolean      iValidMagCal;
    private double[]     ftrV = new double[3];
    private double[]     fV = new double[3];

    private int         uStaticFlag;
    private boolean     uAlignFlag;
    private boolean     uKalmanFusionFlag;
    private boolean     uMechanizationFlag;
    private boolean     uActionStartFlag;
    private boolean     uActionEndFlag;
    public  boolean     uActionComplete;

    private final static int MAG_SUPPORT = 1;
    private final static int ALIGN_NUM = 100;
    private int CalibrationProgress = 0;
    private ArrayList<double[]> fAlignGyroArray = new ArrayList<double[]>(ALIGN_NUM);
    private ArrayList<double[]> fAlignAccArray = new ArrayList<double[]>(ALIGN_NUM);
    private ArrayList<double[]> fAlignMagArray = new ArrayList<double[]>(ALIGN_NUM);
    private ArrayList<SampleData> cSampleDataArray = new ArrayList<SampleData>(20);
    private final static float MAGSENSITIVE =  0.01F;
    private final static int MAGBUFFSIZEX =  14;
    private final static int MAGBUFFSIZEY =  MAGBUFFSIZEX * 2;
    private final static int MAXMEASUREMENTS = 240;
    private final static int MESHDELTAUT = 5;
    private final static float MAGFITERROR = 10;
    private final static int CHX = 0;
    private final static int CHY = 1;
    private final static int CHZ = 2;
    private float fuTPerCount;
    private float fCountsPeruT;
    private int[][][] iMagRawBuffer = new int[3][MAGBUFFSIZEX][MAGBUFFSIZEY];
    private int[][] Index = new int[MAGBUFFSIZEX][MAGBUFFSIZEY];
    private int[] Tanarray = new int[MAGBUFFSIZEX - 1];
    private int iMagBufferCount = 0;

    public final static double GRAVITY = 9.80665;
    public final static double SAMPLE_RATE = 100;

    private final static int STATE_NUM = 9;
    private int uKfCount = 1;

    private final static int Calibration = 0;
    private final static int Alignment = 1;
    private final static int Fusion = 2;
    private int iStatus = Calibration;

    private final static int Peace = 0;
    private final static int Step1 = 1;
    private final static int Step2 = 2;
    private final static int Step3 = 3;
    private int iCurveCondition = Peace;
    private double fLinerAccXLast = 0.0;
    private double actionTime = 0.0;
    private double downTime = 0.0;
    private double peakValue = 0.0;

    private double fPlatformOmegaMaxZ = 0.0;
    private double fPlatformOmegaMinZ = 0.0;
    private double fRangeMax = 0.0;
    private double fVelocityMax = 0.0;
    private double fAudioMax = 0.0;
    private int strikeIndex = 0;

    private SensorKalman sensorKalman;
    private StringBuffer sAttitude;
    public TrainData trainData;

    public SensorFusion()
    {
        uTime = 0;
        fPsiPl = 0;
        fThePl = 0;
        fPhiPl = 0;
        euler2q(fqPl, fPsiPl, fThePl, fPhiPl);
        euler2dcm(fCbn, fPsiPl, fThePl, fPhiPl);
        Matrix temp = new Matrix(fCbn);
        fCnb = temp.transpose().getArray();
        Arrays.fill(fGyroBias, 0);
        Arrays.fill(fAccBias, 0);
        Arrays.fill(fMagBias, 0);
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
        sAttitude = new StringBuffer();
        sensorKalman = new SensorKalman(STATE_NUM);
        trainData = new TrainData();
        iStatus = Calibration;
        CalibrationProgress = 0;
        magCalibrationInit();
    }

    public String sensorFusionExec(int time, double[] gyro, double[] acc, double[] mag, double audio)
    {
        double dt = 1.0 / SAMPLE_RATE;
        uTime = time;
        fOmegaB = gyro;
        fAccelerate = acc;
        fMagnetic = mag;
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
        double[] fGyroRaw = new double[]{gyro[0], gyro[1], gyro[2]};
        double[] fAccRaw = new double[]{acc[0], acc[1], acc[2]};
        double[] fMagRaw = new double[]{mag[0], mag[1], mag[2]};
        sensorDataCorrection(gyro, acc, mag);

        // static detection
        staticDetectUpdate(fGyroRaw, fAccRaw, fMagRaw);
        if (time % 100 == 0) // 1Hz static check frequency
        {
            uStaticFlag = staticDetectCheck();
        }

        // mag buffer update
        if (MAG_SUPPORT == 1) {
            if (uStaticFlag == 0)
            {
                magBufferUpdate(fMagRaw, mag, time);
            }
            if (time % 100 == 0) // 1Hz calibration frequency
            {
                magCalibrationExec();
            }
        }

        // mag calibration process
        if (MAG_SUPPORT == 1)
        {
            if (iValidMagCal == true)
            {
                iStatus = Alignment;
                CalibrationProgress = 100;
            }
            else
            {
                // mag calibration process not execute
                CalibrationProgress = (int)(iMagBufferCount * 95 / MAXMEASUREMENTS );
            }
        }
        else
        {
            iStatus = Alignment;
        }

        if (uAlignFlag == false) {
            if (uStaticFlag == 1 && iStatus == Alignment)
            {
                // initial alignment
                if (sensorAlignment(fAlignAccArray, fAlignMagArray) == true) {
                    gyroCalibration(fAlignGyroArray);
                    uAlignFlag = true;
                    iStatus = Fusion;
                }
            }

            return null;
        }

        // AHRS/INS process
        sAttitude = new StringBuffer();

        if (uStaticFlag == 1) {
            gyroCalibration(fAlignGyroArray);
        }

        /*// quaternion integration
        quaternionIntegration(dt, gyro);

        // kalman filter fusion
        if (uKalmanFusionFlag == true)
        {
        	Matrix Acc = new Matrix(acc, 3);
        	Matrix Cbn = new Matrix(fCbn);
            double[] G = {0, 0, SensorFusion.GRAVITY};
            Matrix Gvector = new Matrix(G, 3);
            Matrix gEstimate;
            Matrix res;

        	gEstimate = Cbn.times(Acc.times(-1));
        	res = Gvector.minus(gEstimate);

        	// check if measurement is reasonable
        	if (res.norm2()< SensorFusion.GRAVITY * 1.2)
        	{
        		sensorKalman.sensorFusionKalman(dt*uKfCount, acc, fCbn);
                // error correction
                errCorrection(sensorKalman);
                uKfCount = 1;
        	}
        	else
        	{
        		uKfCount++;
        	}
        }
        else
        {
        	uKfCount++;
        }*/
        // quaternion integration for attitude and heading
        if (uKalmanFusionFlag == true)
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
            SampleData sampleData = new SampleData();

            // ins mechanization
            insStrapdownMechanization(dt, acc);

            // copy sample data into array list
            copyInSampleData(this, sampleData);
            cSampleDataArray.add(sampleData);
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
        }

        sAttitude.append(String.valueOf(uTime));
        sAttitude.append(" ");
        sAttitude.append(String.valueOf(0));
        sAttitude.append(" ");
        sAttitude.append(String.valueOf(0));
        sAttitude.append(" ");
        sAttitude.append(String.valueOf(0));
        sAttitude.append(" ");
        sAttitude.append(String.valueOf(fqPlPlat[0]));
        sAttitude.append(" ");
        sAttitude.append(String.valueOf(-fqPlPlat[1]));
        sAttitude.append(" ");
        sAttitude.append(String.valueOf(-fqPlPlat[2]));
        sAttitude.append(" ");
        sAttitude.append(String.valueOf(fqPlPlat[3]));
        sAttitude.append(" ");
        sAttitude.append(String.valueOf(fLinerAccXLast));
        sAttitude.append(" ");
        sAttitude.append("x");

        return String.valueOf(sAttitude);
    }

    private void platformDataProcess()
    {
        int i = 0;
        int j = 0;

        if (MAG_SUPPORT == 0)
        {
            for (i = 0; i < 4; i++)
            {
                fqPlPlat[i] = fqPl[i];
            }

            for (i = 0; i < 3; i++)
            {
                for (j = 0; j < 3; j++)
                {
                    fCbnPlat[i][j] = fCbn[i][j];
                }
            }
        }
        else
        {
            Matrix cbn = new Matrix(fCbn);
            Matrix cnp = new Matrix(fCnp);
            Matrix cbnPlatform = cnp.times(cbn);
            fCbnPlat = cbnPlatform.getArray();

            double[] euler = dcm2euler(fCbnPlat);
            fPsiPlPlat = euler[0];
            fThePlPlat = euler[1];
            fPhiPlPlat = euler[2];
            euler2q(fqPlPlat, fPsiPlPlat, fThePlPlat, fPhiPlPlat);
        }
    }

    private int processSampleData(ArrayList<SampleData> sampleDataArray, TrainData data)
    {
        int typeIndex = -1;
        int count = 0;
        StringBuffer trajectory = new StringBuffer();
        SampleData value;

        for(SampleData val:sampleDataArray)
        {
            int i = 0;
            double deltaN = 0;
            double deltaE = 0;
            double deltaD = 0;
            SampleData valLast;

            // platform omega
            if (val.fOmegaN[2] > fPlatformOmegaMaxZ)
            {
                fPlatformOmegaMaxZ = val.fOmegaN[2];
            }

            if (val.fOmegaN[2]  < fPlatformOmegaMinZ)
            {
                fPlatformOmegaMinZ = val.fOmegaN[2];
            }

            // max velocity
            if (val.fVel > fVelocityMax)
            {
                fVelocityMax = val.fVel;
            }

            // range
            i = sampleDataArray.indexOf(val);
            if (i > 0)
            {
                valLast = sampleDataArray.get(i - 1);
                deltaN = val.fPosN - valLast.fPosN;
                deltaE = val.fPosE - valLast.fPosE;
                deltaD = val.fPosD - valLast.fPosD;
            }
            fRangeMax += Math.sqrt(deltaN*deltaN + deltaE*deltaE + deltaD*deltaD);

            // max audio for strike timing
            if (val.fAudio > fAudioMax)
            {
                fAudioMax = val.fAudio;
                strikeIndex = sampleDataArray.indexOf(val);
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

        for(SampleData val:sampleDataArray)
        {
            // trajectory
            trajectory.append(String.valueOf(val.uTime));
            trajectory.append(" ");
            trajectory.append(String.valueOf(val.fPosN));
            trajectory.append(" ");
            trajectory.append(String.valueOf(-val.fPosD));
            trajectory.append(" ");
            trajectory.append(String.valueOf(-val.fPosE));
            trajectory.append(" ");
            trajectory.append(String.valueOf(val.fqPlPlat[0]));
            trajectory.append(" ");
            trajectory.append(String.valueOf(-val.fqPlPlat[1]));
            trajectory.append(" ");
            trajectory.append(String.valueOf(-val.fqPlPlat[2]));
            trajectory.append(" ");
            trajectory.append(String.valueOf(val.fqPlPlat[3]));
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
        trajectory.deleteCharAt(trajectory.length() - 1);

        // update train data
        data.bValid = true;
        data.uActionCount ++;
        data.fRangeMax = fRangeMax;
        data.fVelocityMax = fVelocityMax;
        data.fStrikeAudio = fAudioMax;
        if (strikeIndex != -1)
        {
            value = sampleDataArray.get(strikeIndex);
            data.fVelocityStrike = Math.sqrt(value.fVelN*value.fVelN + value.fVelE*value.fVelE + value.fVelD*value.fVelD);
            if (data.fVelocityStrike > 9)
            {
                data.uStrikePower = 100;
            }
            else if (data.fVelocityStrike > 8)
            {
                data.uStrikePower = (int) ((data.fVelocityStrike - 8) * 10.0 + 90);
            }
            else if (data.fVelocityStrike > 7)
            {
                data.uStrikePower = (int) ((data.fVelocityStrike - 7) * 5.0 + 85);
            }
            else if (data.fVelocityStrike > 6)
            {
                data.uStrikePower = (int) ((data.fVelocityStrike - 6) * 5.0 + 80);
            }
            else if (data.fVelocityStrike > 5)
            {
                data.uStrikePower = (int) ((data.fVelocityStrike - 5) * 15.0 + 65);
            }
            else if (data.fVelocityStrike > 4)
            {
                data.uStrikePower = (int) ((data.fVelocityStrike - 4) * 25.0 + 40);
            }
            else
            {
                data.uStrikePower = (int) (data.fVelocityStrike * 10);
            }
        }
        else
        {
            data.fVelocityStrike = 0;
            data.uStrikePower = 0;
        }


        if (Math.abs(fPlatformOmegaMaxZ) > Math.abs(fPlatformOmegaMinZ))
        {
            data.sActionType = "backhand";
        }
        else
        {
            data.sActionType = "forehand";
        }

        // replace the type character in trajectory
        while ((typeIndex = trajectory.indexOf("x")) != -1)
        {
            trajectory.replace(typeIndex, typeIndex + 1, data.sActionType.substring(0,1));
        }
        data.sTrajectory = String.valueOf(trajectory);

        // update audio index
        updateAudioInfo(data);

        return 0;
    }

    private int updateAudioInfo(TrainData data)
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
        else if ((data.fVelocityMax - data.fVelocityStrike) < data.fVelocityMax*0.08)
        {
            data.uAudioType = 4;
        }
        else
        {
            data.uAudioType = 0;
        }

        return 0;
    }

    private int copyInSampleData(SensorFusion src, SampleData dst)
    {
        int i;

        dst.uTime = src.uTime;
        dst.fPsiPlPlat = src.fPsiPlPlat;
        dst.fThePlPlat = src.fThePlPlat;
        dst.fPhiPlPlat = src.fPhiPlPlat;
        for (i = 0; i < src.fCbnPlat.length; i++)
        {
            dst.fCbnPlat[i] = Arrays.copyOf(src.fCbnPlat[i], src.fCbnPlat[i].length);
        }
        dst.fqPlPlat = Arrays.copyOf(src.fqPlPlat, src.fqPlPlat.length);
        dst.fLinerAccN = src.fLinerAccN;
        dst.fLinerAccE = src.fLinerAccE;
        dst.fLinerAccD = src.fLinerAccD;
        dst.fVelN = src.fVelN;
        dst.fVelE = src.fVelE;
        dst.fVelD = src.fVelD;
        dst.fPosN = src.fPosN;
        dst.fPosE = src.fPosE;
        dst.fPosD = src.fPosD;
        dst.fOmegaB = Arrays.copyOf(src.fOmegaB, src.fOmegaB.length);
        dst.fAccelerate = Arrays.copyOf(src.fAccelerate, src.fAccelerate.length);
        dst.fMagnetic = Arrays.copyOf(src.fMagnetic, src.fMagnetic.length);
        dst.fAudio = src.fAudio;
        for (i = 0; i < 3; i++)
        {
            dst.fOmegaN[i] = src.fCbnPlat[i][0] * src.fOmegaB[0] + src.fCbnPlat[i][1] * src.fOmegaB[1] + src.fCbnPlat[i][2] * src.fOmegaB[2];
        }
        dst.fVel = Math.sqrt(src.fVelN*src.fVelN +  src.fVelE*src.fVelE + src.fVelD*src.fVelD);

        return 0;
    }

    private void ahrsProcess(double dt, double[] gyro, double[] acc, double[] mag)
    {
        int i = 0;
        double accNorm = 0;
        double[] qDot = new double[]{0, 0, 0, 0};
        double[] qDotError = new double[]{0, 0, 0, 0};
        double gyroMeasError = 10 * Math.PI / 180; // gyroscope measurement error in rad/s (shown as 10 deg/s)
        double beta = Math.sqrt(3.0 / 4.0) * gyroMeasError;

        qDot[0] = -(gyro[0] * fqPl[1] + gyro[1] * fqPl[2] + gyro[2] * fqPl[3]) / 2.0;
        qDot[1] =  (gyro[0] * fqPl[0] + gyro[2] * fqPl[2] - gyro[1] * fqPl[3]) / 2.0;
        qDot[2] =  (gyro[1] * fqPl[0] - gyro[2] * fqPl[1] + gyro[0] * fqPl[3]) / 2.0;
        qDot[3] =  (gyro[2] * fqPl[0] + gyro[1] * fqPl[1] - gyro[0] * fqPl[2]) / 2.0;

        accNorm = Math.sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
        if (accNorm < 12.0) {
            // execute the acc aid process
            double diff = 0;
            double[] gEstimate = new double[3];
            Matrix F = new Matrix(3, 1);
            Matrix J = new Matrix(3, 4);
            Matrix step = new Matrix(4, 1);

            gEstimate[0] = -acc[0] / accNorm;
            gEstimate[1] = -acc[1] / accNorm;
            gEstimate[2] = -acc[2] / accNorm;

            F.set(0, 0, 2 * (fqPl[1] * fqPl[3] - fqPl[0] * fqPl[2]) - gEstimate[0]);
            F.set(1, 0, 2 * (fqPl[0] * fqPl[1] + fqPl[2] * fqPl[3]) - gEstimate[1]);
            F.set(2, 0, 2 * (0.5 - fqPl[1] * fqPl[1] - fqPl[2] * fqPl[2]) - gEstimate[2]);

            J.set(0, 0, -2 * fqPl[2]);
            J.set(0, 1, 2 * fqPl[3]);
            J.set(0, 2, -2 * fqPl[0]);
            J.set(0, 3, 2 * fqPl[1]);

            J.set(1, 0, 2 * fqPl[1]);
            J.set(1, 1, 2 * fqPl[0]);
            J.set(1, 2, 2 * fqPl[3]);
            J.set(1, 3, 2 * fqPl[2]);

            J.set(2, 0, 0);
            J.set(2, 1, -4 * fqPl[1]);
            J.set(2, 2, -4 * fqPl[2]);
            J.set(2, 3, 0);

            step = J.transpose().times(F);
            qDotError[0] += step.get(0, 0);
            qDotError[1] += step.get(1, 0);
            qDotError[2] += step.get(2, 0);
            qDotError[3] += step.get(3, 0);

            diff = F.norm2();
            if (diff < 0.1) {
                gyroMeasError = 3 * Math.PI / 180;
                beta = Math.sqrt(3.0 / 4.0) * gyroMeasError;
            }
            else
            {
                gyroMeasError = 10 * Math.PI / 180;
                beta = Math.sqrt(3.0 / 4.0) * gyroMeasError;
            }
        }

        if (MAG_SUPPORT == 1)
        {
            double magNorm = Math.sqrt(mag[0]*mag[0] + mag[1]*mag[1] + mag[2]*mag[2]);
            if (magNorm > fB * 0.8 && magNorm < fB * 1.2)
            {
                // execute the acc aid process
                double diff = 0;
                double[] mEstimate = new double[3];
                double[] b = new double[4];
                Matrix F = new Matrix(3, 1);
                Matrix J = new Matrix(3, 4);
                Matrix h = new Matrix(3, 1);
                Matrix cbn = new Matrix(fCbn);
                Matrix m = new Matrix(3, 1);
                Matrix step = new Matrix(4, 1);

                mEstimate[0] = mag[0] / magNorm;
                mEstimate[1] = mag[1] / magNorm;
                mEstimate[2] = mag[2] / magNorm;

                m.set(0, 0, mEstimate[0]);
                m.set(1, 0, mEstimate[1]);
                m.set(2, 0, mEstimate[2]);

                h = cbn.times(m);
                b[0] = 0;
                b[1] = Math.sqrt(h.get(0, 0)*h.get(0, 0) + h.get(1, 0)*h.get(1, 0));
                b[2] = 0;
                b[3] = h.get(2, 0);

                F.set(0, 0, 2*b[1]*(0.5 - fqPl[2]*fqPl[2] - fqPl[3]*fqPl[3]) + 2*b[3]*(fqPl[1]*fqPl[3] - fqPl[0]*fqPl[2]) - mEstimate[0]);
                F.set(1, 0, 2*b[1]*(fqPl[1]*fqPl[2] - fqPl[0]*fqPl[3]) + 2*b[3]*(fqPl[0]*fqPl[1] + fqPl[2]*fqPl[3]) - mEstimate[1]);
                F.set(2, 0, 2*b[1]*(fqPl[0]*fqPl[2] + fqPl[1]*fqPl[3]) + 2*b[3]*(0.5 - fqPl[1]*fqPl[1] - fqPl[2]*fqPl[2]) - mEstimate[2]);

                J.set(0, 0, -2 * b[3] * fqPl[2]);
                J.set(0, 1, 2 * b[3] * fqPl[3]);
                J.set(0, 2, -4 * b[1] * fqPl[2] - 2 * b[3] * fqPl[0]);
                J.set(0, 3, -4 * b[1] * fqPl[3] + 2 * b[3] * fqPl[1]);

                J.set(1, 0, -2 * b[1] * fqPl[3] + 2 * b[3] * fqPl[1]);
                J.set(1, 1, 2 * b[1] * fqPl[2] + 2 * b[3] * fqPl[0]);
                J.set(1, 2, 2 * b[1] * fqPl[1] + 2 * b[3] * fqPl[3]);
                J.set(1, 3, -2 * b[1] * fqPl[0] + 2 * b[3] * fqPl[2]);

                J.set(2, 0, 2 * b[1] * fqPl[2]);
                J.set(2, 1, 2 * b[1] * fqPl[3] - 4 * b[3] * fqPl[1]);
                J.set(2, 2, 2 * b[1] * fqPl[0] - 4 * b[3] * fqPl[2]);
                J.set(2, 3, 2 * b[1] * fqPl[1]);

                diff = F.norm2();
                step = J.transpose().times(F);
                qDotError[0] += step.get(0, 0);
                qDotError[1] += step.get(1, 0);
                qDotError[2] += step.get(2, 0);
                qDotError[3] += step.get(3, 0);
                if (diff < 0.1) {
                    gyroMeasError = 10 * Math.PI / 180;
                    beta = Math.sqrt(3.0 / 4.0) * gyroMeasError;
                }
                else
                {
                    gyroMeasError = 40 * Math.PI / 180;
                    beta = Math.sqrt(3.0 / 4.0) * gyroMeasError;
                }
            }
        }

        double qDotErrorNorm = Math.sqrt(qDotError[0] * qDotError[0] + qDotError[1] * qDotError[1] + qDotError[2] * qDotError[2] + qDotError[3] * qDotError[3]);
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
        double[] euler = dcm2euler(fCbn);
        fPsiPl = euler[0];
        fThePl = euler[1];
        fPhiPl = euler[2];
        Matrix temp = new Matrix(fCbn);
        fCnb = temp.transpose().getArray();
    }

    private void systemConditionSet()
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

            // clear ins data
            fLinerAccN = 0;
            fLinerAccE = 0;
            fLinerAccD = 0;
            fVelN = 0;
            fVelE = 0;
            fVelD = 0;
            fPosN = 0;
            fPosE = 0;
            fPosD = 0;

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

            // clear ins data
            fLinerAccN = 0;
            fLinerAccE = 0;
            fLinerAccD = 0;
            fVelN = 0;
            fVelE = 0;
            fVelD = 0;
            fPosN = 0;
            fPosE = 0;
            fPosD = 0;
        }
    }

    private void insStrapdownMechanization(double dt, double[] acc)
    {
        int i;
        double[] linerAccIBP = new double[]{0, 0, 0};
        double[] velIBP = new double[]{0, 0, 0};
        double[] linerAccAve = new double[]{0, 0, 0};
        double[] velAve = new double[]{0, 0, 0};
        double deltaN = 0.0;
        double deltaE = 0.0;
        double deltaD = 0.0;
        double velCurrent = 0.0;

        for (i = 0; i < 3; i++)
        {
            linerAccIBP[i] = acc[0]*fCbnPlat[i][0] + acc[1]*fCbnPlat[i][1] + acc[2]*fCbnPlat[i][2];
        }
        linerAccIBP[2] += GRAVITY;

        // static constrain
        for (i = 0; i < 3; i++)
        {
            if (Math.abs(linerAccIBP[i]) < 1)
            {
                linerAccIBP[i] = 0;
            }
        }

        linerAccAve[0] = (linerAccIBP[0] + fLinerAccN) / 2.0;
        linerAccAve[1] = (linerAccIBP[1] + fLinerAccE) / 2.0;
        linerAccAve[2] = (linerAccIBP[2] + fLinerAccD) / 2.0;
        velIBP[0] = fVelN + linerAccAve[0] * dt;
        velIBP[1] = fVelE + linerAccAve[1] * dt;
        velIBP[2] = fVelD + linerAccAve[2] * dt;
        velAve[0] = (fVelN + velIBP[0]) / 2.0;
        velAve[1] = (fVelE + velIBP[1]) / 2.0;
        velAve[2] = (fVelD + velIBP[2]) / 2.0;

        fLinerAccN = linerAccIBP[0];
        fLinerAccE = linerAccIBP[1];
        fLinerAccD = linerAccIBP[2];
        fVelN = velIBP[0];
        fVelE = velIBP[1];
        fVelD = velIBP[2];
        deltaN = velAve[0] * dt;
        deltaE = velAve[1] * dt;
        deltaD = velAve[2] * dt;
        fPosN += deltaN;
        fPosE += deltaE;
        fPosD += deltaD;
    }

    private void actionDetect(double dt, double[] gyro, double[] acc)
    {
        int i;
        int slop = 0;
        double[] linerAccIBP = new double[]{0, 0, 0};
        double linerAccX = 0.0;

        // calculate the liner accelerate along the x axis
        for (i = 0; i < 3; i++)
        {
            linerAccIBP[i] = acc[0]*fCbnPlat[i][0] + acc[1]*fCbnPlat[i][1] + acc[2]*fCbnPlat[i][2];
        }
        linerAccIBP[2] += GRAVITY;

        // static constrain
        for (i = 0; i < 3; i++)
        {
            if (Math.abs(linerAccIBP[i]) < 1)
            {
                linerAccIBP[i] = 0;
            }
        }
        linerAccX = linerAccIBP[0];
        switch(iCurveCondition)
        {
            case Peace:
                if (linerAccX > 3){
                    SampleData sampleData = new SampleData();

                    uActionStartFlag = true;
                    iCurveCondition = Step1;
                    actionTime = 0;

                    // copy sample data into array list
                    copyInSampleData(this, sampleData);
                    cSampleDataArray.add(sampleData);
                }
                break;

            case Step1:
                actionTime += dt;
                if (linerAccX > fLinerAccXLast){
                    slop = 1;
                }else{
                    slop = -1;
                    // reach the up peak
                    if (fLinerAccXLast < 8){
                        // false peak
                        iCurveCondition = Peace;
                        uActionStartFlag = false;
                    }else{
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
                if (downTime > 0.3)
                {
                    iCurveCondition = Peace;
                    uActionStartFlag = false;
                }
                else
                {
                    if (linerAccX > fLinerAccXLast){
                        slop = 1;
                        // reach the trough
                        if (fLinerAccXLast > 0.5 * peakValue && peakValue < 30){
                            // maybe there is false peak in the step1
                            if (downTime > 0.05)
                            {
                                // there is a false peak in the step1
                                iCurveCondition = Peace;
                                uActionStartFlag = false;
                            }
                            else
                            {
                                //the following peak is false peak
                            }
                        }
                        else if(fLinerAccXLast > -5){
                            // false trough
                            // no action, because it is normal
                        }
                        else{
                            iCurveCondition = Step3;
                        }
                    }else{
                        slop = -1;
                    }
                }
                break;

            case Step3:
                actionTime += dt;
                if (linerAccX > fLinerAccXLast){
                    slop = 1;
                }else {
                    slop = -1;
                }
                if (linerAccX > -10 && linerAccX < 10){
                    if (actionTime > 0.1 && actionTime < 0.6)
                    {
                        uActionEndFlag = true;
                    }
                    else
                    {
                        uActionStartFlag = false;
                    }
                    iCurveCondition = Peace;
                }
                break;
        }
        fLinerAccXLast = linerAccX;
    }

    private void errCorrection(SensorKalman kalmanInfo)
    {
        double[][] deltaCbn = new double[3][3];
        Matrix temp;
        int i;

        euler2dcm(deltaCbn, kalmanInfo.x.get(2, 0), kalmanInfo.x.get(1, 0), kalmanInfo.x.get(0, 0));
        temp = new Matrix(deltaCbn);
        temp = temp.times(new Matrix(fCbn));
        fCbn = temp.getArray();
        fCnb = temp.transpose().getArray();
        double[] euler = dcm2euler(fCbn);
        fPsiPl = euler[0];
        fThePl = euler[1];
        fPhiPl = euler[2];
        euler2q(fqPl, fPsiPl, fThePl, fPhiPl);
        qNorm(fqPl);

        for (i = 0; i < 3; i++)
        {
            fGyroBias[i] += kalmanInfo.x.get(3+i, 0);
            fAccBias[i] += kalmanInfo.x.get(6+i, 0);
        }
        kalmanInfo.x = new Matrix(kalmanInfo.stateNum, 1, 0);
    }

    private void quaternionIntegration(double dt, double[] gyro)
    {
        int i = 0;
        double[] fdq = new double[]{0, 0, 0, 0};

        fdq[0] = -(gyro[0] * fqPl[1] + gyro[1] * fqPl[2] + gyro[2] * fqPl[3]) / 2.0;
        fdq[1] =  (gyro[0] * fqPl[0] + gyro[2] * fqPl[2] - gyro[1] * fqPl[3]) / 2.0;
        fdq[2] =  (gyro[1] * fqPl[0] - gyro[2] * fqPl[1] + gyro[0] * fqPl[3]) / 2.0;
        fdq[3] =  (gyro[2] * fqPl[0] + gyro[1] * fqPl[1] - gyro[0] * fqPl[2]) / 2.0;

        for (i = 0; i < 4; i++)
        {
            fqPl[i] += fdq[i] * dt;
        }
        qNorm(fqPl);
        q2dcm(fqPl, fCbn);
        double[] euler = dcm2euler(fCbn);
        fPsiPl = euler[0];
        fThePl = euler[1];
        fPhiPl = euler[2];
        Matrix temp = new Matrix(fCbn);
        fCnb = temp.transpose().getArray();
    }

    private boolean sensorAlignment(ArrayList<double[]> accArray, ArrayList<double[]> magArray)
    {
        if (MAG_SUPPORT == 1)
        {
            int i = 0;
            int j = 0;
            int X = 0;
            int Y = 1;
            int Z = 2;
            double ftmp = 0;
            double[] fg = new double[]{0,0,0};
            double[] fm = new double[]{0,0,0};
            double[] fmod = new double[]{0,0,0};
            double[][] fR = new double[3][3];

            for(double[] value:accArray)
            {
                fg[0] -= value[0];
                fg[1] -= value[1];
                fg[2] -= value[2];
            }
            for (i = 0; i < 3; i++)
            {
                fg[i] = fg[i] / accArray.size();
                fR[i][Z] = fg[i];;
            }

            for(double[] value:magArray)
            {
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
                fmod[i] = Math.sqrt(fR[X][i] * fR[X][i] + fR[Y][i] * fR[Y][i] + fR[Z][i] * fR[Z][i]);
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

            Matrix temp = new Matrix(fCnb);
            fCbn = temp.transpose().getArray();
            double[] euler = dcm2euler(fCbn);
            fPsiPl = euler[0];
            fThePl = euler[1];
            fPhiPl = euler[2];
            euler2q(fqPl, fPsiPl, fThePl, fPhiPl);
            euler2dcm(fCnp, fPsiPl, 0, 0);
            temp = new Matrix(fCnp);
            fCnp = temp.transpose().getArray();

            return true;
        }
        else
        {
            int i = 0;
            double fmodGxyz = 0;
            double fmodGyz = 0;
            double frecipmodGxyz = 0;
            double ftmp = 0;
            double[] fg = new double[]{0,0,0};

            // tilt alignment
            for(double[] value:accArray)
            {
                fg[0] -= value[0];
                fg[1] -= value[1];
                fg[2] -= value[2];
            }
            for (i = 0; i < 3; i++)
            {
                fg[i] = fg[i] / accArray.size();
            }

/*        fPsiPl = 0;
        fThePl = -Math.asin(fg[0] / SensorFusion.GRAVITY);
        fPhiPl = Math.atan2(fg[1] / SensorFusion.GRAVITY, fg[2] / SensorFusion.GRAVITY);*/

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
            fmodGyz = Math.sqrt(fmodGyz);
            fmodGxyz = Math.sqrt(fmodGxyz);
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

            Matrix temp = new Matrix(fCnb);
            fCbn = temp.transpose().getArray();
            double[] euler = dcm2euler(fCbn);
            fPsiPl = euler[0];
            fThePl = euler[1];
            fPhiPl = euler[2];
            euler2q(fqPl, fPsiPl, fThePl, fPhiPl);

            return true;
        }
    }

    private void gyroCalibration(ArrayList<double[]> gyroArray)
    {
        int i;
        double[] bias = new double[]{0, 0, 0};

        // gyro calibration
        for(double[] value:gyroArray)
        {
            bias[0] += value[0];
            bias[1] += value[1];
            bias[2] += value[2];
        }
        for (i = 0; i < 3; i++)
        {
            bias[i] /= gyroArray.size();
        }

        if (Math.abs(bias[0]) < 0.01 && Math.abs(bias[1]) < 0.01 && Math.abs(bias[2]) < 0.01)
        {
            for (i = 0; i < 3; i++)
            {
                fGyroBias[i] = bias[i] / gyroArray.size();
            }
        }
    }

    private int staticDetectUpdate(double[] gyro, double[] acc, double[] mag)
    {
        if (fAlignGyroArray.size() < ALIGN_NUM)
        {
            fAlignAccArray.add(new double[]{acc[0], acc[1], acc[2]});
            fAlignGyroArray.add(new double[]{gyro[0], gyro[1], gyro[2]});
            if (MAG_SUPPORT == 1  && mag[0] != 0 && mag[1] != 0 && mag[2] != 0)
            {
                fAlignMagArray.add(new double[]{mag[0],mag[1],mag[2]});
            }
        }
        else
        {
            fAlignAccArray.remove(0);
            fAlignGyroArray.remove(0);
            fAlignAccArray.add(new double[]{acc[0], acc[1], acc[2]});
            fAlignGyroArray.add(new double[]{gyro[0], gyro[1], gyro[2]});
            if (MAG_SUPPORT == 1  && mag[0] != 0 && mag[1] != 0 && mag[2] != 0)
            {
                fAlignMagArray.remove(0);
                fAlignMagArray.add(new double[]{mag[0],mag[1],mag[2]});
            }
        }

        return 0;
    }

    private int staticDetectCheck()
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

    private void sensorDataCorrection(double[] gyro, double[] acc, double[] mag)
    {
        int i = 0;

        for (i = 0; i < 3; i++)
        {
            gyro[i] = gyro[i] - fGyroBias[i];
            acc[i] = acc[i] - fAccBias[i];
            if (MAG_SUPPORT == 1 && mag[0] != 0 && mag[1] != 0 && mag[2] != 0)
            {
                mag[i] = mag[i] - fMagBias[i];
            }
        }
    }

    private double stdCal(ArrayList<double[]> numList)
    {
        double det = 0;
        double value = 0; // the sum of Xi
        double square = 0; // the sum of xi^2
        double sigma = 0; // standard deviation
        double size = numList.size();

        for(double[] val:numList)
        {
            det = Math.sqrt(val[0]*val[0] + val[1]*val[1] + val[2]*val[2]);
            value += det;
            square += det*det;
        }
        sigma = Math.sqrt((square - value * value / size) / (size - 1));

        return sigma;

    }

    private void euler2q(double[] q, double fyaw, double fpitch, double froll)
    {
        q[0] = Math.cos(froll / 2) * Math.cos(fpitch / 2) * Math.cos(fyaw / 2) + Math.sin(froll / 2) * Math.sin(fpitch / 2) * Math.sin(fyaw / 2);
        q[1] = Math.sin(froll / 2) * Math.cos(fpitch / 2) * Math.cos(fyaw / 2) - Math.cos(froll / 2) * Math.sin(fpitch / 2) * Math.sin(fyaw / 2);
        q[2] = Math.cos(froll / 2) * Math.sin(fpitch / 2) * Math.cos(fyaw / 2) + Math.sin(froll / 2) * Math.cos(fpitch / 2) * Math.sin(fyaw / 2);
        q[3] = Math.cos(froll / 2) * Math.cos(fpitch / 2) * Math.sin(fyaw / 2) - Math.sin(froll / 2) * Math.sin(fpitch / 2) * Math.cos(fyaw / 2);
    }

    private void euler2dcm(double[][] cbn, double fyaw, double fpitch, double froll)
    {
        cbn[0][0] = Math.cos(fpitch) * Math.cos(fyaw);
        cbn[0][1] = Math.sin(froll) * Math.sin(fpitch) * Math.cos(fyaw) - Math.cos(froll) * Math.sin(fyaw);
        cbn[0][2] = Math.cos(froll) * Math.sin(fpitch) * Math.cos(fyaw) + Math.sin(froll) * Math.sin(fyaw);

        cbn[1][0] = Math.cos(fpitch) * Math.sin(fyaw);
        cbn[1][1] = Math.sin(froll) * Math.sin(fpitch) * Math.sin(fyaw) + Math.cos(froll) * Math.cos(fyaw);
        cbn[1][2] = Math.cos(froll) * Math.sin(fpitch) * Math.sin(fyaw) - Math.sin(froll) * Math.cos(fyaw);

        cbn[2][0] = -Math.sin(fpitch);
        cbn[2][1] = Math.sin(froll) * Math.cos(fpitch);
        cbn[2][2] = Math.cos(froll) * Math.cos(fpitch);
    }

    private double[] dcm2euler(double[][] cbn)
    {
        double[] euler = new double[3];
        euler[0] = Math.atan2(cbn[1][0], cbn[0][0]);
        euler[1] = Math.asin(-cbn[2][0]);
        euler[2] = Math.atan2(cbn[2][1], cbn[2][2]);

        return euler;
    }

    private void q2dcm(double[] q, double[][] cbn)
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

    private void qNorm(double[] fq)
    {
        double fnorm = 0;

        fnorm = Math.sqrt(fq[0] * fq[0] + fq[1] * fq[1] + fq[2] * fq[2] + fq[3] * fq[3]);
        fnorm = 1.0 / fnorm;
        fq[0] *= fnorm;
        fq[1] *= fnorm;
        fq[2] *= fnorm;
        fq[3] *= fnorm;
    }

    private void magCalibrationInit()
    {
        int i = 0;
        int j = 0;

        // initialize the mag buffer
        iMagBufferCount = 0;
        for (i = 0; i < MAGBUFFSIZEX; i++)
        {
            for (j = 0; j < MAGBUFFSIZEY; j++)
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
        for (i = 0; i < (MAGBUFFSIZEX - 1); i++)
        {
            Tanarray[i] = (int) (100.0F * Math.tan(Math.PI * (-0.5F + (i + 1) * 1.0F / MAGBUFFSIZEX)));
        }
    }

    private int magBufferUpdate(double[] magRaw, double[] magCal, int loopCounter)
    {
        int i = 0;
        int j = 0;
        int k = 0;
        int l = 0;
        int m = 0;
        int[] iMagRaw = new int[3];
        int[] iMagCal = new int[3];
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
        while ((j < (MAGBUFFSIZEX - 1) && (itanj >= Tanarray[j]))) j++;
        while ((k < (MAGBUFFSIZEX - 1) && (itank >= Tanarray[k]))) k++;
        if (iMagCal[CHX] < 0) k += MAGBUFFSIZEX;

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
                idelta += Math.abs(iMagRaw[i] - iMagRawBuffer[i][j][k]);
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
                                idelta += Math.abs(iMagRaw[i] - iMagRawBuffer[i][j][k]);
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

    private int magCalibrationExec()
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

        if (ftrFitErrorpc <= MAGFITERROR && ftrB > 10 && ftrB < 200)
        {
            if (iValidMagCal == false || ftrFitErrorpc <= fFitErrorpc || ftrFitErrorpc <= 5.0F)
            {
                iValidMagCal = true;
                fFitErrorpc = ftrFitErrorpc;
                fB = ftrB;
                for (i = CHX; i <= CHZ; i++) {
                    fMagBias[i] = fV[i] = ftrV[i];
                }
            }
        }

        return isolver;
    }

    private void calibration4INV() {
        // local variables
        double fBs2;                            // fBs[CHX]^2+fBs[CHY]^2+fBs[CHZ]^2
        double fSumBs4;                            // sum of fBs2
        double fscaling;                        // set to FUTPERCOUNT * FMATRIXSCALING
        double fE;                                // error function = r^T.r
        int[] iOffset = new int[3];        // offset to remove large DC hard iron bias in matrix
        int iCount;                                // number of measurements counted
        int ierror;                                // matrix inversion error flag
        int i, j, k, l;                            // loop counters

        double DEFAULTB = 50.0;
        double[] fvecB = new double[4];
        double[] fvecA = new double[6];
        double[][] fmatA = new double[4][4];
        double[][] fmatB = new double[4][4];

        // compute fscaling to reduce multiplications later
        fscaling = fuTPerCount / DEFAULTB;

        // zero fSumBs4=Y^T.Y, fvecB=X^T.Y (4x1) and on and above diagonal elements of fmatA=X^T*X (4x4)
        fSumBs4 = 0.0;
        for (i = 0; i < 4; i++) {
            fvecB[i] = 0.0;
            for (j = i; j < 4; j++) {
                fmatA[i][j] = 0.0;
            }
        }

        // the offsets are guaranteed to be set from the first element but to avoid compiler error
        iOffset[0] = iOffset[1] = iOffset[2] = 0;

        // use entries from magnetic buffer to compute matrices
        iCount = 0;
        for (j = 0; j < MAGBUFFSIZEX; j++) {
            for (k = 0; k < MAGBUFFSIZEY; k++) {
                if (Index[j][k] != -1) {
                    // use first valid magnetic buffer entry as estimate (in counts) for offset
                    if (iCount == 0) {
                        for (l = 0; l <= 2; l++) {
                            iOffset[l] = iMagRawBuffer[l][j][k];
                        }
                    }
                    // store scaled and offset fBs[XYZ] in fvecA[0-2] and fBs[XYZ]^2 in fvecA[3-5]
                    for (l = 0; l <= 2; l++) {
                        fvecA[l] = (iMagRawBuffer[l][j][k] - iOffset[l]) * fscaling;
                        fvecA[l + 3] = fvecA[l] * fvecA[l];
                    }
                    // calculate fBs2 = fBs[CHX]^2 + fBs[CHY]^2 + fBs[CHZ]^2 (scaled uT^2)
                    fBs2 = fvecA[3] + fvecA[4] + fvecA[5];
                    // accumulate fBs^4 over all measurements into fSumBs4=Y^T.Y
                    fSumBs4 += fBs2 * fBs2;
                    // now we have fBs2, accumulate fvecB[0-2] = X^T.Y =sum(fBs2.fBs[XYZ])
                    for (l = 0; l <= 2; l++) {
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
        Matrix temp = new Matrix(fmatB, 4, 4);
        fmatB = temp.inverse().getArray();
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
        ftrB = Math.sqrt(fvecA[3] + ftrV[0] * ftrV[0] + ftrV[1] * ftrV[1] + ftrV[2] * ftrV[2]);
        // calculate the trial fit error (percent) normalized to number of measurements and scaled geomagnetic field strength
        ftrFitErrorpc = Math.sqrt(fE / 300) * 100.0F / (2.0F * ftrB * ftrB);
        // correct the hard iron estimate for FMATRIXSCALING and the offsets applied (result in uT)
        for (l = CHX; l <= CHZ; l++)
        {
            ftrV[l] = (float)(ftrV[l] * DEFAULTB + iOffset[l] * 1.0 * fuTPerCount);
        }
        // correct the geomagnetic field strength B to correct scaling (result in uT)
        ftrB *= DEFAULTB;
    }

    public int getCalibrationProgress()
    {
        return CalibrationProgress;
    }

    public int resetSensorFusion()
    {
        CalibrationProgress = 0;
        uTime = 0;
        fPsiPl = 0;
        fThePl = 0;
        fPhiPl = 0;
        euler2q(fqPl, fPsiPl, fThePl, fPhiPl);
        euler2dcm(fCbn, fPsiPl, fThePl, fPhiPl);
        Matrix temp = new Matrix(fCbn);
        fCnb = temp.transpose().getArray();
        Arrays.fill(fGyroBias, 0);
        Arrays.fill(fAccBias, 0);
        Arrays.fill(fMagBias, 0);
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
        sAttitude = new StringBuffer();
        sensorKalman = new SensorKalman(STATE_NUM);
        trainData = new TrainData();
        iStatus = Calibration;
        magCalibrationInit();

        return 0;
    }
}


