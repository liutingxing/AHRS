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

    private boolean     uStaticFlag;
    private boolean     uAlignFlag;
    private boolean     uKalmanFusionFlag;
    private boolean     uMechanizationFlag;
    private boolean     uActionStartFlag;
    private boolean     uActionEndFlag;
    public  boolean     uActionComplete;

    private final static int MAG_SUPPORT = 1;
    private final static int ALIGN_NUM = 100;
    private ArrayList<double[]> fAlignGyroArray = new ArrayList<double[]>(100);
    private ArrayList<double[]> fAlignAccArray = new ArrayList<double[]>(100);
    private ArrayList<double[]> fAlignMagArray = new ArrayList<double[]>(100);
    private ArrayList<SampleData> cSampleDataArray = new ArrayList<SampleData>(20);

    public final static double GRAVITY = 9.80665;
    public final static double SAMPLE_RATE = 40;

    private final static int STATE_NUM = 9;
    private int uKfCount = 1;

    private final static int Peace = 0;
    private final static int Step1 = 1;
    private final static int Step2 = 2;
    private final static int Step3 = 3;
    private int iCurveCondition = Peace;
    private double fLinerAccXLast = 0.0;
    private double actionTime = 0.0;
    private double downTime = 0.0;

    private double fPlatformOmegaMaxZ = 0.0;
    private double fPlatformOmegaMinZ = 0.0;
    private double fRangeMax = 0.0;
    private double fVelocityMax = 0.0;
    private double fAudioMax = 0.0;
    private int strikeIndex = 0;

	private SensorKalman sensorKalman;
    private StringBuffer sAttitude;
    public TrainData trainData;

    SensorFusion()
    {
        uTime = 0;
        fPsiPl = 0;
        fThePl = 0;
        fPhiPl = 0;
        euler2q(fqPl, fPsiPl, fThePl, fPhiPl);
        euler2dcm(fCbn, fPsiPl, fThePl, fPhiPl);
        Matrix temp = new Matrix(fCbn);
        fCnb = temp.transpose().getArray();
        java.util.Arrays.fill(fGyroBias, 0);
        java.util.Arrays.fill(fAccBias, 0);
        fLinerAccN = 0;
        fLinerAccE = 0;
        fLinerAccD = 0;
        fVelN = 0;
        fVelE = 0;
        fVelD = 0;
        fPosN = 0;
        fPosE = 0;
        fPosD = 0;
        uStaticFlag = false;
        uAlignFlag = false;
        uKalmanFusionFlag = true;
        uMechanizationFlag = false;
        uActionStartFlag = false;
        uActionEndFlag = false;
        uActionComplete = false;
        sAttitude = new StringBuffer();
        sensorKalman = new SensorKalman(STATE_NUM);
        trainData = new TrainData();
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

        // static detection
        uStaticFlag = staticDetect(gyro, acc, mag);

        if (uAlignFlag == false) {
            if (uStaticFlag == true) {
                // initial alignment
                if (sensorAlignment(fAlignAccArray, fAlignMagArray) == true) {
                    gyroCalibration(fAlignGyroArray);
                    fAlignGyroArray.clear();
                    uAlignFlag = true;
                }
            }

            return null;
        }

        // AHRS/INS process
        sAttitude = new StringBuffer();

        if (uStaticFlag == true) {
            gyroCalibration(fAlignGyroArray);
            fAlignGyroArray.clear();
        }

        sensorDataCorrection(gyro, acc);
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
        StringBuffer trajectory = new StringBuffer();
        SampleData value;

        for(SampleData val:sampleDataArray)
        {
            int i = 0;
            double deltaN = 0;
            double deltaE = 0;
            double deltaD = 0;
            SampleData valLast;

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
            trajectory.append("x");
            trajectory.append("\n");

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
        // delete the last enter character
        trajectory.deleteCharAt(trajectory.length() - 1);

        // update train data
        data.bValid = true;
        data.uActionCount ++;
        data.fRangeMax = fRangeMax;
        data.fVelocityMax = fVelocityMax;
        data.fStrikeAudio = fAudioMax;
        value = sampleDataArray.get(strikeIndex);
        data.fVelocityStrike = Math.sqrt(value.fVelN*value.fVelN + value.fVelE*value.fVelE + value.fVelD*value.fVelD);

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
            if (magNorm != 0)
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
                    gyroMeasError = 100 * Math.PI / 180;
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
                if (linerAccX > 1.5){
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
                    if (fLinerAccXLast < 5){
                        // false peak
                        iCurveCondition = Peace;
                        uActionStartFlag = false;
                    }else{
                        iCurveCondition = Step2;
                        downTime = 0;
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
                        if (fLinerAccXLast > -5){
                            // false trough
                            // no action, because it is normal
                        }else{
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
                fm[i] = fm[i] / magArray.size();
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
            fGyroBias[i] += bias[i] / gyroArray.size();
        }
    }

    private boolean staticDetect(double[] gyro, double[] acc, double[] mag)
    {
        double gyro_det = 0;
        double acc_det = 0;
        double gyro_std = 0;
        double acc_std = 0;

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

        if (fAlignGyroArray.size() == ALIGN_NUM && fAlignAccArray.size() == ALIGN_NUM)
        {
            gyro_std = stdCal(fAlignGyroArray);
            acc_std = stdCal(fAlignAccArray);

            if (gyro_std < 0.01 && acc_std < 0.1)
            {
                return true;
            }
        }

        return false;
    }

    private void sensorDataCorrection(double[] gyro, double[] acc)
    {
        int i = 0;

        for (i = 0; i < 3; i++)
        {
            gyro[i] = gyro[i] - fGyroBias[i];
            acc[i] = acc[i] - fAccBias[i];
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
}
