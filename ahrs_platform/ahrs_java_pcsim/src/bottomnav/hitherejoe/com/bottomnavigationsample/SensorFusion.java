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
    private double[]     fOmegaBRaw;
    private double[]     fAccelerateRaw;
    private double[]     fMagneticRaw;
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
    private final static int CALIBRATION_NUM = 500;
    private ArrayList<double[]> fCalibrationMagArray = new ArrayList<double[]>(CALIBRATION_NUM);
    private double       fGeoB;
    private double       fResidual; // unit:%


    public final static double GRAVITY = 9.80665;
    private final static double SAMPLE_RATE = 100;
    private final static double dt = 1.0 / SAMPLE_RATE;

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
    private double fGyroZLast = 0.0;
    private int iSlopGyroLast = 0;
    private boolean bSlopChange = false;
    private double actionTime = 0.0;
    private double downTime = 0.0;
    private double peakValue = 0.0;
    private int iActionEndTimeLast = 0;

    private double fPlatformOmegaMaxZ = 0.0;
    private double fPlatformOmegaMinZ = 0.0;
    private double fRangeMax = 0.0;
    private double fVelocityMax = 0.0;
    private double fAudioMax = 0.0;
    private int strikeIndex = 0;

    // low pass filter
    private final static int LPF_ORDER = 2;
    private final static double[] LpfGyroA = new double[]{1.0, -1.647459981076977, 0.700896781188403};
    private final static double[] LpfGyroB = new double[]{0.013359200027856, 0.026718400055713, 0.013359200027856};
    private double[][] LpfGyroX = new double[3][2];
    private double[][] LpfGyroY = new double[3][2];
    private final static double[] LpfAccA = new double[]{1.0, -1.647459981076977, 0.700896781188403};
    private final static double[] LpfAccB = new double[]{0.013359200027856, 0.026718400055713, 0.013359200027856};
    private double[][] LpfAccX = new double[3][2];
    private double[][] LpfAccY = new double[3][2];

    private SensorKalman sensorKalman;
    private StringBuffer sAttitude;
    public TrainData trainData;

    // extra data for performance tuning
    public double[] extLinerAccIBP = new double[]{0, 0, 0};
    public double[] extPlatQ = new double[]{0, 0, 0, 0};

    private final static double MAX_OMEGA_DEG = 2000;
    private final static double OMEGA_MARGIN = 10;
    public native double splineFitting(double[] x0, double[] y0, int num, double x);

    // refine parameters
    private double fOmegaMax = 0;
    private int fOmegaMaxIndex = 0;
    private double fOmegaMin = 0;
    private int fOmegaMinIndex = 0;
    private double fOmegaPeak = 0;
    private double fOmegaLetter = 0;
    private double fScale = 10;
    private double fAccMaxX = 0;
    private double fAccMinX = 0;
    private int fAccMaxIndex = 0;
    private int fAccMinIndex = 0;

    // forehand special refine parameters
    private double fLastForehandActionTime = 0;
    private int uContinueForehandActionCount = 0;
    private boolean bContinueForehandActionRefine = false;

    static
    {
        System.loadLibrary("fitting");
    }

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
        uTime = time;
        fOmegaB = gyro;
        fAccelerate = acc;
        fMagnetic = mag;
        fAudio = audio;

        if (uActionComplete == true)
        {
            uActionComplete = false;
            cSampleDataArray.clear();
        }

        // recording the raw data
        double[] fGyroRaw = new double[]{gyro[0], gyro[1], gyro[2]};
        double[] fAccRaw = new double[]{acc[0], acc[1], acc[2]};
        double[] fMagRaw = new double[]{mag[0], mag[1], mag[2]};
        fOmegaBRaw = fGyroRaw;
        fAccelerateRaw = fAccRaw;
        fMagneticRaw = fMagRaw;

        // data filter
        gyroFilter(gyro); // 4Hz cutoff frequency
        accFilter(acc);   // 4Hz cutoff frequency

        // data correction
        sensorDataCorrection(gyro, acc, mag);

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
        if (MAG_SUPPORT == 1) {
            if (uStaticFlag == 0 && iStatus == Calibration)
            {
                if (magCalibration(mag) == true)
                {
                    // mag calibration process complete
                    if (fGeoB > 10 && fGeoB < 100 && fResidual < 10)
                    {
                        fB = fGeoB;
                        iValidMagCal = true;
                        fV[CHX] = fMagBias[CHX];
                        fV[CHY] = fMagBias[CHY];
                        fV[CHY] = fMagBias[CHZ];
                        iStatus = Alignment;
                        CalibrationProgress = 100;
                    }
                }
                else
                {
                    // mag calibration process not execute
                    CalibrationProgress = (int)(fCalibrationMagArray.size() * 95 / CALIBRATION_NUM );
                }
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
                    uAlignFlag = true;
                    iStatus = Fusion;
                }
            }

            return null;
        }

        if (MAG_SUPPORT == 1)
        {
            if (uStaticFlag == 0) {
                magBufferUpdate(fMagRaw, mag, time);
            }
            if (time % 100 == 0) // 1Hz calibration frequency
            {
                magCalibrationExec();
            }
        }

        // AHRS/INS process
        sAttitude = new StringBuffer();

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
            SampleData sampleData = new SampleData();

            // copy sample data into array list
            copyInSampleData(this, sampleData);
            cSampleDataArray.add(sampleData);
        }

        // refine the sample data array
        if (uActionComplete == true)
        {
            //outlierCompensate(cSampleDataArray, gyro);
            refineSampleData(cSampleDataArray);
            specialActionProcess(cSampleDataArray);
        }

        // process the sample data array
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

            double[] euler = dcm2euler(fCbnPlat);
            fPsiPlPlat = euler[0];
            fThePlPlat = euler[1];
            fPhiPlPlat = euler[2];
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

        // record the debug info
        for (i = 0; i < 4; i++)
        {
            extPlatQ[i] = fqPlPlat[i];
        }
    }

    private int processSampleData(ArrayList<SampleData> sampleDataArray, TrainData data)
    {
        int typeIndex = -1;
        int count = 0;
        StringBuffer trajectory = new StringBuffer();
        SampleData value;

        fPlatformOmegaMaxZ = 0;
        fPlatformOmegaMinZ = 0;
        fRangeMax = 0.0;
        fVelocityMax = 0.0;
        fAudioMax = 0.0;
        strikeIndex = 0;

        if (sampleDataArray.isEmpty())
        {
            return -1;
        }

        insStrapdownMechanization(dt, sampleDataArray);

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
//            if (count == strikeIndex)
//            {
//                trajectory.append("s");
//            }
//            else
//            {
//                trajectory.append("x");
//            }
            trajectory.append("x");
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
        if (strikeIndex != -1) {
            value = sampleDataArray.get(strikeIndex);
            data.fVelocityStrike = Math.sqrt(value.fVelN * value.fVelN + value.fVelE * value.fVelE + value.fVelD * value.fVelD);
        }
        else {
            data.fVelocityStrike = data.fVelocityMax;
        }
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

        // replace the type character in trajectory
        while ((typeIndex = trajectory.indexOf("x")) != -1)
        {
            trajectory.replace(typeIndex, typeIndex + 1, data.sActionType.substring(0,1));
        }
        data.sTrajectory = String.valueOf(trajectory);

        // update audio index
        updateAudioInfo(data);

        // update action score
        updateActionScore(data);

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

    private void updateActionScore(TrainData data)
    {
        int iscore = (int)((data.uStrikePower + data.fVelocityStrike * 10) / 2.0);

        if (data.sActionType.equals("push-pull"))
        {
            iscore *= 2;
            if (data.fRangeMax > 0.5 && data.fRangeMax < 0.8)
            {
                iscore += 20;
            }
            else if (data.fRangeMax > 0.8)
            {
                iscore -= (data.fRangeMax - 0.8) * 50;
            }
            else
            {
                iscore -= (0.5 - data.fRangeMax) * 50;
            }
        }
        else if (data.sActionType.equals("backhand"))
        {
            iscore *= 3;
            if (data.fRangeMax > 0.3 && data.fRangeMax < 1.0)
            {
                iscore += 10;
            }
            else if (data.fRangeMax > 1.0)
            {
                iscore -= (data.fRangeMax - 1.0) * 50;
            }
            else
            {
                iscore -= (0.3 - data.fRangeMax) * 50;
            }
        }
        else if (data.sActionType.equals("forehand"))
        {
            iscore += 30;
            if (data.fRangeMax > 0.8 && data.fRangeMax < 1.5)
            {
                iscore += 10;
            }
            else if (data.fRangeMax > 1.5)
            {
                iscore -= (data.fRangeMax - 1.5) * 50;
            }
            else
            {
                iscore -= (1.0 - data.fRangeMax) * 50;
            }
        }
        else
        {

        }

        if (iscore > 99)
        {
            iscore = 99;
        }
        else if (iscore < 20)
        {
            iscore = 30;
        }
        data.iScore = iscore;
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
        dst.fLinerAccN = src.fAccelerate[0] * src.fCbnPlat[0][0] + src.fAccelerate[1] * src.fCbnPlat[0][1] + src.fAccelerate[2] * src.fCbnPlat[0][2];
        dst.fLinerAccE = src.fAccelerate[0] * src.fCbnPlat[1][0] + src.fAccelerate[1] * src.fCbnPlat[1][1] + src.fAccelerate[2] * src.fCbnPlat[1][2];
        dst.fLinerAccD = src.fAccelerate[0] * src.fCbnPlat[2][0] + src.fAccelerate[1] * src.fCbnPlat[2][1] + src.fAccelerate[2] * src.fCbnPlat[2][2] + GRAVITY;
        dst.fVelN = src.fVelN;
        dst.fVelE = src.fVelE;
        dst.fVelD = src.fVelD;
        dst.fPosN = src.fPosN;
        dst.fPosE = src.fPosE;
        dst.fPosD = src.fPosD;
        dst.fOmegaB = Arrays.copyOf(src.fOmegaB, src.fOmegaB.length);
        dst.fAccelerate = Arrays.copyOf(src.fAccelerate, src.fAccelerate.length);
        dst.fMagnetic = Arrays.copyOf(src.fMagnetic, src.fMagnetic.length);
        dst.fOmegaBRaw = Arrays.copyOf(src.fOmegaBRaw, src.fOmegaBRaw.length);
        dst.fAccelerateRaw = Arrays.copyOf(src.fAccelerateRaw, src.fAccelerateRaw.length);
        dst.fMagneticRaw = Arrays.copyOf(src.fMagneticRaw, src.fMagneticRaw.length);
        dst.fAudio = src.fAudio;
        for (i = 0; i < 3; i++)
        {
            dst.fOmegaN[i] = src.fCbnPlat[i][0] * src.fOmegaB[0] + src.fCbnPlat[i][1] * src.fOmegaB[1] + src.fCbnPlat[i][2] * src.fOmegaB[2];
        }
        dst.fVel = Math.sqrt(src.fVelN*src.fVelN +  src.fVelE*src.fVelE + src.fVelD*src.fVelD);

        return 0;
    }

    private void ahrsFusionGeneral(double[] fq, double dt, double[] gyro, double[] acc, double[] mag)
    {
        double accNorm = 0;
        double[] qDot = new double[]{0, 0, 0, 0};
        double[] qDotError = new double[]{0, 0, 0, 0};
        double gyroMeasError = 10 * Math.PI / 180; // gyroscope measurement error in rad/s (shown as 10 deg/s)
        double beta = Math.sqrt(3.0 / 4.0) * gyroMeasError;

        qDot[0] = -(gyro[0] * fq[1] + gyro[1] * fq[2] + gyro[2] * fq[3]) / 2.0;
        qDot[1] =  (gyro[0] * fq[0] + gyro[2] * fq[2] - gyro[1] * fq[3]) / 2.0;
        qDot[2] =  (gyro[1] * fq[0] - gyro[2] * fq[1] + gyro[0] * fq[3]) / 2.0;
        qDot[3] =  (gyro[2] * fq[0] + gyro[1] * fq[1] - gyro[0] * fq[2]) / 2.0;

        accNorm = Math.sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
        if (accNorm > 8 && accNorm < 12) {
            // execute the acc aid process
            double diff = 0;
            double[] gEstimate = new double[3];
            Matrix F = new Matrix(3, 1);
            Matrix J = new Matrix(3, 4);
            Matrix step = new Matrix(4, 1);

            gEstimate[0] = -acc[0] / accNorm;
            gEstimate[1] = -acc[1] / accNorm;
            gEstimate[2] = -acc[2] / accNorm;

            F.set(0, 0, 2 * (fq[1] * fq[3] - fq[0] * fq[2]) - gEstimate[0]);
            F.set(1, 0, 2 * (fq[0] * fq[1] + fq[2] * fq[3]) - gEstimate[1]);
            F.set(2, 0, 2 * (0.5 - fq[1] * fq[1] - fq[2] * fq[2]) - gEstimate[2]);

            J.set(0, 0, -2 * fq[2]);
            J.set(0, 1, 2 * fq[3]);
            J.set(0, 2, -2 * fq[0]);
            J.set(0, 3, 2 * fq[1]);

            J.set(1, 0, 2 * fq[1]);
            J.set(1, 1, 2 * fq[0]);
            J.set(1, 2, 2 * fq[3]);
            J.set(1, 3, 2 * fq[2]);

            J.set(2, 0, 0);
            J.set(2, 1, -4 * fq[1]);
            J.set(2, 2, -4 * fq[2]);
            J.set(2, 3, 0);

            step = J.transpose().times(F);
            qDotError[0] += step.get(0, 0);
            qDotError[1] += step.get(1, 0);
            qDotError[2] += step.get(2, 0);
            qDotError[3] += step.get(3, 0);
        }

        gyroMeasError = 3 * Math.PI / 180;
        beta = Math.sqrt(3.0 / 4.0) * gyroMeasError;

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

        for (int i = 0; i < 4; i++)
        {
            fq[i] += qDot[i] * dt;
        }
        qNorm(fq);
    }

    private void ahrsFusionRefine(double[] fq, double dt, double[] gyro, double[] acc, double[] mag)
    {
        double[] fg = new double[]{-acc[0], -acc[1], -acc[2]};

        q2dcm(fq, fCbn);
        double[] euler = dcm2euler(fCbn);
        fPsiPl = euler[0];
        fThePl = -Math.asin(fg[0] / SensorFusion.GRAVITY);
        fPhiPl = Math.atan2(fg[1] / SensorFusion.GRAVITY, fg[2] / SensorFusion.GRAVITY);
        euler2q(fq, fPsiPl, fThePl, fPhiPl);
    }

    private void ahrsFusionReset(double[] fq, double dt, double[] gyro, double[] acc, double[] mag)
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

        for (i = 0; i < 3; i++)
        {
            fg[i] = -acc[i];
            fR[i][Z] = fg[i];;
        }

        for (i = 0; i < 3; i++)
        {
            fm[i] = mag[i];
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
            return;
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
        euler2q(fq, fPsiPl, fThePl, fPhiPl);
    }

    private void ahrsFusion(double[] fq, double dt, double[] gyro, double[] acc, double[] mag)
    {
        double accNorm = 0;
        double[] qDot = new double[]{0, 0, 0, 0};
        double[] qDotError = new double[]{0, 0, 0, 0};
        double gyroMeasError = 10 * Math.PI / 180; // gyroscope measurement error in rad/s (shown as 10 deg/s)
        double beta = Math.sqrt(3.0 / 4.0) * gyroMeasError;

        qDot[0] = -(gyro[0] * fq[1] + gyro[1] * fq[2] + gyro[2] * fq[3]) / 2.0;
        qDot[1] =  (gyro[0] * fq[0] + gyro[2] * fq[2] - gyro[1] * fq[3]) / 2.0;
        qDot[2] =  (gyro[1] * fq[0] - gyro[2] * fq[1] + gyro[0] * fq[3]) / 2.0;
        qDot[3] =  (gyro[2] * fq[0] + gyro[1] * fq[1] - gyro[0] * fq[2]) / 2.0;

        accNorm = Math.sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
        if (accNorm > 5 && accNorm < 30) {
            // execute the acc aid process
            double diff = 0;
            double[] gEstimate = new double[3];
            Matrix F = new Matrix(3, 1);
            Matrix J = new Matrix(3, 4);
            Matrix step = new Matrix(4, 1);

            gEstimate[0] = -acc[0] / accNorm;
            gEstimate[1] = -acc[1] / accNorm;
            gEstimate[2] = -acc[2] / accNorm;

            F.set(0, 0, 2 * (fq[1] * fq[3] - fq[0] * fq[2]) - gEstimate[0]);
            F.set(1, 0, 2 * (fq[0] * fq[1] + fq[2] * fq[3]) - gEstimate[1]);
            F.set(2, 0, (fq[0] * fq[0] - fq[1] * fq[1] - fq[2] * fq[2] + fq[3] * fq[3]) - gEstimate[2]);

            J.set(0, 0, -2 * fq[2]);
            J.set(0, 1, 2 * fq[3]);
            J.set(0, 2, -2 * fq[0]);
            J.set(0, 3, 2 * fq[1]);

            J.set(1, 0, 2 * fq[1]);
            J.set(1, 1, 2 * fq[0]);
            J.set(1, 2, 2 * fq[3]);
            J.set(1, 3, 2 * fq[2]);

            J.set(2, 0, 2 * fq[0]);
            J.set(2, 1, -2 * fq[1]);
            J.set(2, 2, -2 * fq[2]);
            J.set(2, 3, 2 * fq[3]);

            step = J.transpose().times(F);
            qDotError[0] += step.get(0, 0);
            qDotError[1] += step.get(1, 0);
            qDotError[2] += step.get(2, 0);
            qDotError[3] += step.get(3, 0);
        }

        if (MAG_SUPPORT == 1)
        {
            double magNorm = Math.sqrt(mag[0]*mag[0] + mag[1]*mag[1] + mag[2]*mag[2]);
            if (iValidMagCal && magNorm > 0.8*fB && magNorm < 1.2*fB)
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

                F.set(0, 0, b[1]*(fq[0]*fq[0] + fq[1]*fq[1] - fq[2]*fq[2] - fq[3]*fq[3]) + 2*b[3]*(fq[1]*fq[3] - fq[0]*fq[2]) - mEstimate[0]);
                F.set(1, 0, 2*b[1]*(fq[1]*fq[2] - fq[0]*fq[3]) + 2*b[3]*(fq[0]*fq[1] + fq[2]*fq[3]) - mEstimate[1]);
                F.set(2, 0, 2*b[1]*(fq[0]*fq[2] + fq[1]*fq[3]) + b[3]*(fq[0]*fq[0] - fq[1]*fq[1] - fq[2]*fq[2] + fq[3]*fq[3]) - mEstimate[2]);

                J.set(0, 0, 2 * b[1] * fq[0] -2 * b[3] * fq[2]);
                J.set(0, 1, 2 * b[1] * fq[1] + 2 * b[3] * fq[3]);
                J.set(0, 2, -2 * b[1] * fq[2] - 2 * b[3] * fq[0]);
                J.set(0, 3, -2 * b[1] * fq[3] + 2 * b[3] * fq[1]);

                J.set(1, 0, -2 * b[1] * fq[3] + 2 * b[3] * fq[1]);
                J.set(1, 1, 2 * b[1] * fq[2] + 2 * b[3] * fq[0]);
                J.set(1, 2, 2 * b[1] * fq[1] + 2 * b[3] * fq[3]);
                J.set(1, 3, -2 * b[1] * fq[0] + 2 * b[3] * fq[2]);

                J.set(2, 0, 2 * b[1] * fq[2] + 2 * b[3] * fq[0]);
                J.set(2, 1, 2 * b[1] * fq[3] - 2 * b[3] * fq[1]);
                J.set(2, 2, 2 * b[1] * fq[0] - 2 * b[3] * fq[2]);
                J.set(2, 3, 2 * b[1] * fq[1] + 2 * b[3] * fq[3]);

                diff = F.norm2();
                step = J.transpose().times(F);
                qDotError[0] += step.get(0, 0);
                qDotError[1] += step.get(1, 0);
                qDotError[2] += step.get(2, 0);
                qDotError[3] += step.get(3, 0);
            }
        }

        if (accNorm > 5.0 && accNorm < 30.0){
            gyroMeasError = 60 * Math.PI / 180;
        }
        else
        {
            gyroMeasError = 30 * Math.PI / 180;
        }
        beta = Math.sqrt(3.0 / 4.0) * gyroMeasError;

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

        for (int i = 0; i < 4; i++)
        {
            fq[i] += qDot[i] * dt;
        }
        qNorm(fq);
    }

    private void ahrsProcess(double dt, double[] gyro, double[] acc, double[] mag)
    {
        double accNorm = Math.sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);

         ahrsFusion(fqPl, dt, gyro, acc, mag);
         if (accNorm > 9 && accNorm < 11) {
             ahrsFusionReset(fqPl, dt, gyro, acc, mag);
         }
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

    private void insStrapdownMechanization(double dt, ArrayList<SampleData> sampleDataArray)
    {
        int i;
        double[] linerAccIBP = new double[]{0, 0, 0};
        double[] velIBP = new double[]{0, 0, 0};
        double[] linerAccAve = new double[]{0, 0, 0};
        double[] velAve = new double[]{0, 0, 0};
        double deltaN = 0.0;
        double deltaE = 0.0;
        double deltaD = 0.0;
        SampleData valLast;

        for(SampleData val:sampleDataArray)
        {
            int index = sampleDataArray.indexOf(val);

            linerAccIBP[CHX] = val.fLinerAccN;
            linerAccIBP[CHY] = val.fLinerAccE;
            linerAccIBP[CHZ] = val.fLinerAccD;

            if (index == 0)
            {
                valLast = new SampleData();
                valLast.fLinerAccN = 0;
                valLast.fLinerAccE = 0;
                valLast.fLinerAccD = 0;
                valLast.fVelN = 0;
                valLast.fVelE = 0;
                valLast.fVelD = 0;
                valLast.fPosN = 0;
                valLast.fPosE = 0;
                valLast.fPosD = 0;
            }
            else
            {
                valLast = sampleDataArray.get(index -1);
            }
            linerAccAve[0] = (linerAccIBP[0] + valLast.fLinerAccN) / 2.0;
            linerAccAve[1] = (linerAccIBP[1] + valLast.fLinerAccE) / 2.0;
            linerAccAve[2] = (linerAccIBP[2] + valLast.fLinerAccD) / 2.0;
            velIBP[0] = valLast.fVelN + linerAccAve[0] * dt;
            velIBP[1] = valLast.fVelE + linerAccAve[1] * dt;
            velIBP[2] = valLast.fVelD + linerAccAve[2] * dt;
            velAve[0] = (valLast.fVelN + velIBP[0]) / 2.0;
            velAve[1] = (valLast.fVelE + velIBP[1]) / 2.0;
            velAve[2] = (valLast.fVelD + velIBP[2]) / 2.0;

            val.fVelN = velIBP[0];
            val.fVelE = velIBP[1];
            val.fVelD = velIBP[2];
            val.fVel = Math.sqrt(val.fVelN*val.fVelN + val.fVelE*fVelE + val.fVelD*val.fVelD);
            deltaN = velAve[0] * dt;
            deltaE = velAve[1] * dt;
            deltaD = velAve[2] * dt;
            val.fPosN = valLast.fPosN + deltaN;
            val.fPosE = valLast.fPosE+ deltaE;
            val.fPosD = valLast.fPosD + deltaD;
        }
    }

    private void actionDetect(double dt, double[] gyro, double[] acc)
    {
        int i;
        int slop = 0;
        int slopGyro = 0;
        double[] linerAccIBP = new double[]{0, 0, 0};
        double linerAccX = 0.0;

        // calculate the liner accelerate along the x axis
        for (i = 0; i < 3; i++)
        {
            linerAccIBP[i] = acc[0]*fCbnPlat[i][0] + acc[1]*fCbnPlat[i][1] + acc[2]*fCbnPlat[i][2];
        }
        linerAccIBP[2] += GRAVITY;

        // record the extra data for performance tuning
        for (i = 0; i < 3; i++)
        {
            extLinerAccIBP[i] = linerAccIBP[i];
        }

        // static constrain
        for (i = 0; i < 3; i++)
        {
            if (Math.abs(linerAccIBP[i]) < 1)
            {
                linerAccIBP[i] = 0;
            }
        }
        linerAccX = linerAccIBP[0];
        if (gyro[CHZ] > fGyroZLast)
        {
            slopGyro = 1;
        }
        else
        {
            slopGyro = -1;
        }
        if (slopGyro * iSlopGyroLast < 0)
        {
            bSlopChange = true;
        }
        switch(iCurveCondition)
        {
            case Peace:
                if (linerAccX > 3){
                    uActionStartFlag = true;
                    iCurveCondition = Step1;
                    actionTime = 0;
                    bSlopChange = false;
                    iSlopGyroLast = 0;
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
                if (linerAccX > fLinerAccXLast){
                    slop = 1;
                }else{
                    slop = -1;
                    // reach the up peak
                    if (fLinerAccXLast < 4 && Math.abs(gyro[CHZ]) < 10){
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
                if (actionTime > 1.5 || downTime > 1.0 || (Math.abs(linerAccX) < 1e-5 && Math.abs(fLinerAccXLast) < 1e-5))
                {
                    iCurveCondition = Peace;
                    uActionStartFlag = false;
                    break;
                }
                if (linerAccX > fLinerAccXLast){
                    slop = 1;
                    // reach the trough
                    if (fLinerAccXLast > 0.5 * peakValue){
                        // there are 2 kind of false peak:
                        // 1. first peak is false peak
                        // 2. the following peak is false peak
                        // we recording the two kinds of false peak for the following refine
                    }
                    else if(fLinerAccXLast > -6){
                        // false trough
                        // no action, because it is normal
                    }
                    else{
                        iCurveCondition = Step3;
                    }
                }else{
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
                if (linerAccX > fLinerAccXLast){
                    slop = 1;
                }else {
                    slop = -1;
                }
                if (linerAccX > -10 && linerAccX < 10 && Math.abs(gyro[CHZ]) < 5 && bSlopChange){
                    uActionEndFlag = true;
                    iCurveCondition = Peace;
                }
                break;
        }
        fLinerAccXLast = linerAccX;
        fGyroZLast = gyro[CHZ];
        iSlopGyroLast = slopGyro;
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
            euler2dcm(fCnp, 0, 0, 0);

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

        //if (Math.abs(bias[0]) < 0.01 && Math.abs(bias[1]) < 0.01 && Math.abs(bias[2]) < 0.01)
        {
            for (i = 0; i < 3; i++)
            {
                fGyroBias[i] = bias[i];
            }
        }
    }

    private int staticDetectUpdate(double[] gyro, double[] acc, double[] mag)
    {
        if (fAlignGyroArray.size() >= ALIGN_NUM)
        {
            fAlignAccArray.remove(0);
            fAlignGyroArray.remove(0);
            fAlignMagArray.remove(0);
        }

        fAlignAccArray.add(new double[]{acc[0], acc[1], acc[2]});
        fAlignGyroArray.add(new double[]{gyro[0], gyro[1], gyro[2]});
        fAlignMagArray.add(new double[]{mag[0],mag[1],mag[2]});

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

            if (gyro_std < 0.05 && acc_std < 0.5)
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

    private double stdCalVec(ArrayList<Double> numList)
    {
        double value = 0; // the sum of Xi
        double square = 0; // the sum of xi^2
        double sigma = 0; // standard deviation
        double size = numList.size();

        for(Double val:numList)
        {
            value += val.doubleValue();
            square += val.doubleValue()*val.doubleValue();
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
        if (iMagBufferCount > 200)
        {
            isolver = 4;
            calibration4INV();
        }

        if (ftrFitErrorpc <= MAGFITERROR && ftrB > 10 && ftrB < 100)
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
            ftrV[l] = ftrV[l] * DEFAULTB + iOffset[l] * 1.0 * fuTPerCount;
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
        fCalibrationMagArray.clear();
        fAlignGyroArray.clear();
        fAlignAccArray.clear();
        fAlignMagArray.clear();
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

    private boolean magCalibration(double[] mag)
    {
        if (mag[2] == 0)
        {
            return false;
        }
        if (fCalibrationMagArray.size() >= CALIBRATION_NUM)
        {
            fCalibrationMagArray.remove(0);
        }
        fCalibrationMagArray.add(new double[]{mag[0], mag[1], mag[2]});

        if (fCalibrationMagArray.size() == CALIBRATION_NUM)
        {
            calibration4INV_Raw(fCalibrationMagArray);

            return true;
        }

        return false;
    }

    private void calibration4INV_Raw(ArrayList<double[]> magArray)
    {
        // local variables
        double fBs2;							// fBs[CHX]^2+fBs[CHY]^2+fBs[CHZ]^2
        double fSumBs4;							// sum of fBs2
        double fscaling;						// set to FUTPERCOUNT * FMATRIXSCALING
        double fE;								// error function = r^T.r
        double[] fOffset = new double[3];		// offset to remove large DC hard iron bias in matrix
        int iCount;							    // number of measurements counted
        int ierror;							    // matrix inversion error flag
        int i, j, k, l;						    // loop counters

        double ftrB;
        double ftrFitErrorpc;
        double DEFAULTB = 50.0;
        double[] fvecB = new double[4];
        double[] fvecA = new double[6];
        double[][] fmatA = new double[4][4];
        double[][] fmatB = new double[4][4];
        double[] ftrV = new double[3];

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
        for(double[] mag:magArray)
        {
            // use first valid magnetic buffer entry as estimate (in counts) for offset
            if (iCount == 0)
            {
                for (l = 0; l <= 2; l++)
                {
                    fOffset[l] = mag[l];
                }
            }
            // store scaled and offset fBs[XYZ] in fvecA[0-2] and fBs[XYZ]^2 in fvecA[3-5]
            for (l = 0; l <= 2; l++)
            {
                fvecA[l] = (mag[l] - fOffset[l]) * fscaling;
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
        for (l = 0; l <= 2; l++)
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

    private void gyroFilter(double[] gyro)
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

    private void accFilter(double[] acc)
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

    private void outlierCompensate(ArrayList<SampleData> sampleDataArray, double[] gyro)
    {
        int[] left_index = new int[]{-1,-1,-1};
        int[] right_index = new int[]{-1,-1,-1};
        int index;
        boolean[] outlier_flag = new boolean[]{false, false, false};

        for (int i = CHX; i <= CHZ; i++) {
            index = 0;
            for (SampleData val : sampleDataArray) {
                if (left_index[i] == -1) {
                    if (Math.abs(val.fOmegaBRaw[i]) > Math.toRadians(MAX_OMEGA_DEG - OMEGA_MARGIN)) {
                        left_index[i] = index;
                    }
                } else {
                    if (right_index[i] == -1) {
                        if (Math.abs(val.fOmegaBRaw[i]) < Math.toRadians(MAX_OMEGA_DEG - OMEGA_MARGIN)) {
                            right_index[i] = index - 1;
                            outlier_flag[i] = true;
                        }
                    }
                }

                if (outlier_flag[i]) {
                    break;
                }
                index++;
            }
        }

        for (int channel = CHX; channel <= CHZ; channel++) {
            if (outlier_flag[channel]) {
                /* estimate the outlier value */
                double[] x0 = new double[6];
                double[] y0 = new double[6];
                int seq_left = left_index[channel] - 1;
                int seq_right = right_index[channel] + 1;

                // check the index valid
                if (seq_left - 2 < 0 || seq_right + 2 > sampleDataArray.size() - 1) {
                    return;
                }
                x0[0] = seq_left - 2;
                x0[1] = seq_left - 1;
                x0[2] = seq_left;
                x0[3] = seq_right;
                x0[4] = seq_right + 1;
                x0[5] = seq_right + 2;
                for (int i = 0; i < 6; i++) {
                    y0[i] = sampleDataArray.get((int) x0[i]).fOmegaBRaw[channel];
                }
                for (int i = left_index[channel]; i <= right_index[channel]; i++) {
                    sampleDataArray.get(i).fOmegaBRaw[channel] = splineFitting(x0, y0, 6, i);
                }

                /* recompute the past filtered gyro value */
                double[] lpfGyroX = new double[2];
                double[] lpfGyroY = new double[2];
                //left_index = 2;
                lpfGyroX[0] = sampleDataArray.get(left_index[channel] - 1).fOmegaBRaw[channel];
                lpfGyroX[1] = sampleDataArray.get(left_index[channel] - 2).fOmegaBRaw[channel];
                lpfGyroY[0] = sampleDataArray.get(left_index[channel] - 1).fOmegaB[channel] + fGyroBias[channel];
                lpfGyroY[1] = sampleDataArray.get(left_index[channel] - 2).fOmegaB[channel] + fGyroBias[channel];

                for (int i = left_index[channel]; i < sampleDataArray.size(); i++) {
                    double gyroN = sampleDataArray.get(i).fOmegaBRaw[channel];
                    double temp = LpfGyroB[0] * gyroN + LpfGyroB[1] * lpfGyroX[0] + LpfGyroB[2] * lpfGyroX[1]
                            - LpfGyroA[1] * lpfGyroY[0] - LpfGyroA[2] * lpfGyroY[1];
                    lpfGyroX[1] = lpfGyroX[0];
                    lpfGyroX[0] = gyroN;
                    lpfGyroY[1] = lpfGyroY[0];
                    lpfGyroY[0] = temp;
                    temp -= fGyroBias[channel];
                    sampleDataArray.get(i).fOmegaB[channel] = temp;
                }

                /* recompute the current filtered gyro value */
                double gyroN = fOmegaBRaw[channel];
                double temp = LpfGyroB[0] * gyroN + LpfGyroB[1] * lpfGyroX[0] + LpfGyroB[2] * lpfGyroX[1]
                        - LpfGyroA[1] * lpfGyroY[0] - LpfGyroA[2] * lpfGyroY[1];
                lpfGyroX[1] = lpfGyroX[0];
                lpfGyroX[0] = gyroN;
                lpfGyroY[1] = lpfGyroY[0];
                lpfGyroY[0] = temp;
                temp -= fGyroBias[channel];
                fOmegaB[channel] = gyro[channel] = temp;
                // store the new filter parameters
                LpfGyroX[channel][1] = lpfGyroX[1];
                LpfGyroX[channel][0] = lpfGyroX[0];
                LpfGyroY[channel][1] = lpfGyroY[1];
                LpfGyroY[channel][0] = lpfGyroY[0];
            }
        }

        if (outlier_flag[CHX] || outlier_flag[CHY] || outlier_flag[CHZ])
        {
            int left_start = 1;
            /* recompute the past attitude */
            Matrix cbn = new Matrix(3, 3);
            Matrix cnp = new Matrix(fCnp);
            Matrix cbnPlatform = new Matrix(sampleDataArray.get(left_start - 1).fCbnPlat);
            double[] fEuler = new double[3];
            double[][] fCbn = new double[3][3];
            double[] fqBase = new double[4];

            cbn = cnp.transpose().times(cbnPlatform);
            fCbn = cbn.getArray();
            fEuler = dcm2euler(fCbn);
            euler2q(fqBase, fEuler[0], fEuler[1], fEuler[2]);
            for (int k = left_start; k < sampleDataArray.size(); k++) {
                double[] fGyro = new double[]{sampleDataArray.get(k).fOmegaB[CHX], sampleDataArray.get(k).fOmegaB[CHY], sampleDataArray.get(k).fOmegaB[CHZ]};
                double[] fAcc = new double[]{sampleDataArray.get(k).fAccelerate[CHX], sampleDataArray.get(k).fAccelerate[CHY], sampleDataArray.get(k).fAccelerate[CHZ]};
                double[] fMag = new double[]{sampleDataArray.get(k).fMagnetic[CHX], sampleDataArray.get(k).fMagnetic[CHY], sampleDataArray.get(k).fMagnetic[CHZ]};

                ahrsFusion(fqBase, dt, fGyro, fAcc, fMag);
                q2dcm(fqBase, fCbn);
                cbn = new Matrix(fCbn);
                cnp = new Matrix(fCnp);
                cbnPlatform = cnp.times(cbn);
                sampleDataArray.get(k).fCbnPlat = cbnPlatform.getArray();
                fEuler = dcm2euler(sampleDataArray.get(k).fCbnPlat);
                euler2q(sampleDataArray.get(k).fqPlPlat, fEuler[0], fEuler[1], fEuler[2]);
                sampleDataArray.get(k).fPsiPlPlat = fEuler[0];
                sampleDataArray.get(k).fThePlPlat = fEuler[1];
                sampleDataArray.get(k).fPhiPlPlat = fEuler[2];
                sampleDataArray.get(k).fLinerAccN = sampleDataArray.get(k).fAccelerate[0] * sampleDataArray.get(k).fCbnPlat[0][0] + sampleDataArray.get(k).fAccelerate[1] * sampleDataArray.get(k).fCbnPlat[0][1] + sampleDataArray.get(k).fAccelerate[2] * sampleDataArray.get(k).fCbnPlat[0][2];
                sampleDataArray.get(k).fLinerAccE = sampleDataArray.get(k).fAccelerate[0] * sampleDataArray.get(k).fCbnPlat[1][0] + sampleDataArray.get(k).fAccelerate[1] * sampleDataArray.get(k).fCbnPlat[1][1] + sampleDataArray.get(k).fAccelerate[2] * sampleDataArray.get(k).fCbnPlat[1][2];
                sampleDataArray.get(k).fLinerAccD = sampleDataArray.get(k).fAccelerate[0] * sampleDataArray.get(k).fCbnPlat[2][0] + sampleDataArray.get(k).fAccelerate[1] * sampleDataArray.get(k).fCbnPlat[2][1] + sampleDataArray.get(k).fAccelerate[2] * sampleDataArray.get(k).fCbnPlat[2][2];
                for (int i = CHX; i <= CHZ; i++) {
                    sampleDataArray.get(k).fOmegaN[i] = sampleDataArray.get(k).fCbnPlat[i][0] * sampleDataArray.get(k).fOmegaB[0] + sampleDataArray.get(k).fCbnPlat[i][1] * sampleDataArray.get(k).fOmegaB[1] + sampleDataArray.get(k).fCbnPlat[i][2] * sampleDataArray.get(k).fOmegaB[2];
                }
            }

            /* recompute the current attitude */
            double[] fGyro = new double[]{fOmegaB[CHX], fOmegaB[CHY], fOmegaB[CHZ]};
            double[] fAcc = new double[]{fAccelerate[CHX], fAccelerate[CHY], fAccelerate[CHZ]};
            double[] fMag = new double[]{fMagnetic[CHX], fMagnetic[CHY], fMagnetic[CHZ]};
            Matrix cnbTemp;

            ahrsFusion(fqBase, dt, fGyro, fAcc, fMag);
            qNorm(fqBase);
            q2dcm(fqBase, fCbn);
            cbn = new Matrix(fCbn);
            cnp = new Matrix(fCnp);
            cbnPlatform = cnp.times(cbn);
            cnbTemp = new Matrix(fCbn).transpose();
            fCbnPlat = cbnPlatform.getArray();
            this.fCbn = fCbn;
            this.fCnb = cnbTemp.getArray();
            fEuler = dcm2euler(fCbn);
            fPsiPl = fEuler[0];
            fThePl = fEuler[1];
            fPhiPl = fEuler[2];
            this.fqPl = fqBase;
            fEuler = dcm2euler(fCbnPlat);
            fPsiPlPlat = fEuler[0];
            fThePlPlat = fEuler[1];
            fPhiPlPlat = fEuler[2];
            euler2q(fqPlPlat, fEuler[0], fEuler[1], fEuler[2]);
        }
    }

    private void specialActionProcess(ArrayList<SampleData> sampleDataArray)
    {
        if (sampleDataArray.isEmpty())
        {
            return;
        }

        // move action check
        ArrayList<Double> numList = new ArrayList<Double>(30);
        double[] stdChanel = new double[]{0,0,0};

        for (int i = CHX; i <= CHZ; i++) {
            for (SampleData val : sampleDataArray) {
                numList.add(val.fAccelerate[i]);
            }
            stdChanel[i] = stdCalVec(numList);
            numList.clear();
        }

        if (stdChanel[CHX] > 5 * stdChanel[CHY] && stdChanel[CHX] > 5 * stdChanel[CHZ])
        {
            // move action refine
            Matrix cnp = new Matrix(fCnp);
            Matrix cbnPlatform = new Matrix(sampleDataArray.get(0).fCbnPlat);
            double[][] cbn = cnp.transpose().times(cbnPlatform).getArray();
            double[] euler = dcm2euler(cbn);
            double[] fq = new double[4];
            double[][] cbnTemp = new double[3][3];
            euler2q(fq, euler[0], 0, euler[2]);

            for (int i = 0; i < sampleDataArray.size(); i++) {
                SampleData p = sampleDataArray.get(i);
                if (i > 0) {
                    double gyro[] = p.fOmegaB;
                    double acc[] = p.fAccelerate;
                    double mag[] = p.fMagnetic;

                    ahrsFusionGeneral(fq, dt, gyro, acc, mag);
                }
                q2dcm(fq, cbnTemp);
                cbnPlatform = cnp.times(new Matrix(cbnTemp));
                sampleDataArray.get(i).fCbnPlat = cbnPlatform.getArray();

                euler = dcm2euler(p.fCbnPlat);
                p.fPsiPlPlat = euler[0];
                p.fThePlPlat = euler[1];
                p.fPhiPlPlat = euler[2];
                euler2q(p.fqPlPlat, p.fPsiPlPlat, p.fThePlPlat, p.fPhiPlPlat);

                p.fLinerAccN = p.fAccelerate[0] * p.fCbnPlat[0][0] + p.fAccelerate[1] * p.fCbnPlat[0][1] + p.fAccelerate[2] * p.fCbnPlat[0][2];
                p.fLinerAccE = p.fAccelerate[0] * p.fCbnPlat[1][0] + p.fAccelerate[1] * p.fCbnPlat[1][1] + p.fAccelerate[2] * p.fCbnPlat[1][2];
                p.fLinerAccD = p.fAccelerate[0] * p.fCbnPlat[2][0] + p.fAccelerate[1] * p.fCbnPlat[2][1] + p.fAccelerate[2] * p.fCbnPlat[2][2] + GRAVITY;

                for (int j = CHX; j <= CHZ; j++)
                {
                    p.fOmegaN[j] = p.fCbnPlat[j][0] * p.fOmegaB[0] + p.fCbnPlat[j][1] * p.fOmegaB[1] + p.fCbnPlat[j][2] * p.fOmegaB[2];
                }
            }
        }
    }

    private boolean checkHeightDown(ArrayList<SampleData> sampleDataArray)
    {
        int index = 0;

        insStrapdownMechanization(dt, sampleDataArray);
        for(SampleData val:sampleDataArray)
        {
            int lastIndex = index - 1;
            if (lastIndex > 0 && val.fPosD > sampleDataArray.get(lastIndex).fPosD)
            {
                return true;
            }
            index++;
        }

        return false;
    }

    private void forehandRefine(ArrayList<SampleData> sampleDataArray)
    {
        if (sampleDataArray.isEmpty())
        {
            return;
        }

        computeRefineParameters(sampleDataArray);
        // remove the false peak
        removeFalsePeak(sampleDataArray);
        // recompute since sample array is changed
        computeRefineParameters(sampleDataArray);

        trainData.sActionType = "forehand";

        if (Math.abs(fOmegaMax) > Math.abs(fOmegaMin) || fOmegaMinIndex < 0.2*sampleDataArray.size() || fOmegaMinIndex > 0.8*sampleDataArray.size())
        {
            fOmegaPeak = fOmegaMax;
            fOmegaLetter = 1;
        }
        else
        {
            fOmegaPeak = fOmegaMin;
            fOmegaLetter = -1;
        }

        double fGyroLastZ = 0;
        int slop = 0;
        final int START = 0;
        final int UP = 1;
        final int DOWN = 2;
        int condition = START;
        boolean errorFlag = false;
        boolean endFlag = false;
        int startIndex = 0;
        int endIndex = 0;

        for(SampleData val:sampleDataArray)
        {
            double gyroZ = val.fOmegaB[2] * fOmegaLetter * fScale;

            switch(condition)
            {
                case START:
                    if (gyroZ > 0.1 * fOmegaMax * fOmegaLetter * fScale)
                    {
                        condition = UP;
                        startIndex = sampleDataArray.indexOf(val);
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
                    else {
                        if (gyroZ < fGyroLastZ) {
                            // peak
                            if (fGyroLastZ > 0.9 * fOmegaPeak * fOmegaLetter * fScale) {
                                // true peak
                                slop = -1;
                                condition = DOWN;
                            }
                            else
                            {
                                slop = -1;
                                // false peak;
                            }
                        } else{
                            // norm case
                            slop = 1;
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
                        if (gyroZ < 0.85 * fOmegaPeak * fScale * fOmegaLetter)
                        {
                            endIndex = sampleDataArray.indexOf(val);
                            endFlag = true;
                        }
                    }
                    break;

            }
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
                sampleDataArray.remove(0);
                endIndex--;
            }
        }
        if (endIndex > 0)
        {
            int arraySize = sampleDataArray.size();
            for (int i = 0; i < arraySize - endIndex - 1; i++)
            {
                sampleDataArray.remove(sampleDataArray.size() - 1);
            }
        }

        // calculate the ins info firstly
        insStrapdownMechanization(dt, sampleDataArray);
        // remove the backward action
        endIndex = 0;
        for(SampleData val:sampleDataArray)
        {
            int lastIndex = sampleDataArray.indexOf(val) - 1;
            if (lastIndex > 0 && val.fPosN < sampleDataArray.get(lastIndex).fPosN)
            {
                endIndex = sampleDataArray.indexOf(val);
                break;
            }
        }
        if (endIndex > 0)
        {
            int arraySize = sampleDataArray.size();
            endIndex += 5;
            if (endIndex > arraySize)
            {
                endIndex = arraySize;
            }
            for (int i = 0; i < arraySize - endIndex; i++) {
                sampleDataArray.remove(sampleDataArray.size() - 1);
            }
        }

        // check if continue forehand actions (1s interval)
        if ((sampleDataArray.get(0).uTime - fLastForehandActionTime) < 1000 && fLastForehandActionTime > 0)
        {
            uContinueForehandActionCount++;
            if (uContinueForehandActionCount > 3)
            {
                bContinueForehandActionRefine = true;
            }
        }
        else
        {
            uContinueForehandActionCount = 0;
            bContinueForehandActionRefine = false;
        }
        fLastForehandActionTime = sampleDataArray.get(sampleDataArray.size() - 1).uTime;

        // optimize the height of trajectory
        if (checkHeightDown(sampleDataArray) && bContinueForehandActionRefine)
        {
            for(SampleData val:sampleDataArray)
            {
                if (val.fLinerAccD > 0)
                {
                    //val.fLinerAccD = -3.0;
                }
            }

        }
    }

    private void backhandRefine(ArrayList<SampleData> sampleDataArray)
    {
        if (sampleDataArray.isEmpty())
        {
            return;
        }

        computeRefineParameters(sampleDataArray);

        if (fPlatformOmegaMaxZ < 6 && Math.abs(fPlatformOmegaMinZ) < 6 && fOmegaMax < 4 && Math.abs(fOmegaMin) < 4)
        {
            pushpullRefine(sampleDataArray);
        }
        else
        {
            // backhand action
            trainData.sActionType = "backhand";
            int startIndex = 0;
            int endIndex = 0;
            int falseIndex = 0;
            ArrayList<SampleData> sampleDataArrayCpy = new ArrayList<SampleData>();

            sampleDataArrayCpy = (ArrayList<SampleData>)sampleDataArray.clone();
            computeRefineParameters(sampleDataArray);
            falseIndex = removeFalsePeak(sampleDataArrayCpy);
            if (falseIndex > 20)
            {
                removeFalsePeak(sampleDataArray);
            }

            if (Math.abs(fOmegaMax) > Math.abs(fOmegaMin))
            {
                fOmegaPeak = fOmegaMax;
                fOmegaLetter = 1;
            }
            else
            {
                fOmegaPeak = fOmegaMin;
                fOmegaLetter = -1;
            }

            // remove the negative gyro Z actions
            for (int i = 0; i < sampleDataArray.size(); i++)
            {
                if (sampleDataArray.get(sampleDataArray.size() - 1 - i).fOmegaB[CHZ] * fOmegaLetter > 0)
                {
                    endIndex = sampleDataArray.size() - 1 - i;
                    break;
                }
            }
            if (endIndex > 0)
            {
                int arraySize = sampleDataArray.size();
                for (int i = 0; i < arraySize - endIndex; i++) {
                    sampleDataArray.remove(sampleDataArray.size() - 1);
                }
            }
            // calculate the ins info firstly
            insStrapdownMechanization(dt, sampleDataArray);
            // remove the negative position actions
            endIndex = 0;
            for(SampleData val:sampleDataArray)
            {
                int lastIndex = sampleDataArray.indexOf(val) - 1;
                if (lastIndex > 0 && val.fPosN < 0)
                {
                    endIndex = sampleDataArray.indexOf(val);
                    break;
                }
            }
            if (endIndex > 0)
            {
                int arraySize = sampleDataArray.size();
                for (int i = 0; i < arraySize - endIndex; i++) {
                    sampleDataArray.remove(sampleDataArray.size() - 1);
                }
            }
        }
    }

    private void pushpullRefine(ArrayList<SampleData> sampleDataArray)
    {
        if (sampleDataArray.isEmpty())
        {
            return;
        }

        // no rotation, it is push action
        trainData.sActionType = "push-pull";
        // remove the false peak
        removeFalsePeak(sampleDataArray);
        // recompute since sample array is changed
        computeRefineParameters(sampleDataArray);

        int startIndex = 0;
        int endIndex = fAccMinIndex;

        for (SampleData val : sampleDataArray)
        {
            if (val.fLinerAccN > 0.75 * fAccMaxX)
            {
                startIndex = sampleDataArray.indexOf(val);
                break;
            }
        }
        // refine the sample data array
        if (startIndex > 0)
        {
            for (int i = 0; i < startIndex; i++)
            {
                sampleDataArray.remove(0);
                endIndex--;
            }
        }
        if (endIndex > 0)
        {
            int arraySize = sampleDataArray.size();
            for (int i = 0; i < arraySize - endIndex - 1; i++)
            {
                sampleDataArray.remove(sampleDataArray.size() - 1);
            }
        }
        // calculate the ins info firstly
        insStrapdownMechanization(dt, sampleDataArray);
        // remove the backward action
        endIndex = 0;
        for(SampleData val:sampleDataArray)
        {
            int lastIndex = sampleDataArray.indexOf(val) - 1;
            if (lastIndex > 0 && val.fPosN < sampleDataArray.get(lastIndex).fPosN)
            {
                endIndex = sampleDataArray.indexOf(val);
                break;
            }
        }
        if (endIndex > 0)
        {
            int arraySize = sampleDataArray.size();
            for (int i = 0; i < arraySize - endIndex; i++) {
                sampleDataArray.remove(sampleDataArray.size() - 1);
            }
        }
    }

    private void computeRefineParameters(ArrayList<SampleData> sampleDataArray)
    {
        if (sampleDataArray.isEmpty())
        {
            return;
        }

        fAccMaxX = Double.MIN_VALUE;
        fAccMinX = Double.MAX_VALUE;
        // record the max liner acc
        for(SampleData val:sampleDataArray)
        {
            double linerAccX = 0;

            // calculate the liner accelerate along the x axis
            linerAccX = val.fAccelerate[0]*val.fCbnPlat[0][0] + val.fAccelerate[1]*val.fCbnPlat[0][1] + val.fAccelerate[2]*val.fCbnPlat[0][2];
            if (linerAccX > fAccMaxX)
            {
                fAccMaxX = linerAccX;
                fAccMaxIndex = sampleDataArray.indexOf(val);
            }

            if (linerAccX < fAccMinX)
            {
                fAccMinX = linerAccX;
                fAccMinIndex = sampleDataArray.indexOf(val);
            }
        }

        // record the max/min platform Omega and body Omega
        fPlatformOmegaMaxZ = Double.MIN_VALUE;
        fPlatformOmegaMinZ = Double.MAX_VALUE;
        fOmegaMax = Double.MIN_VALUE;
        fOmegaMin = Double.MAX_VALUE;
        for (SampleData val:sampleDataArray)
        {
            // platform omega
            if (val.fOmegaN[CHZ] > fPlatformOmegaMaxZ)
            {
                fPlatformOmegaMaxZ = val.fOmegaN[CHZ];
            }
            if (val.fOmegaN[CHZ] < fPlatformOmegaMinZ)
            {
                fPlatformOmegaMinZ = val.fOmegaN[CHZ];
            }

            // body omega
            if (val.fOmegaB[CHZ] > fOmegaMax)
            {
                fOmegaMax = val.fOmegaB[CHZ];
                fOmegaMaxIndex = sampleDataArray.indexOf(val);
            }
            if (val.fOmegaB[CHZ] < fOmegaMin)
            {
                fOmegaMin = val.fOmegaB[CHZ];
                fOmegaMinIndex = sampleDataArray.indexOf(val);
            }
        }
    }

    private int removeFalsePeak(ArrayList<SampleData> sampleDataArray)
    {
        double slopAngle = 0; // unit:Degree
        int startIndex = 0;
        double fLastLinerAccX = 0;
        boolean bCurveRising = true;
        int tempIndex = 0;
        int removeIndex = 0;

        if (sampleDataArray.isEmpty())
        {
            return 0;
        }

        startIndex = 0;
        tempIndex = 0;
        for(SampleData val:sampleDataArray)
        {
            double linerAccX = val.fLinerAccN;

            if (linerAccX < fLastLinerAccX && bCurveRising)
            {
                // reach the peak and check the peak
                if (Math.abs(fLastLinerAccX - fAccMaxX) < 1e-5)
                {
                    // the max peak
                    startIndex = tempIndex;
                    break;
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
                if (linerAccX > fLastLinerAccX)
                {
                    tempIndex = sampleDataArray.indexOf(val) - 1;
                    bCurveRising = true;
                }
            }
            fLastLinerAccX = linerAccX;
        }
        if (startIndex > 0)
        {
            removeIndex += startIndex;
            for (int i = 0; i < startIndex; i++)
            {
                sampleDataArray.remove(0);
            }
        }

        startIndex = 0;
        tempIndex = 0;
        for(SampleData val:sampleDataArray)
        {
            double linerAccX = val.fLinerAccN;
            if (Math.abs(linerAccX - fAccMaxX) < 1)
            {
                // the max peak
                startIndex = tempIndex;
                break;
            }
            int indexCurr = sampleDataArray.indexOf(val);
            if (indexCurr > 0)
            {
                slopAngle = Math.toDegrees(Math.atan(linerAccX - sampleDataArray.get(indexCurr - 1).fLinerAccN));
                if (slopAngle < 15)
                {
                    tempIndex = indexCurr;
                }
            }
        }
        if (startIndex > 0)
        {
            removeIndex += startIndex;
            for (int i = 0; i < startIndex; i++)
            {
                sampleDataArray.remove(0);
            }
        }

        // remove the negative liner acc index
        if (removeIndex > 0)
        {
            startIndex = 0;
            for(SampleData val:sampleDataArray) {
                double linerAccX = val.fLinerAccN;
                if (linerAccX > 3)
                {
                    startIndex = sampleDataArray.indexOf(val);
                    break;
                }
            }
            if (startIndex > 0)
            {
                removeIndex += startIndex;
                for (int i = 0; i < startIndex; i++)
                {
                    sampleDataArray.remove(0);
                }
            }
        }

        return removeIndex;
    }

    private void refineSampleData(ArrayList<SampleData> sampleDataArray)
    {
        if (sampleDataArray.isEmpty())
        {
            return;
        }

        // check the attitude for action distinguish
        double azimuth = 0;
        ArrayList<SampleData> sampleDataArrayCpy = new ArrayList<SampleData>();
        sampleDataArrayCpy = (ArrayList<SampleData>)sampleDataArray.clone();
        computeRefineParameters(sampleDataArray);
        removeFalsePeak(sampleDataArrayCpy);
        for (SampleData val : sampleDataArrayCpy)
        {
            if (val.fLinerAccN > 0)
            {
                azimuth = val.fPsiPlPlat;
                break;
            }
        }

        if (azimuth > 0 || azimuth < (-Math.PI + 0.3))
        {
            // forehand actions
            forehandRefine(sampleDataArray);
        }
        else
        {
            // backhand actions and include the push-pull actions
            backhandRefine(sampleDataArray);
        }

        // check action time and action interval time
        if (sampleDataArray.isEmpty())
        {
            uActionComplete = false;

            return;
        }
        double actionSustainedTime = sampleDataArray.size() * dt;
        double actionIntervalTime = (sampleDataArray.get(0).uTime - iActionEndTimeLast) * dt;
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
        iActionEndTimeLast = sampleDataArray.get(sampleDataArray.size() - 1).uTime;
    }
}


