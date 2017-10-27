package bottomnav.hitherejoe.com.bottomnavigationsample;

/**
 * Created by Anakin on 2017/7/11.
 */

import java.util.ArrayList;
import Jama.Matrix;

public class SensorFusion {
    public int           uTime;
    private double       fPsiPl;
    private double       fThePl;
    private double       fPhiPl;
    private double[][]   fCnb = new double[3][3];
    private double[][]   fCbn = new double[3][3];
    private double[]     fqPl = new double[4];
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

    private boolean     uStaticFlag;
    private boolean     uAlignFlag;
    private boolean     uKalmanFusionFlag;
    private boolean     uMechanizationFlag;
    private boolean     uActionStartFlag;
    private boolean     uActionEndFlag;
    public  boolean     uActionComplete;

    private final static int ALIGN_NUM = 100;
    private ArrayList<double[]> fAlignGyroArray = new ArrayList<double[]>(100);
    private ArrayList<double[]> fAlignAccArray = new ArrayList<double[]>(100);

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

	private SensorKalman sensorKalman;
    private StringBuffer sAttitude;
    private StringBuffer sTrajectory;
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
        sTrajectory = new StringBuffer();
        sensorKalman = new SensorKalman(STATE_NUM);
        trainData = new TrainData();
    }

    public String sensorFusionExec(int time, double[] gyro, double[] acc)
    {
        double dt = 1.0 / SAMPLE_RATE;

        if (uActionComplete == true)
        {
            uActionComplete = false;
            sTrajectory = new StringBuffer();
            fLinerAccXLast = 0;
            fPlatformOmegaMaxZ = 0;
            fPlatformOmegaMinZ = 0;
            fRangeMax = 0.0;
            fVelocityMax = 0.0;

            trainData.bValid = false;
            trainData.sTrajectory = null;
            trainData.sActionType = null;
            trainData.fRangeMax = 0;
            trainData.fVelocityMax = 0;
            trainData.sTrajectorySweet = "perfect";
            trainData.sStrikeSweet = "perfect";
            trainData.uStrikePower = 0;
            trainData.uPlayLoad = 0;
        }

        // static detection
        uStaticFlag = staticDetect(gyro, acc);

        if (uAlignFlag == false) {
            if (uStaticFlag == true) {
                // initial alignment
                if (sensorAlignment(fAlignAccArray) == true) {
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
        ahrsProcess(dt, gyro, acc);

        // action detect
        actionDetect(dt, gyro, acc);

        // system condition change
        systemConditionSet();

        if (uMechanizationFlag == true)
        {
            int i = 0;
            double[] fPlatformOmega = new double[]{0.0, 0.0, 0.0};

            // ins mechanization
            insStrapdownMechanization(dt, acc);

            // platform omega
            for (i = 0; i < 3; i++)
            {
                fPlatformOmega[i] = fCbn[i][0] * gyro[0] + fCbn[i][1] * gyro[1] + fCbn[i][2] * gyro[2];
            }

            if (fPlatformOmega[2] > fPlatformOmegaMaxZ)
            {
                fPlatformOmegaMaxZ = fPlatformOmega[2];
            }

            if (fPlatformOmega[2] < fPlatformOmegaMinZ)
            {
                fPlatformOmegaMinZ = fPlatformOmega[2];
            }

            sTrajectory.append(String.valueOf(uTime));
            sTrajectory.append(" ");
            sTrajectory.append(String.valueOf(fPosN));
            sTrajectory.append(" ");
            sTrajectory.append(String.valueOf(-fPosD));
            sTrajectory.append(" ");
            sTrajectory.append(String.valueOf(-fPosE));
            sTrajectory.append(" ");
            sTrajectory.append(String.valueOf(fqPl[0]));
            sTrajectory.append(" ");
            sTrajectory.append(String.valueOf(-fqPl[1]));
            sTrajectory.append(" ");
            sTrajectory.append(String.valueOf(-fqPl[2]));
            sTrajectory.append(" ");
            sTrajectory.append(String.valueOf(fqPl[3]));
            sTrajectory.append(" ");
            sTrajectory.append("x");
            sTrajectory.append("\n");
        }

        // record the attitude and trajectory
        if (uActionComplete == true)
        {
            int typeIndex = -1;

            // delete the last enter characcter
            sTrajectory.deleteCharAt(sTrajectory.length() - 1);

            // update train data
            trainData.bValid = true;
            trainData.uActionCount ++;
            trainData.fRangeMax = fRangeMax;
            trainData.fVelocityMax = fVelocityMax;

            if (Math.abs(fPlatformOmegaMaxZ) > Math.abs(fPlatformOmegaMinZ))
            {
                trainData.sActionType = "backhand";
            }
            else
            {
                trainData.sActionType = "forehand";
            }

            // replace the type character in trajectory
            while ((typeIndex = sTrajectory.indexOf("x")) != -1)
            {
                sTrajectory.replace(typeIndex, typeIndex + 1, trainData.sActionType.substring(0,1));
            }
            trainData.sTrajectory = String.valueOf(sTrajectory);
        }

        sAttitude.append(String.valueOf(uTime));
        sAttitude.append(" ");
        sAttitude.append(String.valueOf(0));
        sAttitude.append(" ");
        sAttitude.append(String.valueOf(0));
        sAttitude.append(" ");
        sAttitude.append(String.valueOf(0));
        sAttitude.append(" ");
        sAttitude.append(String.valueOf(fqPl[0]));
        sAttitude.append(" ");
        sAttitude.append(String.valueOf(-fqPl[1]));
        sAttitude.append(" ");
        sAttitude.append(String.valueOf(-fqPl[2]));
        sAttitude.append(" ");
        sAttitude.append(String.valueOf(fqPl[3]));
        sAttitude.append(" ");
        sAttitude.append(String.valueOf(fLinerAccXLast));
        sAttitude.append(" ");
        sAttitude.append("x");

        return String.valueOf(sAttitude);
    }

    private void ahrsProcess(double dt, double[] gyro, double[] acc)
    {
        int i = 0;
        double accNorm = 0;
        double[] qDot = new double[]{0, 0, 0, 0};

        qDot[0] = -(gyro[0] * fqPl[1] + gyro[1] * fqPl[2] + gyro[2] * fqPl[3]) / 2.0;
        qDot[1] =  (gyro[0] * fqPl[0] + gyro[2] * fqPl[2] - gyro[1] * fqPl[3]) / 2.0;
        qDot[2] =  (gyro[1] * fqPl[0] - gyro[2] * fqPl[1] + gyro[0] * fqPl[3]) / 2.0;
        qDot[3] =  (gyro[2] * fqPl[0] + gyro[1] * fqPl[1] - gyro[0] * fqPl[2]) / 2.0;

        accNorm = Math.sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
        if (accNorm < 12.0)
        {
            // execute the acc aid process
            double diff = 0;
            double gyroMeasError = 10 * Math.PI / 180; // gyroscope measurement error in rad/s (shown as 10 deg/s)
            double beta = Math.sqrt(3.0 / 4.0) * gyroMeasError;
            double[] gEstimate = new double[3];
            Matrix F = new Matrix(3, 1);
            Matrix J = new Matrix(3, 4);
            Matrix step = new Matrix(4, 1);

            gEstimate[0] = -acc[0]/accNorm;
            gEstimate[1] = -acc[1]/accNorm;
            gEstimate[2] = -acc[2]/accNorm;

            F.set(0, 0, 2*(fqPl[1]*fqPl[3] - fqPl[0]*fqPl[2]) - gEstimate[0]);
            F.set(1, 0, 2*(fqPl[0]*fqPl[1] - fqPl[2]*fqPl[3]) - gEstimate[1]);
            F.set(2, 0, 2*(0.5-fqPl[1]*fqPl[1] - fqPl[2]*fqPl[2]) - gEstimate[2]);

            J.set(0, 0, -2*fqPl[2]);
            J.set(0, 1, 2*fqPl[3]);
            J.set(0, 2, -2*fqPl[0]);
            J.set(0, 3, 2*fqPl[1]);

            J.set(1, 0, 2*fqPl[1]);
            J.set(1, 1, 2*fqPl[0]);
            J.set(1, 2, 2*fqPl[3]);
            J.set(1, 3, 2*fqPl[2]);

            J.set(2, 0, 0);
            J.set(2, 1, -4*fqPl[1]);
            J.set(2, 2, -4*fqPl[2]);
            J.set(2, 3, 0);

            step = J.transpose().times(F);
            step = step.times(1.0/step.norm2());

            diff = F.norm2();
            if (diff < 1)
            {
                gyroMeasError = 0.1 * Math.PI / 180;
                beta = Math.sqrt(3.0 / 4.0) * gyroMeasError;
            }

            qDot[0] -= beta * step.get(0, 0);
            qDot[1] -= beta * step.get(1, 0);
            qDot[2] -= beta * step.get(2, 0);
            qDot[3] -= beta * step.get(3, 0);
        }

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

            // clear Trajectory
            sTrajectory = new StringBuffer();
            trainData.fVelocityMax = 0;
            trainData.fRangeMax = 0;
            fPlatformOmegaMaxZ = 0;
            fPlatformOmegaMinZ = 0;
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
            linerAccIBP[i] = acc[0]*fCbn[i][0] + acc[1]*fCbn[i][1] + acc[2]*fCbn[i][2];
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

        fRangeMax += Math.sqrt(deltaN*deltaN + deltaE*deltaE + deltaD*deltaD);
        velCurrent = Math.sqrt(fVelN*fVelN + fVelE*fVelE + fVelD*fVelD);
        if (velCurrent > trainData.fVelocityMax)
        {
            fVelocityMax = velCurrent;
        }
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
            linerAccIBP[i] = acc[0]*fCbn[i][0] + acc[1]*fCbn[i][1] + acc[2]*fCbn[i][2];
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
                if (linerAccX > 5){
                    uActionStartFlag = true;
                    iCurveCondition = Step1;
                    actionTime = 0;

                    // add origin point data
                    sTrajectory.append(String.valueOf(uTime));
                    sTrajectory.append(" ");
                    sTrajectory.append(String.valueOf(fPosN));
                    sTrajectory.append(" ");
                    sTrajectory.append(String.valueOf(-fPosD));
                    sTrajectory.append(" ");
                    sTrajectory.append(String.valueOf(-fPosE));
                    sTrajectory.append(" ");
                    sTrajectory.append(String.valueOf(fqPl[0]));
                    sTrajectory.append(" ");
                    sTrajectory.append(String.valueOf(-fqPl[1]));
                    sTrajectory.append(" ");
                    sTrajectory.append(String.valueOf(-fqPl[2]));
                    sTrajectory.append(" ");
                    sTrajectory.append(String.valueOf(fqPl[3]));
                    sTrajectory.append(" ");
                    sTrajectory.append("x");
                    sTrajectory.append("\n");
                }
                break;

            case Step1:
                actionTime += dt;
                if (linerAccX > fLinerAccXLast){
                    slop = 1;
                }else{
                    slop = -1;
                    // reach the up peak
                    if (fLinerAccXLast < 12){
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
                if (downTime > 0.2)
                {
                    iCurveCondition = Peace;
                    uActionStartFlag = false;
                }
                if (linerAccX > fLinerAccXLast){
                    slop = 1;
                    // reach the trough
                    if (fLinerAccXLast > -12){
                        // false trough
                        // no action, because it is normal
                    }else{
                        iCurveCondition = Step3;
                    }
                }else{
                    slop = -1;
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
                    if (actionTime > 0.1 && actionTime < 0.4)
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

    private boolean sensorAlignment(ArrayList<double[]> accArray)
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

    private boolean staticDetect(double[] gyro, double[] acc)
    {
        double gyro_det = 0;
        double acc_det = 0;
        double gyro_std = 0;
        double acc_std = 0;

        if (fAlignGyroArray.size() < ALIGN_NUM)
        {
            fAlignAccArray.add(acc);
            fAlignGyroArray.add(gyro);
        }
        else
        {
            fAlignAccArray.remove(0);
            fAlignGyroArray.remove(0);
            fAlignAccArray.add(acc);
            fAlignGyroArray.add(gyro);
        }

        if (fAlignGyroArray.size() == ALIGN_NUM)
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
