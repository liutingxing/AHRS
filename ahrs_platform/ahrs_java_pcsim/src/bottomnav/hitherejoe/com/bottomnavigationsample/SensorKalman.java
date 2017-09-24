package bottomnav.hitherejoe.com.bottomnavigationsample;

/**
 * Created by Anakin on 2017/7/13.
 */

import Jama.Matrix;

public class SensorKalman {
    public  int    stateNum;
    public  Matrix x;
    private Matrix F;
    private Matrix PHIM;
    private Matrix qdt;
    private Matrix Q;
    private Matrix G;
    private Matrix P;

    private final static double Corr_time_gyro = 100;
    private final static double Corr_time_acc = 100;
    private final static double sigma_Win = 1.0e-6;
    private final static double sigma_acc = 5.0e-4 * 9.78032667 * 5.0e-4 * 9.78032667;
    private final static double sigma_gyro = 20.0 * Math.PI / 180.0 / 3600 * 20.0 * Math.PI / 180.0 / 3600;
    private final static double sigma_phim_e_n = 1.0 * Math.PI / 180;
    private final static double sigma_phim_u = 1.0 * Math.PI / 180;
    private final static double sigma_phim_gyro = 1000 * Math.PI / 180/3600;
    private final static double sigma_phim_acc = 0.3;

    SensorKalman(int num)
    {
        this.stateNum = num;
        this.x = new Matrix(num, 1, 0);
        this.F = new Matrix(num, num, 0);
        this.PHIM = new Matrix(num, num, 0);
        this.qdt = new Matrix(num, num, 0);
        this.Q = new Matrix(num, num, 0);
        this.G = new Matrix(num, num, 0);
        this.P = new Matrix(num, num, 0);
        this.P.set(0, 0, sigma_phim_e_n*sigma_phim_e_n);
        this.P.set(1, 1, sigma_phim_e_n*sigma_phim_e_n);
        this.P.set(2, 2, sigma_phim_u*sigma_phim_u);
        this.P.set(3, 3, sigma_phim_gyro*sigma_phim_gyro);
        this.P.set(4, 4, sigma_phim_gyro*sigma_phim_gyro);
        this.P.set(5, 5, sigma_phim_gyro*sigma_phim_gyro);
        this.P.set(6, 6, sigma_phim_acc*sigma_phim_acc);
        this.P.set(7, 7, sigma_phim_acc*sigma_phim_acc);
        this.P.set(8, 8, sigma_phim_acc*sigma_phim_acc);
    }

    public void sensorFusionKalman(double dt, double[] acc, double[][] fCbn)
    {
        setPhiQd(dt, fCbn);
        predict();
        accMeasUpdate(acc, fCbn);
    }

    private void accMeasUpdate(double[] acc, double[][] fCbn)
    {
        Matrix H = new Matrix(3, stateNum, 0);
        Matrix R = new Matrix(3, 3, 0);
        Matrix Acc = new Matrix(acc, 3);
        Matrix Cbn = new Matrix(fCbn);
        double[] G = {0, 0, SensorFusion.GRAVITY};
        Matrix Gvector = new Matrix(G, 3);
        Matrix I = Matrix.identity(stateNum, stateNum);
        Matrix Z;
        Matrix K;
        Matrix gEstimate;
        Matrix temp;

        H.set(0, 1, SensorFusion.GRAVITY);
        H.set(1, 0, -SensorFusion.GRAVITY);

        H.set(0, 6, fCbn[0][0]);
        H.set(0, 7, fCbn[0][1]);
        H.set(0, 8, fCbn[0][2]);

        H.set(1, 6, fCbn[1][0]);
        H.set(1, 7, fCbn[1][1]);
        H.set(1, 8, fCbn[1][2]);

        H.set(2, 6, fCbn[2][0]);
        H.set(2, 7, fCbn[2][1]);
        H.set(2, 8, fCbn[2][2]);

        R.set(0, 0, 5*5);
        R.set(1, 1, 5*5);
        R.set(2, 2, 5*5);

        gEstimate = Cbn.times(Acc.times(-1));
        Z = Gvector.minus(gEstimate);
        temp = H.times(P).times(H.transpose()).plus(R).inverse();
        K = P.times(H.transpose()).times(temp);
        x = x.plus(K.times(Z.minus(H.times(x))));
        P = I.minus(K.times(H)).times(P);
    }

    private void predict()
    {
        x = PHIM.times(x);
        P = PHIM.times(P).times(PHIM.transpose()).plus(Q);
    }


    private void setPhiQd(double dt, double[][] fCbn)
    {
        Matrix M1;
        Matrix M2;
        Matrix I;

        // set F matrix
        F.set(0, 3, fCbn[0][0]);
        F.set(0, 4, fCbn[0][1]);
        F.set(0, 5, fCbn[0][2]);

        F.set(1, 3, fCbn[1][0]);
        F.set(1, 4, fCbn[1][1]);
        F.set(1, 5, fCbn[1][2]);

        F.set(2, 3, fCbn[2][0]);
        F.set(2, 4, fCbn[2][1]);
        F.set(2, 5, fCbn[2][2]);

        F.set(3, 3, -1 / Corr_time_gyro);
        F.set(4, 4, -1 / Corr_time_gyro);
        F.set(5, 5, -1 / Corr_time_gyro);

        F.set(6, 6, -1 / Corr_time_acc);
        F.set(7, 7, -1 / Corr_time_acc);
        F.set(8, 8, -1 / Corr_time_acc);

        //set Q matrix
        qdt.set(0, 0, sigma_Win);
        qdt.set(1, 1, sigma_Win);
        qdt.set(2, 2, sigma_Win);

        qdt.set(3, 3, sigma_gyro);
        qdt.set(4, 4, sigma_gyro);
        qdt.set(5, 5, sigma_gyro);

        qdt.set(6, 6, sigma_acc);
        qdt.set(7, 7, sigma_acc);
        qdt.set(8, 8, sigma_acc);

        // set G matrix
        G.set(0, 0, fCbn[0][0]);
        G.set(0, 1, fCbn[0][1]);
        G.set(0, 2, fCbn[0][2]);

        G.set(1, 0, fCbn[1][0]);
        G.set(1, 1, fCbn[1][1]);
        G.set(1, 2, fCbn[1][2]);

        G.set(2, 0, fCbn[2][0]);
        G.set(2, 1, fCbn[2][1]);
        G.set(2, 2, fCbn[2][2]);

        G.set(3, 3, 1);
        G.set(4, 4, 1);
        G.set(5, 5, 1);

        G.set(6, 6, 1);
        G.set(7, 7, 1);
        G.set(8, 8, 1);

        // Q matrix discretization-2 order
        Q = G.times(qdt).times(G.transpose());
        M1 = Q;
        M2 = Q.times(F.transpose()).plus(F.times(Q));
        Q = M1.times(dt).plus(M2.times(0.5*dt*dt));

        // PHIM matrix discretization-2 order
        I = Matrix.identity(stateNum, stateNum);
        PHIM = I.plus(F.times(dt)).plus(F.times(F).times(0.5*dt*dt));
    }
}




















