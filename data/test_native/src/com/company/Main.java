package com.company;

public class Main {

    public native double splineFitting(double[] x0, double[] y0, int num, double x);

    public static void main(String[] args) {
	// write your code here
        double[] x0 = {1455, 1456, 1457, 1463, 1464, 1465};
        double[] y0 = {911.8, 984.9, 1028, 1064, 985.9, 897.7};
        Main test = new Main();

        System.out.println(test.splineFitting(x0, y0, 6, 1457));
        System.out.println(test.splineFitting(x0, y0, 6, 1458));
        System.out.println(test.splineFitting(x0, y0, 6, 1459));
        System.out.println(test.splineFitting(x0, y0, 6, 1460));
        System.out.println(test.splineFitting(x0, y0, 6, 1461));
        System.out.println(test.splineFitting(x0, y0, 6, 1462));
        System.out.println(test.splineFitting(x0, y0, 6, 1463));
    }

    static{
        System.loadLibrary("jni_gen");
    }
}
