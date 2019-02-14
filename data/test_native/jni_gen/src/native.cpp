#include <iostream>
#include "fitting.h"
#include "com_company_Main.h"

using namespace std;

JNIEXPORT jdouble JNICALL Java_com_company_Main_splineFitting
(JNIEnv *env, jobject obj, jdoubleArray arrX, jdoubleArray arrY, jint num, jdouble x)
{
    jdouble *carrX = env->GetDoubleArrayElements(arrX, false);
    jdouble *carrY = env->GetDoubleArrayElements(arrY, false);
    double y = 0;
    Eigen::VectorXd xvals(num);
    Eigen::VectorXd yvals(xvals.rows());

    if (carrX == NULL || carrY == NULL)
    {
        return -1;
    }

    if (num != 6)
    {
        return -1;
    }

    xvals << carrX[0], carrX[1], carrX[2], carrX[3], carrX[4], carrX[5];
    yvals << carrY[0], carrY[1], carrY[2], carrY[3], carrY[4], carrY[5];
    SplineFunction s(xvals, yvals);
    y = s(x);

    env->ReleaseDoubleArrayElements(arrX, carrX, 0);
    env->ReleaseDoubleArrayElements(arrY, carrY, 0);

    return y;
}
