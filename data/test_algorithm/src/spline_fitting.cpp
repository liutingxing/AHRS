//
// Created by jiangtianyu on 2019/1/30.
//
#include <iostream>
#include <Eigen/Core>
#include <unsupported/Eigen/Splines>

using namespace Eigen;

double uvalue(double x, double low, double high)
{
    return (x - low)/(high-low);
}

VectorXd uvalues(VectorXd xvals)
{
    const double low = xvals.minCoeff();
    const double high = xvals.maxCoeff();
    for (int i=0; i<xvals.size(); ++i)
    {
        xvals(i) = uvalue(xvals(i), low, high);
    }
    return xvals;
}

int main(int argc, char* argv[])
{
    typedef Spline<double,1> Spline2d;

    const VectorXd xvals = (VectorXd(5) << 1,2,3,4,6).finished();
    const VectorXd yvals = xvals.array().square();
    const Spline2d spline = SplineFitting<Spline2d>::Interpolate(yvals.transpose(), 3, uvalues(xvals).transpose());

    const double step = 0.1;
    for (double x = 1; x < 6 + 0.5*step; x += step)
    {
        std::cout << "(" << x << "," << spline(uvalue(x, xvals.minCoeff(), xvals.maxCoeff())).transpose() << ")\n";
    }
    std::cout << std::endl;
}