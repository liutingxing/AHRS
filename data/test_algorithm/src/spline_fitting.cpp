//
// Created by jiangtianyu on 2019/1/30.
//
#include <iostream>
#include <Eigen/Core>
#include <unsupported/Eigen/Splines>


class SplineFunction {
public:
    SplineFunction(Eigen::VectorXd const &x_vec,
                   Eigen::VectorXd const &y_vec)
            : x_min(x_vec.minCoeff()),
              x_max(x_vec.maxCoeff()),
            // Spline fitting here. X values are scaled down to [0, 1] for this.
              spline_(Eigen::SplineFitting<Eigen::Spline<double, 1>>::Interpolate(
                      y_vec.transpose(),
                      // No more than cubic spline, but accept short vectors.

                      std::min<int>(x_vec.rows() - 1, 3),
                      scaled_values(x_vec)))
    { }

    double operator()(double x) const {
        // x values need to be scaled down in extraction as well.
        return spline_(scaled_value(x))(0);
    }

private:
    // Helpers to scale X values down to [0, 1]
    double scaled_value(double x) const {
        return (x - x_min) / (x_max - x_min);
    }

    Eigen::RowVectorXd scaled_values(Eigen::VectorXd const &x_vec) const {
        return x_vec.unaryExpr([this](double x) { return scaled_value(x); }).transpose();
    }

    double x_min;
    double x_max;

    // Spline of one-dimensional "points."
    Eigen::Spline<double, 1> spline_;
};

int main(int argc, char const* argv[])
{
    Eigen::VectorXd xvals(3);
    Eigen::VectorXd yvals(xvals.rows());

    xvals << 0, 15, 30;
    yvals << 0, 12, 17;

    SplineFunction s(xvals, yvals);

    std::cout << s(12.34) << std::endl;
}