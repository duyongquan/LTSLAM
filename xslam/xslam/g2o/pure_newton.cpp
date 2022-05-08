#include "xslam/g2o/pure_newton.h"

#include <vector>
#include <map>

namespace xslam{
namespace g2o {

void PureNewtonSolver::RunDemo(bool visualization)
{
    Eigen::Vector2d x0 {1, 1};
    bool ok = NewtonMethod(x0);

    if (visualization) {
        DrawContour();
    }
}

bool PureNewtonSolver::NewtonMethod(const Eigen::Vector2d& x0)
{
    const double kEpsilon = 1e-8;

    Eigen::Vector2d x = x0;
    Eigen::Vector2d gval = ComputeJacobianMatrix(x); // 2 x 1
    Eigen::Matrix2d hval = ComputeHessianMatrix(x);  // 2 x 2

    Eigen::Vector2d d =  hval.inverse() * gval;
    int iter = 0;

    bool result = true;
    while (gval.norm() > kEpsilon && iter < 10000)
    {
        iter++;
        x = x - d;
        std::cout << "iter = " << iter << " f(x) = { val: " <<  ComputeFunctionValue(x) << " }" << std::endl;
        gval = ComputeJacobianMatrix(x);
        hval = ComputeHessianMatrix(x);
        d =  hval.inverse() * gval;

        double func_value = ComputeFunctionValue(x);
        if (isnan(func_value)) {
            std::cout << "func_value =  " << func_value << std::endl;
            result = false;
            break;
        }

        if (iter == 10000) {
            result = false;
            std::cout << "did not converge" << std::endl;
        }
    }
    return result;
}

void PureNewtonSolver::DrawContour()
{
    std::vector<std::vector<double>> x, y, z;
    
    for (double i = -5; i <= 5;  i += 0.25) 
    {
        std::vector<double> x_row, y_row, z_row;
        for (double j = -5; j <= 5; j += 0.25) {
            x_row.push_back(i);
            y_row.push_back(j);
            z_row.push_back(ComputeFunctionValue(Eigen::Vector2d{i, j}));
        }
        x.push_back(x_row);
        y.push_back(y_row);
        z.push_back(z_row);
    }

    std::map<std::string, std::string> keywords;
    keywords.insert(std::pair<std::string, std::string>("colors", "gray") );

	matplotlibcpp::contour(x, y, z);
    matplotlibcpp::plot_surface(x, y, z);
    matplotlibcpp::show();
}

double PureNewtonSolver::ComputeFunctionValue(const Eigen::Vector2d& input)
{
  // f(x, y) = (x+1)^2 + (y+1)^2
  double x = input(0) ;
  double y = input(1);
  return std::hypot(x+1, x+1) + std::hypot(y+1, y+1);
}

Eigen::Vector2d PureNewtonSolver::ComputeJacobianMatrix(const Eigen::Vector2d& input)
{
    // [2x]
    // [2y]
    return Eigen::Vector2d {
        2 * input(0) ,
        2 * input(1)
    };
}


Eigen::Matrix2d PureNewtonSolver::ComputeHessianMatrix(const Eigen::Vector2d& input)
{
    return Eigen::Matrix2d {
        // 2, 0,
        // 0, 2
    };
}

} // namespace g2o
} // namespace xslam