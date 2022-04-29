#include "xslam/g2o/pure_newton.h"

namespace xslam{
namespace g2o {

void PureNewtonSolver::RunDemo(bool visualization)
{

}


double PureNewtonSolver::ComputeFunctionValue(const Eigen::Vector2d& input)
{

  // x^2 + 3y^2 + 3xy + 4
  double x = input(0);
  double y = input(1);
  return std::hypot(x, x) + 3 * std::hypot(y, y) + 3 * x * y + 4;
}

Eigen::Vector2d PureNewtonSolver::ComputeJacobianMatrix(const Eigen::Vector2d& input)
{
    // [2x + 3y]
    // [6y + 3x]
    return Eigen::Vector2d {
        2 * input(0) + 3 * input(1),
        3 * input(0) + 6 * input(1)
    };
}


Eigen::Matrix2d PureNewtonSolver::ComputeHessianMatrix(const Eigen::Vector2d& iter_x)
{
    return Eigen::Matrix2d {

    };
}


} // namespace g2o
} // namespace xslam