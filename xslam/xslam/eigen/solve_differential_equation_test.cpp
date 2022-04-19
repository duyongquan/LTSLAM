#include "xslam/eigen/solve_differential_equation.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace egein {

TEST(DiffEquationSolver, euler_method)
{
  DiffEquationSolver solver;
  solver.RunEulerMethod();

}

TEST(DiffEquationSolver, runge_kutta_fourth)
{
  DiffEquationSolver solver;
  solver.RunRK4Method();
}

}  // namespace xslam 
}  // namespace egein 
