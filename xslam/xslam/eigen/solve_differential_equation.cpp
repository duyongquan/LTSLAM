#include "xslam/eigen/solve_differential_equation.h"

#include "glog/logging.h"

namespace xslam {
namespace egein {

// defining ordinary differential equation to be solved
// In this example we are solving dy/dx = x + y
// f'(x) = x^2
#define euler_function(x) (x*x)

#define rk4_function(x)   (x*x)

// f'(x,y) = x+y
// https://www.codesansar.com/numerical-methods/eulers-method-using-cpp-output.htm
void DiffEquationSolver::RunEulerMethod()
{
  double x0 = 0.0f;
  double y0 = 0.0f;
  double yn = 0.0f;

  const double xn = 0.5; // [x0, xn] ~ [0, 0.5]
  const int n = 10; // n steps

  // Calculating step size (h)
  const double h = (xn - x0) / n;
 
  // Euler's Method
  LOG(INFO) << "\nx0\ty0\tslope\tyn\n";
  LOG(INFO) << "------------------------------";


  for(int i = 0; i < n; i++)
  {
    double slope = euler_function(x0);
    yn = y0 + h * slope;
    LOG(INFO)  << x0 <<"\t"<< y0 << "\t" << slope << "\t" << yn;
    y0 = yn;
    x0 = x0 + h;
  }

  // Displaying result
  LOG(INFO) << "Value of y at x = "<< xn << " is " << yn;

}

// https://www.codesansar.com/numerical-methods/runge-kutta-rk-fourth-order-using-cpp-output.htm
// Output of this is program is solution for dy/dx = (y^2 - x^2)/(y^2+x^2) with initial condition 
// y = 1 for x = 0 i.e. y(0) = 1 and we are trying to evaluate this differential equation at y = 0.6 
// in three steps i.e. n = 3. ( Here y = 0.6 i.e. y(0.6) = ? is our calculation point)
// // f'(x) = x^2
void DiffEquationSolver::RunRK4Method()
{
  double x0 = 0.0f;
  double y0 = 0.0f;
  double yn = 0.0f;

  const double xn = 0.5; // [x0, xn] ~ [0, 0.5]
  const int n = 10; // n steps

  // Calculating step size (h)
  const double h = (xn - x0) / n;
 
  // Runge Kutta Method
  LOG(INFO) << "\nx0\ty0\tyn\n";
  LOG(INFO) << "------------------";

  for(int i = 0; i < n; i++)
  {
    auto k1 = h * (rk4_function(x0));
    auto k2 = h * (rk4_function((x0 + h/2)));
    auto k3 = h * (rk4_function((x0 + h/2)));
    auto k4 = h * (rk4_function((x0 + h)));
    auto k = (k1 + 2 * k2 + 2 * k3 + k4) / 6;
    yn = y0 + k;
    LOG(INFO) << x0 << "\t"<< y0<<"\t"<< yn;
    x0 = x0+h;
    y0 = yn;
  }

  // Displaying result
  LOG(INFO) << "Value of y at x = "<< xn << " is " << yn;
}

}  // namespace xslam 
}  // namespace egein 
