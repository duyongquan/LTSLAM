#ifndef XSLAM_EIGEN_SOLVE_DIFFERENTIAL_EQUATION_H
#define XSLAM_EIGEN_SOLVE_DIFFERENTIAL_EQUATION_H

namespace xslam {
namespace egein {

class DiffEquationSolver
{
public:
  void RunEulerMethod();
  void RunRK4Method();
};

}  // namespace xslam 
}  // namespace egein 


#endif // XSLAM_EIGEN_SOLVE_DIFFERENTIAL_EQUATION_H