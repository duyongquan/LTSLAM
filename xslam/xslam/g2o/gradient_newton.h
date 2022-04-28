#ifndef XSLAM_G2O_GRADIENT_NEWTON_H
#define XSLAM_G2O_GRADIENT_NEWTON_H

namespace xslam{
namespace g2o {

// https://github.com/TiantianUpup/numerical-optimization

class GradientNewtonSolver
{
public:
  void RunDemo(bool visualization = false);
};

} // namespace g2o
} // namespace xslam

#endif // XSLAM_G2O_GRADIENT_NEWTON_H
