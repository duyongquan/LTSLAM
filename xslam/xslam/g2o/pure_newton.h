#ifndef XSLAM_G2O_PURE_NEWTON_H
#define XSLAM_G2O_PURE_NEWTON_H

namespace xslam{
namespace g2o {

// https://github.com/TiantianUpup/numerical-optimization

class PureNewtonSolver
{
public:
  void RunDemo(bool visualization = false);
};

} // namespace g2o
} // namespace xslam

#endif // XSLAM_G2O_PURE_NEWTON_H
