#ifndef XSLAM_G2O_INCOMPLETE_CHOLESKY_H
#define XSLAM_G2O_INCOMPLETE_CHOLESKY_H

namespace xslam{
namespace g2o {

// https://github.com/TiantianUpup/numerical-optimization

class IncompleteCholeskySolver
{
public:
  void RunDemo(bool visualization = false);
};

} // namespace g2o
} // namespace xslam

#endif // XSLAM_G2O_INCOMPLETE_CHOLESKY_H
