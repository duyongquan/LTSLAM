#ifndef XSLAM_G2O_MATRIX_CONJUGATE_GRADIENT_H
#define XSLAM_G2O_MATRIX_CONJUGATE_GRADIENT_H

namespace xslam{
namespace g2o {

// https://github.com/TiantianUpup/numerical-optimization

class MatrixConjugateGradientSolver
{
public:
  void RunDemo(bool visualization = false);
};

} // namespace g2o
} // namespace xslam

#endif // XSLAM_G2O_MATRIX_CONJUGATE_GRADIENT_H
