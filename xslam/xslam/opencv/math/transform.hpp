#ifndef XSLAM_OPENCV_MATH_TRANSFORM_HPP
#define XSLAM_OPENCV_MATH_TRANSFORM_HPP

#include "xslam/opencv/math/matrix.hpp"
#include "xslam/opencv/math/matrix_svd.hpp"

namespace xslam {
namespace opencv {
namespace math {

/*
 * Determines the transformation between two lists of corresponding points.
 * Minimizing sum_i (R * s * p0[i] + t - p1[i])^2
 */
template <typename T, int N>
bool
determine_transform(std::vector<Vector<T, N>> const& p0,
    std::vector<Vector<T, N>> const& p1,
    Matrix<T, N, N>* rot, T* scale, Vector<T, N>* trans);

template <typename T, int N>
bool
determine_transform(std::vector<Vector<T, N>> const& p0,
    std::vector<Vector<T, N>> const& p1,
    Matrix<T, N, N>* rot, T* scale, Vector<T, N>* trans)
{
    if (p0.size() != p1.size())
        throw std::invalid_argument("Dimension size mismatch");

    std::size_t num_correspondences = p0.size();
    if (num_correspondences < 3)
        throw std::invalid_argument("At least three correspondences required");

    /* Calculate centroids. */
    Vector<T, N> c0(0.0), c1(0.0);
    for (std::size_t i = 0; i < num_correspondences; ++i)
    {
        c0 += p0[i];
        c1 += p1[i];
    }
    c0 /= static_cast<T>(num_correspondences);
    c1 /= static_cast<T>(num_correspondences);

    /* Calculate covariance and variance. */
    T sigma2(0.0);
    Matrix<T, N, N> cov(0.0);
    for (std::size_t i = 0; i < num_correspondences; ++i)
    {
        Vector<T, N> pc0 = p0[i] - c0;
        Vector<T, N> pc1 = p1[i] - c1;
        cov += Matrix<T, N, 1>(pc0.begin()) *
            Matrix<T, 1, N>(pc1.begin());
        sigma2 += pc0.square_norm();
    }
    cov /= static_cast<T>(num_correspondences);
    sigma2 /= static_cast<T>(num_correspondences);

    /* Determine rotation and scale. */
    Matrix<T, N, N> U, S, V;
    matrix_svd(cov, &U, &S, &V);
    if (S(N - 1, N - 1) < T(MATH_SVD_DEFAULT_ZERO_THRESHOLD))
        return false;

    Matrix<T, N, N> R = V * U.transposed();
    T s = matrix_trace(S) / sigma2;

    /* Handle improper rotations (reflections). */
    if (matrix_determinant(R) < T(0.0))
    {
        Matrix<T, N, N> F;
        matrix_set_identity(&F);
        F(N - 1, N - 1) = T(-1.0);
        s = matrix_trace(S * F) / sigma2;
        R = V * F * U.transposed();
    }

    if (rot != nullptr)
        *rot = R;

    if (scale != nullptr)
        *scale = s;

    if (trans != nullptr)
        *trans = c1 + (-R * s * c0);

    return true;
}

} // namespace math
} // namespace opencv
} // namespace xslam

#endif // XSLAM_OPENCV_MATH_TRANSFORM_HPP