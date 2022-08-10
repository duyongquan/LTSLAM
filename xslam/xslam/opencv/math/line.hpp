#ifndef XSLAM_OPENCV_MATH_LINE_HPP
#define XSLAM_OPENCV_MATH_LINE_HPP

#include <algorithm>
#include <functional>
#include <stdexcept>
#include <numeric>
#include <cmath>
#include <ostream>

#include "xslam/opencv/math/defines.hpp"
#include "xslam/opencv/math/vector.hpp"

namespace xslam {
namespace opencv {
namespace math {

template <class T>
class Line3;

typedef Line3<float> Line3f;
typedef Line3<double> Line3d;

/**
 * Class that represents a line using a point and a vector.
 * The normal is expected to have unit length.
 */
template <class T>
class Line3
{
public:
    /** Creates an uninitialized line. */
    Line3 (void);

    /** Creates a line with normal n and distance d from the origin. */
    Line3 (Vector<T, 3> const& d, Vector<T, 3> const& p);

public:
    Vector<T, 3> d;
    Vector<T, 3> p;
};

/* ---------------------------------------------------------------- */

template <class T>
inline
Line3<T>::Line3 (void)
{
}

template <class T>
inline
Line3<T>::Line3 (Vector<T, 3> const& d, Vector<T, 3> const& p)
    : d(d), p(p)
{
}

} // namespace math
} // namespace opencv
} // namespace xslam

#endif // XSLAM_OPENCV_MATH_LINE_HPP