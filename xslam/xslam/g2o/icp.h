
#ifndef XSLAM_G2O_ICP_H
#define XSLAM_G2O_ICP_H

#include <vector>
#include <string>

#include <Eigen/Core>

namespace xslam {
namespace g2o {

class ICP
{
public:

    enum class Mode
    {
        kPoint2Point,
        kPoint2Plane
    };

    void RunDemo(const Mode& mode);

private:

    // point
    void ICPPoint2Point();

    // plane
    void ICPPoint2Plane();

};

} // namespace g2o
} // namespace xslam


#endif // XSLAM_G2O_ICP_H
