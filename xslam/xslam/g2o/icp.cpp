#include "xslam/g2o/icp.h"

#include "glog/logging.h"

namespace xslam {
namespace g2o {

void ICP::RunDemo(const Mode& mode)
{
  mode == Mode::kPoint2Plane ? ICPPoint2Plane() : ICPPoint2Point();
}


void ICP::ICPPoint2Point()
{

}

void ICP::ICPPoint2Plane()
{

}


} // namespace g2o
} // namespace xslam
