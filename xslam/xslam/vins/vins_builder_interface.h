#ifndef XSLAM_VINS_VINS_BUILDER_INTERFACE_H_
#define XSLAM_VINS_VINS_BUILDER_INTERFACE_H_

#include <set>
#include <string>
#include <vector>

#include "xslam/vins/estimator/estimator.h"

namespace xslam {
namespace vins {

class VinsBuilderInterface 
{
public:
    VinsBuilderInterface() {}
    virtual ~VinsBuilderInterface() {}

    VinsBuilderInterface(const VinsBuilderInterface&) = delete;
    VinsBuilderInterface& operator=(const VinsBuilderInterface&) = delete;

    virtual mapping::PoseGraphInterface* pose_graph() = 0;

};

} // namespace vins
} // namespace xslam

#endif // XSLAM_VINS_VINS_BUILDER_INTERFACE_H_