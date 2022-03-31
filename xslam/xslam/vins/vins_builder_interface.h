#ifndef XSLAM_VINS_VINS_BUILDER_INTERFACE_H_
#define XSLAM_VINS_VINS_BUILDER_INTERFACE_H_

#include <set>
#include <string>
#include <vector>

#include "xslam/vins/sensor/imu_data.h"
#include "xslam/vins/sensor/image_data.h"
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

    virtual void AddSensorData(const std::string& sensor_id, const sensor::ImuData& imu_data) = 0;
    virtual void AddSensorData(const std::string& sensor_id, const sensor::ImageData& image_data) = 0;

    virtual estimator::Estimator* estimator() = 0;
};

} // namespace vins
} // namespace xslam

#endif // XSLAM_VINS_VINS_BUILDER_INTERFACE_H_