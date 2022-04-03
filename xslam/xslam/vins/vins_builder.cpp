#include "xslam/vins/vins_builder.h"

#include "glog/logging.h"

namespace xslam {
namespace vins {

VinsBuilder::VinsBuilder(const estimator::proto::VinsBuilderOptions& options)
    : options_(options)
{
    LOG(INFO) << "VinsBuilder ";
}

void VinsBuilder::AddSensorData(const std::string& sensor_id, const sensor::ImuData& imu_data)
{
    LOG(INFO) << "sensor_id --> " << sensor_id;
}

void VinsBuilder::AddSensorData(const std::string& sensor_id, const sensor::ImageData& image_data)
{
    LOG(INFO) << "sensor_id --> " << sensor_id;
}

std::shared_ptr<VinsBuilderInterface> CreateVinsBuilder(
    const estimator::proto::VinsBuilderOptions& options)
{ 
    return std::make_shared<VinsBuilder>(options);
}

} // namespace vins
} // namespace xslam