#include "xslam/vins/vins_builder.h"
#include "xslam/vins/sensor/collator.h"

#include "glog/logging.h"

namespace xslam {
namespace vins {

const std::string kImuTopic = "imu";
const std::string kImageTopic = "image";

VinsBuilder::VinsBuilder(const estimator::proto::EstimatorOptions& options)
    : options_(options)
{
    estimator_ = std::make_unique<estimator::Estimator>(options_);
    collator_ = std::make_unique<sensor::Collator>();
}

void VinsBuilder::AddSensorData(
    const std::string& sensor_id, const sensor::ImuData& imu_data)
{
    auto imu_tracker_ptr = estimator_->imu_tracker();
    imu_tracker_ptr->AddImuData(imu_data);
}

void VinsBuilder::AddSensorData(
    const std::string& sensor_id, const sensor::ImageData& image_data)
{
    auto feature_tracker_ptr = estimator_->feature_tracker();
    feature_tracker_ptr->AddImageData(image_data);
}

bool VinsBuilder::GetNewestFeaturePoints(common::messages::PointCloud& points)
{
    return estimator_->feature_tracker()->GetNewestFeaturePoints(points);
}

std::shared_ptr<VinsBuilder> CreateVinsBuilder(
    const estimator::proto::EstimatorOptions& options)
{ 
    return std::make_shared<VinsBuilder>(options);
}

} // namespace vins
} // namespace xslam