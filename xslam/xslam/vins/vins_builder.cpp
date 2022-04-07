#include "xslam/vins/vins_builder.h"
#include "xslam/vins/sensor/collator.h"

#include "glog/logging.h"

namespace xslam {
namespace vins {

const std::string kImuTopic = "imu";
const std::string kImageTopic = "image";

VinsBuilder::VinsBuilder(const estimator::proto::VinsBuilderOptions& options)
    : options_(options)
{
    // estimator_ = std::make_unique<estimator::Estimator>();
    collator_ = std::make_unique<sensor::Collator>();

    // Register Callback functon handle sensor data
    std::vector<std::string> sensor_ids {kImuTopic, kImageTopic};
    collator_->HandleSensorData(sensor_ids, 
        [this](const std::string& sensor_id, std::unique_ptr<sensor::Data> data) 
        {
            HandleCollatedSensorData(sensor_id, std::move(data));
        });

}

void VinsBuilder::AddSensorData(
    const std::string& sensor_id, const sensor::ImuData& imu_data)
{
    collator_->AddSensorData(sensor::MakeDispatchable(sensor_id, imu_data));
}

void VinsBuilder::AddSensorData(
    const std::string& sensor_id, const sensor::ImageData& image_data)
{
    collator_->AddSensorData(sensor::MakeDispatchable(sensor_id, image_data));
}

void VinsBuilder::HandleCollatedSensorData(
    const std::string& sensor_id, std::unique_ptr<sensor::Data> data)
{
    if (sensor_id == kImuTopic) {
        LOG(INFO) << "kImuTopic sensor_id : " << sensor_id;
    } else if (sensor_id == kImageTopic) {
        LOG(INFO) << "kImageTopic sensor_id : " << sensor_id;
    }
}

std::shared_ptr<VinsBuilderInterface> CreateVinsBuilder(
    const estimator::proto::VinsBuilderOptions& options)
{ 
    return std::make_shared<VinsBuilder>(options);
}

} // namespace vins
} // namespace xslam