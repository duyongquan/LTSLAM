#ifndef XSLAM_VINS_VINS_BUILDER_H_
#define XSLAM_VINS_VINS_BUILDER_H_

#include <memory>

#include "xslam/vins/sensor/imu_data.h"
#include "xslam/vins/sensor/image_data.h"
#include "xslam/vins/sensor/collator_interface.h"
#include "xslam/vins/vins_builder_interface.h"
#include "xslam/vins/estimator/proto/vins_builder_options.pb.h"

namespace xslam {
namespace vins {

class VinsBuilder : public VinsBuilderInterface 
{
public:
    explicit VinsBuilder(const estimator::proto::VinsBuilderOptions &options);
    ~VinsBuilder() override {} 
  
    void AddSensorData(const std::string& sensor_id, const sensor::ImuData& imu_data) override;

    void AddSensorData(const std::string& sensor_id, const sensor::ImageData& image_data) override;

    estimator::Estimator* estimator() override 
    {
        return estimator_.get();
    }

private:
    const estimator::proto::VinsBuilderOptions options_;
    std::unique_ptr<estimator::Estimator> estimator_;
    std::unique_ptr<sensor::CollatorInterface> sensor_collator_;
};

std::shared_ptr<VinsBuilderInterface> CreateVinsBuilder(
    const estimator::proto::VinsBuilderOptions& options);

} // namespace vins
} // namespace xslam

#endif // XSLAM_VINS_VINS_BUILDER_H_