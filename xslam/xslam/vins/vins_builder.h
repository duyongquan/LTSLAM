#ifndef XSLAM_VINS_VINS_BUILDER_H_
#define XSLAM_VINS_VINS_BUILDER_H_

#include <memory>

#include "xslam/vins/vins_options.h"
#include "xslam/vins/sensor/imu_data.h"
#include "xslam/vins/sensor/image_data.h"
#include "xslam/vins/sensor/collator_interface.h"
#include "xslam/vins/vins_builder_interface.h"

namespace xslam {
namespace vins {

class VinsBuilder : public VinsBuilderInterface 
{
public:
    explicit VinsBuilder(const estimator::proto::EstimatorOptions& options);
    ~VinsBuilder() override {} 

    void AddSensorData(const std::string& sensor_id, const sensor::ImuData& imu_data) override;

    void AddSensorData(const std::string& sensor_id, const sensor::ImageData& image_data) override;

    bool GetNewestFeaturePoints(common::messages::PointCloud& points);


    estimator::Estimator* estimator() override 
    {
        return estimator_.get();
    }

private:

    estimator::proto::EstimatorOptions options_;
    std::unique_ptr<estimator::Estimator> estimator_;
    std::unique_ptr<sensor::CollatorInterface> collator_;
};

std::shared_ptr<VinsBuilder> CreateVinsBuilder(
    const estimator::proto::EstimatorOptions& options);

} // namespace vins
} // namespace xslam

#endif // XSLAM_VINS_VINS_BUILDER_H_