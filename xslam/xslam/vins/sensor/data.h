#ifndef XSLAM_VINS_MAPPING_DATA_H_
#define XSLAM_VINS_MAPPING_DATA_H_

#include "xslam/vins/common/time.h"
#include "xslam/vins/transform/rigid_transform.h"

namespace xslam {
namespace vins {


class VinsBuilderInterface;


namespace sensor {

class Data 
{
public:
    explicit Data(const std::string &sensor_id) : sensor_id_(sensor_id) {}
    virtual ~Data() {}

    virtual common::Time GetTime() const = 0;
    const std::string &GetSensorId() const { return sensor_id_; }
    virtual void AddToTrajectoryBuilder(VinsBuilderInterface *trajectory_builder) = 0;

protected:
    const std::string sensor_id_;
};

}  // namespace sensor
}  // namespace vins
}  // namespace xslam

#endif  // XSLAM_VINS_MAPPING_DATA_H_
