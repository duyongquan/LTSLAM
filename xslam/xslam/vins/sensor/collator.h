#ifndef XSLAM_VINS_SENSOR_COLLATOR_H_
#define XSLAM_VINS_SENSOR_COLLATOR_H_

#include "xslam/vins/sensor/collator_interface.h"
#include "xslam/vins/sensor/ordered_multi_queue.h"

#include <set>

namespace xslam {
namespace vins {
namespace sensor {

class Collator : public CollatorInterface 
{
public:
    Collator() {}

    Collator(const Collator&) = delete;
    Collator& operator=(const Collator&) = delete;

    // Callback handle
    void HandleSensorData(const std::vector<std::string>& sensor_ids, const Callback& callback) override;

    void AddSensorData(std::unique_ptr<Data> data) override;

    void Flush() override;
    
    std::string GetBlockingSensorId() const override;

  private:
    // Queue keys are a pair of trajectory ID and sensor identifier.
    OrderedMultiQueue queue_;
};



}  // namespace sensor
}  // namespace vins
}  // namespace xslam

#endif  // XSLAM_VINS_SENSOR_COLLATOR_H_
