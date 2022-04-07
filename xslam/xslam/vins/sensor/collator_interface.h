#ifndef XSLAM_VINS_SENSOR_COLLATOR_INTERFACE_H_
#define XSLAM_VINS_SENSOR_COLLATOR_INTERFACE_H_

#include <functional>
#include <memory>
#include <vector>

#include "xslam/vins/sensor/data.h"

namespace xslam {
namespace vins {
namespace sensor {

class CollatorInterface 
{
public:
    using Callback = std::function<void(const std::string&, std::unique_ptr<Data>)>;

    CollatorInterface() {}
    virtual ~CollatorInterface() {}

    CollatorInterface(const CollatorInterface&) = delete;
    CollatorInterface& operator=(const CollatorInterface&) = delete;

    // Callback
    virtual void HandleSensorData(const std::vector<std::string>& sensor_ids, const Callback& callback) = 0;

    // Adds 'data' to be collated. 'data' must contain valid
    // sensor data. Sensor packets with matching 'data.sensor_id_' must be added
    // in time order.
    virtual void AddSensorData(std::unique_ptr<Data> data) = 0;

    // Dispatches all queued sensor packets. May only be called once.
    // AddSensorData may not be called after Flush.
    virtual void Flush() = 0;

    virtual std::string GetBlockingSensorId() const = 0;
};

}  // namespace sensor
}  // namespace vins
}  // namespace xslam

#endif  // XSLAM_VINS_SENSOR_COLLATOR_INTERFACE_H_