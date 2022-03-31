#ifndef CARTOGRAPHER_SENSOR_INTERNAL_DISPATCHABLE_H_
#define CARTOGRAPHER_SENSOR_INTERNAL_DISPATCHABLE_H_

#include "xslam/vins/vins_builder_interface.h"
#include "xslam/vins/sensor/data.h"

namespace xslam {
namespace vins {
namespace sensor {

template <typename DataType>
class Dispatchable : public Data 
{
public:
    Dispatchable(const std::string &sensor_id, const DataType &data)
        : Data(sensor_id), data_(data) {}

    common::Time GetTime() const override { return data_.time; }

    void AddToTrajectoryBuilder(VinsBuilderInterface *const vins_builder) override 
    {
        vins_builder->AddSensorData(sensor_id_, data_);
    }

    const DataType &data() const { return data_; }

private:
    const DataType data_;
};

template <typename DataType>
std::unique_ptr<Dispatchable<DataType>> MakeDispatchable(
    const std::string &sensor_id, const DataType &data) 
{
    return std::make_unique<Dispatchable<DataType>>(sensor_id, data);
}

}  // namespace sensor
}  // namespace vins
}  // namespace xslam


#endif  // CARTOGRAPHER_SENSOR_INTERNAL_DISPATCHABLE_H_
