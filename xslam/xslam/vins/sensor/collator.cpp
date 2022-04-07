#include "xslam/vins/sensor/collator.h"

namespace xslam {
namespace vins {
namespace sensor {

void Collator::HandleSensorData(const std::vector<std::string>& sensor_ids, const Callback& callback)
{
  for (const auto& sensor_id : sensor_ids) 
  {
      const auto queue_key = QueueKey{sensor_id};
      queue_.AddQueue(queue_key, [callback, sensor_id](std::unique_ptr<Data> data) 
      {
          callback(sensor_id, std::move(data));
      });
  }
}

void Collator::AddSensorData(std::unique_ptr<Data> data) 
{
    QueueKey queue_key{data->GetSensorId()};
    queue_.Add(std::move(queue_key), std::move(data));
}

void Collator::Flush() 
{   
    queue_.Flush(); 
}

std::string Collator::GetBlockingSensorId() const 
{
    return queue_.GetBlocker().sensor_id;
}


}  // namespace sensor
}  // namespace vins
}  // namespace xslam
