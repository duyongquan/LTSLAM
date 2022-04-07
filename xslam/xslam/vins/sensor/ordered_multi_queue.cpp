#include "xslam/vins/sensor/ordered_multi_queue.h"

#include <algorithm>
#include <sstream>
#include <vector>
#include <memory>

#include "glog/logging.h"

namespace xslam {
namespace vins {
namespace sensor {

namespace {

// Number of items that can be queued up before we log which queues are waiting
// for data.
const int kMaxQueueSize = 500;

}  // namespace

inline std::ostream& operator<<(std::ostream& out, const QueueKey& key) 
{
    return out << '(' << key.sensor_id << ')';
}

OrderedMultiQueue::OrderedMultiQueue() {}

OrderedMultiQueue::~OrderedMultiQueue() 
{
    for (auto& entry : queues_) {
        CHECK(entry.second.finished);
    }
}

void OrderedMultiQueue::AddQueue(const QueueKey& queue_key, Callback callback) 
{
    CHECK_EQ(queues_.count(queue_key), 0);
    queues_[queue_key].callback = std::move(callback);
}

void OrderedMultiQueue::MarkQueueAsFinished(const QueueKey& queue_key) 
{
    auto it = queues_.find(queue_key);
    CHECK(it != queues_.end()) << "Did not find '" << queue_key << "'.";
    auto& queue = it->second;
    CHECK(!queue.finished);
    queue.finished = true;
    Dispatch();
}

void OrderedMultiQueue::Add(const QueueKey& queue_key, std::unique_ptr<Data> data) 
{
    auto it = queues_.find(queue_key);
    if (it == queues_.end()) {
        LOG_EVERY_N(WARNING, 1000)
            << "Ignored data for queue: '" << queue_key << "'";
        return;
    }
    it->second.queue.Push(std::move(data));
    Dispatch();
}

void OrderedMultiQueue::Flush() 
{
    std::vector<QueueKey> unfinished_queues;
    for (auto& entry : queues_) 
    {
        if (!entry.second.finished) {
            unfinished_queues.push_back(entry.first);
        }
    }

    for (auto& unfinished_queue : unfinished_queues) 
    {
        MarkQueueAsFinished(unfinished_queue);
    }
}

QueueKey OrderedMultiQueue::GetBlocker() const 
{
    CHECK(!queues_.empty());
    return blocker_;
}

void OrderedMultiQueue::Dispatch() 
{
    while (true) 
    {
        const Data* next_data = nullptr;
        Queue* next_queue = nullptr;
        QueueKey next_queue_key;

        for (auto it = queues_.begin(); it != queues_.end();) 
        {
            const auto* data = it->second.queue.Peek<Data>();
            if (data == nullptr) 
            {
                if (it->second.finished) {
                    queues_.erase(it++);
                    continue;
                }

                CannotMakeProgress(it->first);
                return;
            }

            if (next_data == nullptr || data->GetTime() < next_data->GetTime()) 
            {
                next_data = data;
                next_queue = &it->second;
                next_queue_key = it->first;
            }

            CHECK_LE(last_dispatched_time_, next_data->GetTime())
                << "Non-sorted data added to queue: '" << it->first << "'";
            ++it;
        }
        
        if (next_data == nullptr) 
        {
            CHECK(queues_.empty());
            return;
        }

        last_dispatched_time_ = next_data->GetTime();
        next_queue->callback(next_queue->queue.Pop());
    }
}

void OrderedMultiQueue::CannotMakeProgress(const QueueKey& queue_key) 
{
    blocker_ = queue_key;
    for (auto& entry : queues_) 
    {
        if (entry.second.queue.Size() > kMaxQueueSize) 
        {
            LOG_EVERY_N(WARNING, 60) << "Queue waiting for data: " << queue_key;
            return;
        }
    }
}

}  // namespace sensor
}  // namespace vins
}  // namespace xslam
