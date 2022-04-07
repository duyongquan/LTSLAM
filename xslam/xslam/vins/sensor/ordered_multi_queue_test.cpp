#include "xslam/vins/sensor/ordered_multi_queue.h"

#include <vector>
#include <memory>

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace vins {
namespace sensor {
namespace {

class OrderedMultiQueueTest : public ::testing::Test 
{
protected:
    // These are keys are chosen so that they sort first, second, third.
    const QueueKey kFirst{"a"};
    const QueueKey kSecond{"b"};
    const QueueKey kThird{"c"};

    void SetUp() 
    {
        for (const auto& queue_key : {kFirst, kSecond, kThird}) 
        {
            queue_.AddQueue(queue_key, [this](std::unique_ptr<Data> data) 
            {
                if (!values_.empty()) 
                {
                    EXPECT_GE(data->GetTime(), values_.back()->GetTime());
                }
                values_.push_back(std::move(data));
            });
        }
    }

    std::unique_ptr<Data> MakeImu(const int ordinal) 
    {
      return MakeDispatchable(
          "imu", ImuData{common::FromUniversal(ordinal), Eigen::Vector3d::Zero(),
                         Eigen::Vector3d::Zero()});
    }

    std::vector<std::unique_ptr<Data>> values_;
    OrderedMultiQueue queue_;
};

TEST_F(OrderedMultiQueueTest, Ordering) 
{
    queue_.Add(kFirst, MakeImu(0));
    queue_.Add(kFirst, MakeImu(4));
    queue_.Add(kFirst, MakeImu(5));
    queue_.Add(kFirst, MakeImu(6));
    // queue_.Add(kFirst, MakeImu(3));

    // queue_.Add(kSecond, MakeImu(0));
    // queue_.Add(kSecond, MakeImu(1));
    // EXPECT_TRUE(values_.empty());

    // queue_.Add(kThird, MakeImu(0));
    // queue_.Add(kThird, MakeImu(2));
    // EXPECT_EQ(values_.size(), 4);

    // queue_.Add(kSecond, MakeImu(3));
    // EXPECT_EQ(values_.size(), 5);

    // queue_.Add(kSecond, MakeImu(7));
    // queue_.Add(kThird, MakeImu(8));
 
    queue_.Flush();

    EXPECT_EQ(4, values_.size());
    for (size_t i = 0; i < values_.size(); ++i) 
    {
        LOG(INFO) << " id: " << values_[i]->GetSensorId() << " time : " << values_[i]->GetTime();
    }
}

// TEST_F(OrderedMultiQueueTest, MarkQueueAsFinished) 
// {
//     queue_.Add(kFirst, MakeImu(1));
//     queue_.Add(kFirst, MakeImu(2));
//     queue_.Add(kFirst, MakeImu(3));
//     EXPECT_TRUE(values_.empty());

//     queue_.MarkQueueAsFinished(kFirst);
//     EXPECT_TRUE(values_.empty());

//     queue_.MarkQueueAsFinished(kSecond);
//     EXPECT_TRUE(values_.empty());

//     queue_.MarkQueueAsFinished(kThird);

//     EXPECT_EQ(3, values_.size());
//     for (size_t i = 0; i < values_.size(); ++i) 
//     {
//         EXPECT_EQ(i + 1, common::ToUniversal(values_[i]->GetTime()));
//     }
// }

// TEST_F(OrderedMultiQueueTest, CommonStartTimePerTrajectory) 
// {
//     queue_.Add(kFirst, MakeImu(0));
//     queue_.Add(kFirst, MakeImu(1));
//     queue_.Add(kFirst, MakeImu(2));
//     queue_.Add(kFirst, MakeImu(3));
//     queue_.Add(kSecond, MakeImu(2));
//     EXPECT_TRUE(values_.empty());

//     queue_.Add(kThird, MakeImu(4));
//     EXPECT_EQ(values_.size(), 2);

//     queue_.MarkQueueAsFinished(kFirst);
//     EXPECT_EQ(values_.size(), 2);

//     queue_.MarkQueueAsFinished(kSecond);
//     EXPECT_EQ(values_.size(), 4);

//     queue_.MarkQueueAsFinished(kThird);
//     EXPECT_EQ(values_.size(), 4);
// }

}
}  // namespace sensor
}  // namespace vins
}  // namespace xslam