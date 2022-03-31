#include "xslam/vins/common/timer.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace vins {
namespace common {

TEST(Timer, TestDefault) 
{
    Timer timer;
    CHECK_EQ(timer.ElapsedMicroSeconds(), 0);
    CHECK_EQ(timer.ElapsedSeconds(), 0);
    CHECK_EQ(timer.ElapsedMinutes(), 0);
    CHECK_EQ(timer.ElapsedHours(), 0);
}

TEST(Timer, TestStart) 
{
    Timer timer;
    timer.Start();
    CHECK_EQ(timer.ElapsedMicroSeconds(), 0);
    CHECK_EQ(timer.ElapsedSeconds(), 0);
    CHECK_EQ(timer.ElapsedMinutes(), 0);
    CHECK_EQ(timer.ElapsedHours(), 0);
}

TEST(Timer, TestPause) 
{
    Timer timer;
    timer.Start();
    timer.Pause();
    double prev_time = timer.ElapsedMicroSeconds();
    for (size_t i = 0; i < 1000; ++i) {
        CHECK_EQ(timer.ElapsedMicroSeconds(), prev_time);
        prev_time = timer.ElapsedMicroSeconds();
    }
    timer.Resume();
    for (size_t i = 0; i < 1000; ++i) {
        CHECK_GE(timer.ElapsedMicroSeconds(), prev_time);
    }
    timer.Reset();
    CHECK_EQ(timer.ElapsedMicroSeconds(), 0);
}

}  // namespace common
}  // namespace vins
}  // namespace xslam