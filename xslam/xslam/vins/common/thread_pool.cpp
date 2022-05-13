#include "xslam/vins/common/thread_pool.h"

#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <numeric>

#include "glog/logging.h"

namespace xslam {
namespace vins {
namespace common {

ThreadPool::ThreadPool(int num_threads) 
{
    MutexLocker locker(&mutex_);
    for (int i = 0; i != num_threads; ++i) {
        pool_.emplace_back([this]() { ThreadPool::DoWork(); });
    }
}

ThreadPool::~ThreadPool() 
{
    {
        MutexLocker locker(&mutex_);
        CHECK(running_);
        running_ = false;
        CHECK_EQ(work_queue_.size(), 0);
    }

    for (std::thread& thread : pool_) {
        thread.join();
    }
}

void ThreadPool::Schedule(std::function<void()> work_item) 
{
    MutexLocker locker(&mutex_);
    CHECK(running_);
    work_queue_.push_back(work_item);
}

void ThreadPool::DoWork() 
{
#ifdef __linux__
  // This changes the per-thread nice level of the current thread on Linux. We
  // do this so that the background work done by the thread pool is not taking
  // away CPU resources from more important foreground threads.
  CHECK_NE(nice(10), -1);
#endif
    for (;;) 
    {
        std::function<void()> work_item;
        {
            MutexLocker locker(&mutex_);
            locker.Await([this]() {
                return !work_queue_.empty() || !running_;
            });
            
            if (!work_queue_.empty()) {
                work_item = work_queue_.front();
                work_queue_.pop_front();
            } else if (!running_) {
                return;
            }
        }
        CHECK(work_item);
        work_item();
    }
}

}  // namespace common
}  // namespace vins
}  // namespace xslam
