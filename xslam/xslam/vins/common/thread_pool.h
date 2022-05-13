#ifndef XSLAM_VINS_COMMON_THREAD_POOL_H_
#define XSLAM_VINS_COMMON_THREAD_POOL_H_

#include "xslam/vins/common/mutex.h"

#include <deque>
#include <functional>
#include <memory>
#include <thread>
#include <mutex>
#include <vector>
#include <map>

namespace xslam {
namespace vins {
namespace common {

// A fixed number of threads working on a work queue of work items. Adding a
// new work item does not block, and will be executed by a background thread
// eventually. The queue must be empty before calling the destructor. The thread
// pool will then wait for the currently executing work items to finish and then
// destroy the threads.
class ThreadPool 
{
public:
    explicit ThreadPool(int num_threads);
    ~ThreadPool();

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    void Schedule(std::function<void()> work_item);

    private:
    void DoWork();

    Mutex mutex_;
    bool running_ = true;
    std::vector<std::thread> pool_;
    std::deque<std::function<void()>> work_queue_;
};

}  // namespace common
}  // namespace vins
}  // namespace xslam

#endif  // XSLAM_VINS_COMMON_THREAD_POOL_H_
