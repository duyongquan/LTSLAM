#ifndef XSLAM_VINS_COMMON_THREAD_POOL_H_
#define XSLAM_VINS_COMMON_THREAD_POOL_H_

#include "xslam/vins/common/task.h"

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

class Task;

class ThreadPoolInterface 
{
public:
    ThreadPoolInterface() {}
    virtual ~ThreadPoolInterface() {}
    virtual std::weak_ptr<Task> Schedule(std::unique_ptr<Task> task) = 0;

protected:
    void Execute(Task* task);
    void SetThreadPool(Task* task);

private:
    friend class Task;

    virtual void NotifyDependenciesCompleted(Task* task) = 0;
};

// A fixed number of threads working on tasks. Adding a task does not block.
// Tasks may be added whether or not their dependencies are completed.
// When all dependencies of a task are completed, it is queued up for execution
// in a background thread. The queue must be empty before calling the
// destructor. The thread pool will then wait for the currently executing work
// items to finish and then destroy the threads.
class ThreadPool : public ThreadPoolInterface 
{
public:
    explicit ThreadPool(int num_threads);
    ~ThreadPool();

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    // When the returned weak pointer is expired, 'task' has certainly completed,
    // so dependants no longer need to add it as a dependency.
    std::weak_ptr<Task> Schedule(std::unique_ptr<Task> task) override;

private:
    void DoWork();

    void NotifyDependenciesCompleted(Task* task) override;

    std::mutex mutex_;
    bool running_  = true;
    std::vector<std::thread> pool_;
    std::deque<std::shared_ptr<Task>> task_queue_;
    std::map<Task*, std::shared_ptr<Task>> tasks_not_ready_;
};

}  // namespace common
}  // namespace vins
}  // namespace xslam

#endif  // XSLAM_VINS_COMMON_THREAD_POOL_H_
