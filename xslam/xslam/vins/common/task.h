#ifndef XSLAM_VINS_COMMON_TASK_H_
#define XSLAM_VINS_COMMON_TASK_H_

#include "xslam/vins/common/thread_pool.h"

#include <set>
#include <mutex>
#include <memory>
#include <functional>

#include "glog/logging.h"


namespace xslam {
namespace vins {
namespace common {

class ThreadPoolInterface;

class Task 
{
public:
    friend class ThreadPoolInterface;

    using WorkItem = std::function<void()>;
    enum State 
    { 
        NEW, 
        DISPATCHED, 
        DEPENDENCIES_COMPLETED, 
        RUNNING, 
        COMPLETED 
    };

    Task() = default;
    ~Task();

    State GetState();

    // State must be 'NEW'.
    void SetWorkItem(const WorkItem& work_item);

    // State must be 'NEW'. 'dependency' may be nullptr, in which case it is
    // assumed completed.
    void AddDependency(std::weak_ptr<Task> dependency);

private:
    // Allowed in all states.
    void AddDependentTask(Task* dependent_task);

    // State must be 'DEPENDENCIES_COMPLETED' and becomes 'COMPLETED'.
    void Execute();

    // State must be 'NEW' and becomes 'DISPATCHED' or 'DEPENDENCIES_COMPLETED'.
    void SetThreadPool(ThreadPoolInterface* thread_pool);

    // State must be 'NEW' or 'DISPATCHED'. If 'DISPATCHED', may become
    // 'DEPENDENCIES_COMPLETED'.
    void OnDependenyCompleted();

    WorkItem work_item_;
    ThreadPoolInterface* thread_pool_to_notify_ = nullptr;
    State state_ = NEW;
    unsigned int uncompleted_dependencies_ = 0;
    std::set<Task*> dependent_tasks_;

    std::mutex mutex_;
};

}  // namespace common
}  // namespace vins
}  // namespace xslam

#endif  // XSLAM_VINS_COMMON_TASK_H_
