#include "xslam/vins/common/threading.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

#if 0

namespace xslam {
namespace vins {
namespace common {

namespace {
// Custom implementation of std::barrier that allows us to execute the below
// tests deterministically.
class Barrier 
{
public:
    Barrier() : Barrier(2) {}

    explicit Barrier(const size_t count)
        : threshold_(count), count_(count), generation_(0) {}

    void Wait() 
    {
        std::unique_lock<std::mutex> lock(mutex_);
        auto current_generation = generation_;
        if (!--count_) {
            ++generation_;
            count_ = threshold_;
            condition_.notify_all();
        } else {
            condition_.wait(lock, [this, current_generation] {
                return current_generation != generation_;
            });
        }
    }

private:
    std::mutex mutex_;
    std::condition_variable condition_;
    const size_t threshold_;
    size_t count_;
    size_t generation_;
};

}  // namespace

// IMPORTANT: CHECK_* macros are not thread-safe,
//            so we use glog's CHECK macros inside threads.

TEST(Thread, TestThreadWait) 
{
    class TestThread : public Thread 
    {
    public:
        Barrier startBarrier;
        Barrier endBarrier;

        void Run() 
        {
            startBarrier.Wait();
            endBarrier.Wait();
        }
    };

    TestThread thread;
    CHECK(!thread.IsStarted());
    CHECK(!thread.IsStopped());
    CHECK(!thread.IsPaused());
    CHECK(!thread.IsRunning());
    CHECK(!thread.IsFinished());

    thread.Start();

    thread.startBarrier.Wait();
    CHECK(thread.IsStarted());
    CHECK(!thread.IsStopped());
    CHECK(!thread.IsPaused());
    CHECK(thread.IsRunning());
    CHECK(!thread.IsFinished());

    thread.endBarrier.Wait();
    thread.Wait();
    CHECK(thread.IsStarted());
    CHECK(!thread.IsStopped());
    CHECK(!thread.IsPaused());
    CHECK(!thread.IsRunning());
    CHECK(thread.IsFinished());
}

TEST(Thread, TestThreadPause) 
{
    class TestThread : public Thread 
    {
    public:
      Barrier startBarrier;
      Barrier pauseBarrier;
      Barrier pausedBarrier;
      Barrier resumedBarrier;
      Barrier endBarrier;

      void Run() 
      {
          startBarrier.Wait();
          pauseBarrier.Wait();
          pausedBarrier.Wait();
          BlockIfPaused();
          resumedBarrier.Wait();
          endBarrier.Wait();
      }
    };

    TestThread thread;
    CHECK(!thread.IsStarted());
    CHECK(!thread.IsStopped());
    CHECK(!thread.IsPaused());
    CHECK(!thread.IsRunning());
    CHECK(!thread.IsFinished());

    thread.Start();

    thread.startBarrier.Wait();
    CHECK(thread.IsStarted());
    CHECK(!thread.IsStopped());
    CHECK(!thread.IsPaused());
    CHECK(thread.IsRunning());
    CHECK(!thread.IsFinished());

    thread.pauseBarrier.Wait();
    thread.Pause();
    thread.pausedBarrier.Wait();
    while (!thread.IsPaused() || thread.IsRunning()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    CHECK(thread.IsStarted());
    CHECK(!thread.IsStopped());
    CHECK(thread.IsPaused());
    CHECK(!thread.IsRunning());
    CHECK(!thread.IsFinished());

    thread.Resume();
    thread.resumedBarrier.Wait();
    CHECK(thread.IsStarted());
    CHECK(!thread.IsStopped());
    CHECK(!thread.IsPaused());
    CHECK(thread.IsRunning());
    CHECK(!thread.IsFinished());

    thread.endBarrier.Wait();
    thread.Wait();
    CHECK(thread.IsStarted());
    CHECK(!thread.IsStopped());
    CHECK(!thread.IsPaused());
    CHECK(!thread.IsRunning());
    CHECK(thread.IsFinished());
}

TEST(Thread, TestThreadPauseStop) 
{
    class TestThread : public Thread 
    {
    public:
        Barrier startBarrier;
        Barrier pauseBarrier;
        Barrier pausedBarrier;
        Barrier resumedBarrier;
        Barrier stopBarrier;
        Barrier stoppingBarrier;
        Barrier stoppedBarrier;
        Barrier endBarrier;

        void Run() 
        {
            startBarrier.Wait();
            pauseBarrier.Wait();
            pausedBarrier.Wait();
            BlockIfPaused();
            resumedBarrier.Wait();
            stopBarrier.Wait();
            stoppingBarrier.Wait();

            if (IsStopped()) {
                stoppedBarrier.Wait();
                endBarrier.Wait();
                return;
            }
        }
    };

    TestThread thread;
    CHECK(!thread.IsStarted());
    CHECK(!thread.IsStopped());
    CHECK(!thread.IsPaused());
    CHECK(!thread.IsRunning());
    CHECK(!thread.IsFinished());

    thread.Start();

    thread.startBarrier.Wait();
    CHECK(thread.IsStarted());
    CHECK(!thread.IsStopped());
    CHECK(!thread.IsPaused());
    CHECK(thread.IsRunning());
    CHECK(!thread.IsFinished());

    thread.pauseBarrier.Wait();
    thread.Pause();
    thread.pausedBarrier.Wait();
    while (!thread.IsPaused() || thread.IsRunning()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    CHECK(thread.IsStarted());
    CHECK(!thread.IsStopped());
    CHECK(thread.IsPaused());
    CHECK(!thread.IsRunning());
    CHECK(!thread.IsFinished());

    thread.Resume();
    thread.resumedBarrier.Wait();
    CHECK(thread.IsStarted());
    CHECK(!thread.IsStopped());
    CHECK(!thread.IsPaused());
    CHECK(thread.IsRunning());
    CHECK(!thread.IsFinished());

    thread.stopBarrier.Wait();
    thread.Stop();
    thread.stoppingBarrier.Wait();
    thread.stoppedBarrier.Wait();
    CHECK(thread.IsStarted());
    CHECK(thread.IsStopped());
    CHECK(!thread.IsPaused());
    CHECK(thread.IsRunning());
    CHECK(!thread.IsFinished());

    thread.endBarrier.Wait();
    thread.Wait();
    CHECK(thread.IsStarted());
    CHECK(thread.IsStopped());
    CHECK(!thread.IsPaused());
    CHECK(!thread.IsRunning());
    CHECK(thread.IsFinished());
}

TEST(Thread, TestThreadRestart) 
{
    class TestThread : public Thread {
    public:
      Barrier startBarrier;
      Barrier endBarrier;

      void Run() {
        startBarrier.Wait();
        endBarrier.Wait();
      }
    };

    TestThread thread;
    CHECK(!thread.IsStarted());
    CHECK(!thread.IsStopped());
    CHECK(!thread.IsPaused());
    CHECK(!thread.IsRunning());
    CHECK(!thread.IsFinished());

    for (size_t i = 0; i < 2; ++i) 
    {
        thread.Start();

        thread.startBarrier.Wait();
        CHECK(thread.IsStarted());
        CHECK(!thread.IsStopped());
        CHECK(!thread.IsPaused());
        CHECK(thread.IsRunning());
        CHECK(!thread.IsFinished());

        thread.endBarrier.Wait();
        thread.Wait();
        CHECK(thread.IsStarted());
        CHECK(!thread.IsStopped());
        CHECK(!thread.IsPaused());
        CHECK(!thread.IsRunning());
        CHECK(thread.IsFinished());
    }
}

TEST(Thread, TestThreadValidSetup) 
{
  class TestThread : public Thread {
   public:
    Barrier startBarrier;
    Barrier signalBarrier;
    Barrier endBarrier;

    void Run() {
      startBarrier.Wait();
      SignalValidSetup();
      signalBarrier.Wait();
      endBarrier.Wait();
    }
  };

  TestThread thread;
  CHECK(!thread.IsStarted());
  CHECK(!thread.IsStopped());
  CHECK(!thread.IsPaused());
  CHECK(!thread.IsRunning());
  CHECK(!thread.IsFinished());

  thread.Start();

  thread.startBarrier.Wait();
  CHECK(thread.IsStarted());
  CHECK(!thread.IsStopped());
  CHECK(!thread.IsPaused());
  CHECK(thread.IsRunning());
  CHECK(!thread.IsFinished());

  thread.signalBarrier.Wait();
  CHECK(thread.CheckValidSetup());

  thread.endBarrier.Wait();
  thread.Wait();
  CHECK(thread.IsStarted());
  CHECK(!thread.IsStopped());
  CHECK(!thread.IsPaused());
  CHECK(!thread.IsRunning());
  CHECK(thread.IsFinished());
}

TEST(Thread, TestThreadInvalidSetup) 
{
  class TestThread : public Thread {
   public:
    Barrier startBarrier;
    Barrier signalBarrier;
    Barrier endBarrier;

    void Run() {
      startBarrier.Wait();
      SignalInvalidSetup();
      signalBarrier.Wait();
      endBarrier.Wait();
    }
  };

  TestThread thread;
  CHECK(!thread.IsStarted());
  CHECK(!thread.IsStopped());
  CHECK(!thread.IsPaused());
  CHECK(!thread.IsRunning());
  CHECK(!thread.IsFinished());

  thread.Start();

  thread.startBarrier.Wait();
  CHECK(thread.IsStarted());
  CHECK(!thread.IsStopped());
  CHECK(!thread.IsPaused());
  CHECK(thread.IsRunning());
  CHECK(!thread.IsFinished());

  thread.signalBarrier.Wait();
  CHECK(!thread.CheckValidSetup());

  thread.endBarrier.Wait();
  thread.Wait();
  CHECK(thread.IsStarted());
  CHECK(!thread.IsStopped());
  CHECK(!thread.IsPaused());
  CHECK(!thread.IsRunning());
  CHECK(thread.IsFinished());
}

TEST(Thread, TestCallback) 
{
  class TestThread : public Thread {
   public:
    enum Callbacks {
      CALLBACK1,
      CALLBACK2,
    };

    TestThread() {
      RegisterCallback(CALLBACK1);
      RegisterCallback(CALLBACK2);
    }

   private:
    void Run() {
      Callback(CALLBACK1);
      Callback(CALLBACK2);
    }
  };

  bool called_back1 = false;
  std::function<void()> CallbackFunc1 = [&called_back1]() {
    called_back1 = true;
  };

  bool called_back2 = false;
  std::function<void()> CallbackFunc2 = [&called_back2]() {
    called_back2 = true;
  };

  bool called_back3 = false;
  std::function<void()> CallbackFunc3 = [&called_back3]() {
    called_back3 = true;
  };

  TestThread thread;
  thread.AddCallback(TestThread::CALLBACK1, CallbackFunc1);
  thread.Start();
  thread.Wait();
  CHECK(called_back1);
  CHECK(!called_back2);
  CHECK(!called_back3);

  called_back1 = false;
  called_back2 = false;
  thread.AddCallback(TestThread::CALLBACK2, CallbackFunc2);
  thread.Start();
  thread.Wait();
  CHECK(called_back1);
  CHECK(called_back2);
  CHECK(!called_back3);

  called_back1 = false;
  called_back2 = false;
  called_back3 = false;
  thread.AddCallback(TestThread::CALLBACK1, CallbackFunc3);
  thread.Start();
  thread.Wait();
  CHECK(called_back1);
  CHECK(called_back2);
  CHECK(called_back3);
}

TEST(Thread, TestDefaultCallback) {
  class TestThread : public Thread {
   public:
    Barrier startBarrier;
    Barrier signalBarrier;
    Barrier endBarrier;

    void Run() {
      startBarrier.Wait();
      endBarrier.Wait();
    }
  };

  bool called_back1 = false;
  std::function<void()> CallbackFunc1 = [&called_back1]() {
    called_back1 = true;
  };

  bool called_back2 = false;
  std::function<void()> CallbackFunc2 = [&called_back2]() {
    called_back2 = true;
  };

  TestThread thread;
  thread.AddCallback(TestThread::STARTED_CALLBACK, CallbackFunc1);
  thread.AddCallback(TestThread::FINISHED_CALLBACK, CallbackFunc2);
  thread.Start();
  thread.startBarrier.Wait();
  CHECK(called_back1);
  CHECK(!called_back2);
  thread.endBarrier.Wait();
  thread.Wait();
  CHECK(called_back1);
  CHECK(called_back2);
}

TEST(Thread, TestThreadPoolNoArgNoReturn) {
  std::function<void(void)> Func = []() {
    int num = 0;
    for (int i = 0; i < 100; ++i) {
      num += i;
    }
  };

  ThreadPool pool(4);
  std::vector<std::future<void>> futures;

  for (int i = 0; i < 100; ++i) {
    futures.push_back(pool.AddTask(Func));
  }

  for (auto& future : futures) {
    future.get();
  }
}

TEST(Thread, TestThreadPoolArgNoReturn) {
  std::function<void(int)> Func = [](int num) {
    for (int i = 0; i < 100; ++i) {
      num += i;
    }
  };

  ThreadPool pool(4);
  std::vector<std::future<void>> futures;

  for (int i = 0; i < 100; ++i) {
    futures.push_back(pool.AddTask(Func, i));
  }

  for (auto& future : futures) {
    future.get();
  }
}

TEST(Thread, TestThreadPoolNoArgReturn) {
  std::function<int(void)> Func = []() { return 0; };

  ThreadPool pool(4);
  std::vector<std::future<int>> futures;

  for (int i = 0; i < 100; ++i) {
    futures.push_back(pool.AddTask(Func));
  }

  for (auto& future : futures) {
    future.get();
  }
}

TEST(Thread, TestThreadPoolArgReturn) {
  std::function<int(int)> Func = [](int num) {
    for (int i = 0; i < 100; ++i) {
      num += i;
    }
    return num;
  };

  ThreadPool pool(4);
  std::vector<std::future<int>> futures;

  for (int i = 0; i < 100; ++i) {
    futures.push_back(pool.AddTask(Func, i));
  }

  for (auto& future : futures) {
    future.get();
  }
}

TEST(Thread, TestThreadPoolStop) {
  std::function<int(int)> Func = [](int num) {
    for (int i = 0; i < 100; ++i) {
      num += i;
    }
    return num;
  };

  ThreadPool pool(4);
  std::vector<std::future<int>> futures;

  for (int i = 0; i < 100; ++i) {
    futures.push_back(pool.AddTask(Func, i));
  }

  pool.Stop();

  // CHECK_THROW(pool.AddTask(Func, 100), std::runtime_error);

  pool.Stop();
}

TEST(Thread, TestThreadPoolWait) {
  std::vector<uint8_t> results(100, 0);
  std::function<void(int)> Func = [&results](const int num) {
    results[num] = 1;
  };

  ThreadPool pool(4);
  pool.Wait();

  for (size_t i = 0; i < results.size(); ++i) {
    pool.AddTask(Func, i);
  }

  pool.Wait();

  for (const auto result : results) {
    CHECK_EQ(result, 1);
  }
}

TEST(Thread, TestThreadPoolWaitEverytime) {
  std::vector<uint8_t> results(4, 0);
  std::function<void(int)> Func = [&results](const int num) {
    results[num] = 1;
  };

  ThreadPool pool(4);

  for (size_t i = 0; i < results.size(); ++i) {
    pool.AddTask(Func, i);
    pool.Wait();

    for (size_t j = 0; j < results.size(); ++j) {
      if (j <= i) {
        CHECK_EQ(results[j], 1);
      } else {
        CHECK_EQ(results[j], 0);
      }
    }
  }

  pool.Wait();
}

TEST(Thread, TestThreadPoolGetThreadIndex) {
  ThreadPool pool(4);

  std::vector<int> results(100, -1);
  std::function<void(int)> Func = [&](const int num) {
    results[num] = pool.GetThreadIndex();
  };

  for (size_t i = 0; i < results.size(); ++i) {
    pool.AddTask(Func, i);
  }

  pool.Wait();

  for (const auto result : results) {
    CHECK_GE(result, 0);
    CHECK_LE(result, 3);
  }
}

TEST(Thread, TestJobQueueSingleProducerSingleConsumer) {
  JobQueue<int> job_queue;

  // IMPORTANT: CHECK_* macros are not thread-safe,
  //            so we use glog's CHECK macros inside threads.

  std::thread producer_thread([&job_queue]() {
    for (int i = 0; i < 10; ++i) {
      CHECK(job_queue.Push(i));
    }
  });

  std::thread consumer_thread([&job_queue]() {
    CHECK_LE(job_queue.Size(), 10);
    for (int i = 0; i < 10; ++i) {
      const auto job = job_queue.Pop();
      CHECK(job.IsValid());
      CHECK_LT(job.Data(), 10);
    }
  });

  producer_thread.join();
  consumer_thread.join();
}

TEST(Thread, TestJobQueueSingleProducerSingleConsumerMaxNumJobs) {
  JobQueue<int> job_queue(2);

  // IMPORTANT: CHECK_* macros are not thread-safe,
  //            so we use glog's CHECK macros inside threads.

  std::thread producer_thread([&job_queue]() {
    for (int i = 0; i < 10; ++i) {
      CHECK(job_queue.Push(i));
    }
  });

  std::thread consumer_thread([&job_queue]() {
    CHECK_LE(job_queue.Size(), 2);
    for (int i = 0; i < 10; ++i) {
      const auto job = job_queue.Pop();
      CHECK(job.IsValid());
      CHECK_LT(job.Data(), 10);
    }
  });

  producer_thread.join();
  consumer_thread.join();
}

TEST(Thread, TestJobQueueMultipleProducerSingleConsumer) {
  JobQueue<int> job_queue(1);

  // IMPORTANT: CHECK_* macros are not thread-safe,
  //            so we use glog's CHECK macros inside threads.

  std::thread producer_thread1([&job_queue]() {
    for (int i = 0; i < 10; ++i) {
      CHECK(job_queue.Push(i));
    }
  });

  std::thread producer_thread2([&job_queue]() {
    for (int i = 0; i < 10; ++i) {
      CHECK(job_queue.Push(i));
    }
  });

  std::thread consumer_thread([&job_queue]() {
    CHECK_LE(job_queue.Size(), 1);
    for (int i = 0; i < 20; ++i) {
      const auto job = job_queue.Pop();
      CHECK(job.IsValid());
      CHECK_LT(job.Data(), 10);
    }
  });

  producer_thread1.join();
  producer_thread2.join();
  consumer_thread.join();
}

TEST(Thread, TestJobQueueSingleProducerMultipleConsumer) {
  JobQueue<int> job_queue(1);

  // IMPORTANT: CHECK_* macros are not thread-safe,
  //            so we use glog's CHECK macros inside threads.

  std::thread producer_thread([&job_queue]() {
    for (int i = 0; i < 20; ++i) {
      CHECK(job_queue.Push(i));
    }
  });

  std::thread consumer_thread1([&job_queue]() {
    CHECK_LE(job_queue.Size(), 1);
    for (int i = 0; i < 10; ++i) {
      const auto job = job_queue.Pop();
      CHECK(job.IsValid());
      CHECK_LT(job.Data(), 20);
    }
  });

  std::thread consumer_thread2([&job_queue]() {
    CHECK_LE(job_queue.Size(), 1);
    for (int i = 0; i < 10; ++i) {
      const auto job = job_queue.Pop();
      CHECK(job.IsValid());
      CHECK_LT(job.Data(), 20);
    }
  });

  producer_thread.join();
  consumer_thread1.join();
  consumer_thread2.join();
}

TEST(Thread, TestJobQueueMultipleProducerMultipleConsumer) {
  JobQueue<int> job_queue(1);

  // IMPORTANT: CHECK_* macros are not thread-safe,
  //            so we use glog's CHECK macros inside threads.

  std::thread producer_thread1([&job_queue]() {
    for (int i = 0; i < 10; ++i) {
      CHECK(job_queue.Push(i));
    }
  });

  std::thread producer_thread2([&job_queue]() {
    for (int i = 0; i < 10; ++i) {
      CHECK(job_queue.Push(i));
    }
  });

  std::thread consumer_thread1([&job_queue]() {
    CHECK_LE(job_queue.Size(), 1);
    for (int i = 0; i < 10; ++i) {
      const auto job = job_queue.Pop();
      CHECK(job.IsValid());
      CHECK_LT(job.Data(), 10);
    }
  });

  std::thread consumer_thread2([&job_queue]() {
    CHECK_LE(job_queue.Size(), 1);
    for (int i = 0; i < 10; ++i) {
      const auto job = job_queue.Pop();
      CHECK(job.IsValid());
      CHECK_LT(job.Data(), 10);
    }
  });

  producer_thread1.join();
  producer_thread2.join();
  consumer_thread1.join();
  consumer_thread2.join();
}

TEST(Thread, TestJobQueueWait) {
  JobQueue<int> job_queue;

  // IMPORTANT: CHECK_* macros are not thread-safe,
  //            so we use glog's CHECK macros inside threads.

  for (int i = 0; i < 10; ++i) {
    CHECK(job_queue.Push(i));
  }

  std::thread consumer_thread([&job_queue]() {
    CHECK_EQ(job_queue.Size(), 10);
    for (int i = 0; i < 10; ++i) {
      const auto job = job_queue.Pop();
      CHECK(job.IsValid());
      CHECK_EQ(job.Data(), i);
    }
  });

  job_queue.Wait();

  CHECK_EQ(job_queue.Size(), 0);
  CHECK(job_queue.Push(0));
  CHECK(job_queue.Pop().IsValid());

  consumer_thread.join();
}

TEST(Thread, TestJobQueueStopProducer) {
  JobQueue<int> job_queue(1);

  // IMPORTANT: CHECK_* macros are not thread-safe,
  //            so we use glog's CHECK macros inside threads.

  Barrier stopBarrier;
  std::thread producer_thread([&job_queue, &stopBarrier]() {
    CHECK(job_queue.Push(0));
    stopBarrier.Wait();
    CHECK(!job_queue.Push(0));
  });

  stopBarrier.Wait();
  CHECK_EQ(job_queue.Size(), 1);

  job_queue.Stop();
  producer_thread.join();

  CHECK(!job_queue.Push(0));
  CHECK(!job_queue.Pop().IsValid());
}

TEST(Thread, TestJobQueueStopConsumer) {
  JobQueue<int> job_queue(1);

  // IMPORTANT: CHECK_* macros are not thread-safe,
  //            so we use glog's CHECK macros inside threads.

  CHECK(job_queue.Push(0));

  Barrier popBarrier;
  std::thread consumer_thread([&job_queue, &popBarrier]() {
    const auto job = job_queue.Pop();
    CHECK(job.IsValid());
    CHECK_EQ(job.Data(), 0);
    popBarrier.Wait();
    CHECK(!job_queue.Pop().IsValid());
  });

  popBarrier.Wait();
  CHECK_EQ(job_queue.Size(), 0);

  job_queue.Stop();
  consumer_thread.join();

  CHECK(!job_queue.Push(0));
  CHECK(!job_queue.Pop().IsValid());
}

TEST(Thread, TestJobQueueClear) {
  JobQueue<int> job_queue(1);

  CHECK(job_queue.Push(0));
  CHECK_EQ(job_queue.Size(), 1);

  job_queue.Clear();
  CHECK_EQ(job_queue.Size(), 0);
}

TEST(Thread, TestGetEffectiveNumThreads) 
{
  CHECK_GT(GetEffectiveNumThreads(-2), 0);
  CHECK_GT(GetEffectiveNumThreads(-1), 0);
  CHECK_GT(GetEffectiveNumThreads(0), 0);
  CHECK_EQ(GetEffectiveNumThreads(1), 1);
  CHECK_EQ(GetEffectiveNumThreads(2), 2);
  CHECK_EQ(GetEffectiveNumThreads(3), 3);
}

}  // namespace common
}  // namespace vins 
}  // namespace xslam

#endif 