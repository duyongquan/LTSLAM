#ifndef TUTORIAL_COMMON_UTILS_DBQUEUE_H
#define TUTORIAL_COMMON_UTILS_DBQUEUE_H

#include <queue>
#include <mutex>
#include <memory>
#include <condition_variable>


namespace tutorial {
namespace common {
namespace utils {

/**
 * Double buffered, threadsafe queue for MPSC (multi-producer, single-consumer) comms.
 */
template<class T>
class DBQueue {

public:
   DBQueue():
      mForegroundQueue(&mQueueAlpha),
      mBackgroundQueue(&mQueueBeta)
   {}

   //! Clears foreground queue and swaps queues.
   void Swap()
   {
      std::unique_lock<std::mutex> fgGuard(mForegroundMutex);
      std::unique_lock<std::mutex> bgGuard(mBackgroundMutex);

      // Clear the foreground queue.
      std::queue<T>().swap(*mForegroundQueue);

      auto* swap       = mBackgroundQueue;
      mBackgroundQueue = mForegroundQueue;
      mForegroundQueue = swap;
   }

   //! Pushes to the background queue.
   void Push(const T& item)
   {
      std::unique_lock<std::mutex> guard(mBackgroundMutex);
      mBackgroundQueue->push(item);
   }

   //! Returns a reference to the front element
   //! in the foregrund queue.
   T& Front()
   {
      std::unique_lock<std::mutex> guard(mForegroundMutex);
      return mForegroundQueue->front();
   }

   const T& Front() const
   {
      std::unique_lock<std::mutex> guard(mForegroundMutex);
      return mForegroundQueue->front();
   }

   //! Pops from the foreground queue.
   void Pop()
   {
      std::unique_lock<std::mutex> guard(mForegroundMutex);
      mForegroundQueue->pop();
   }

   //! Reports whether the foreground queue is empty.
   bool Empty() const
   {
      std::unique_lock<std::mutex> guard(mForegroundMutex);
      return mForegroundQueue->empty();
   }

   //! Reports whether the both queues are empty.
   bool BothEmpty() const
   {
      std::unique_lock<std::mutex> guard(mForegroundMutex);
      std::unique_lock<std::mutex> bgGuard(mBackgroundMutex);
      return mForegroundQueue->empty() && mBackgroundQueue->empty();
   }

   //! Reports the size of the foreground queue.
   size_t Size() const
   {
      std::unique_lock<std::mutex> guard(mForegroundMutex);
      return mForegroundQueue->size();
   }

   //! Clears foreground and background.
   void Clear()
   {
      std::unique_lock<std::mutex> fgGuard(mForegroundMutex);
      std::unique_lock<std::mutex> bgGuard(mBackgroundMutex);
      std::queue<T>().swap(*mForegroundQueue);
      std::queue<T>().swap(*mBackgroundQueue);
   }

private:
   // Underlying queues
   std::queue<T> mQueueAlpha;
   std::queue<T> mQueueBeta;

   // Front and background queue references (double buffering)
   std::queue<T>* mForegroundQueue;
   std::queue<T>* mBackgroundQueue;

   mutable std::mutex mForegroundMutex;
   mutable std::mutex mBackgroundMutex;
};


} // namespace log
} // namespace common
} // namespace tutorial

#endif // TUTORIAL_COMMON_UTILS_DBQUEUE_H
