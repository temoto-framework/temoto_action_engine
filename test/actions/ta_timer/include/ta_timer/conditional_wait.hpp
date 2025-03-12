#pragma once

#include <condition_variable>
#include <mutex>

struct CvWrapper
{
  void wait();
  void stopWaiting();

private:

  std::condition_variable cv_;
  std::mutex cv_mutex_;
  bool wait_{false};
};
