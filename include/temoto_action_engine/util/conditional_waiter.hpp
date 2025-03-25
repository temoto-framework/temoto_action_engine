#pragma once

#include <condition_variable>
#include <mutex>
#include <thread>

namespace temoto::util
{

class ConditionalWaiter
{
public:

  void wait()
  {
    wait_ = true;
    std::unique_lock<std::mutex> lock(m_);
    cv_.wait(lock, [&]{return !wait_;});
  }

  void stopWaiting()
  {
    wait_ = false;
    cv_.notify_all();
  }

private:

  std::condition_variable cv_;
  std::mutex m_;
  bool wait_{false};
};

}
