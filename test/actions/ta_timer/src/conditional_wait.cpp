#include "ta_timer/conditional_wait.hpp"

void CvWrapper::wait()
{
  wait_ = true;
  std::unique_lock<std::mutex> lock(cv_mutex_);
  cv_.wait(lock, [&]{return !wait_;});
}

void CvWrapper::stopWaiting()
{
  wait_ = false;
  cv_.notify_all();
}
