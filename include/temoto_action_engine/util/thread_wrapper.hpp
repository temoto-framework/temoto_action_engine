/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright 2025 TeMoto Framework
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#pragma once

#include "temoto_action_engine/util/compiler_macros.hpp"
#include "temoto_action_engine/util/error.hpp"
#include "temoto_action_engine/util/mutex.hpp"

#include <memory>
#include <thread>

namespace temoto::util
{

struct ThreadWrapper
{
  std::shared_ptr<std::thread> thread;
  bool is_running;
  TemotoErrorStack error_messages;
};

template <typename T>
class Threads
{
public:

  void start(const T& key, const std::shared_ptr<std::thread>& thread)
  {
    LOCK_GUARD_TYPE_R l(threads_rw_mutex_);

    clear(key);

    threads_[key] = ThreadWrapper{
      .thread = thread,
      .is_running = false
    };
  }

  void addErrorMessage(const T& key, const TemotoErrorStack& error)
  {
    LOCK_GUARD_TYPE_R l(threads_rw_mutex_);
    threads_.at(key).error_messages.appendError(error);
  }

  const char* getErrorMessages(const T& key) const
  {
    LOCK_GUARD_TYPE_R l(threads_rw_mutex_);
    return threads_.at(key).error_messages.what();
  }

  TemotoErrorStack getErrorStack(const T& key) const
  {
    LOCK_GUARD_TYPE_R l(threads_rw_mutex_);
    return threads_.at(key).error_messages;
  }

  void done(const T& key)
  {
    LOCK_GUARD_TYPE_R l(threads_rw_mutex_);
    threads_.at(key).is_running = false;
  }

  typename std::map<T, ThreadWrapper>::iterator begin()
  {
      return threads_.begin();
  }

  typename std::map<T, ThreadWrapper>::iterator end()
  {
      return threads_.end();
  }

  typename std::map<T, ThreadWrapper>::const_iterator begin() const
  {
      return threads_.cbegin();
  }

  typename std::map<T, ThreadWrapper>::const_iterator end() const
  {
      return threads_.cend();
  }

private:

  void clear(const T& key)
  {
    LOCK_GUARD_TYPE_R l(threads_rw_mutex_);

    auto it = threads_.find(key);
    if (it == threads_.end())
    {
      return;
    }

    if (it->second.is_running)
    {
      throw CREATE_TEMOTO_ERROR_STACK("Cannot clear thread because it is running");
    }

    if (it->second.thread->joinable())
    {
      it->second.thread->join();
    }

    threads_.erase(it);
  }

  using ThreadsT = std::map<T, ThreadWrapper>;
  mutable MUTEX_TYPE_R threads_rw_mutex_;
  GUARDED_VARIABLE(ThreadsT threads_, threads_rw_mutex_);

};
}
