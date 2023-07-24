/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright 2023 TeMoto Framework
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

#ifndef TEMOTO_ACTION_ENGINE__COMPILER_MACROS_H
#define TEMOTO_ACTION_ENGINE__COMPILER_MACROS_H

// Define the mutex and lock guard according to the compiler type
#if defined(__clang__)
  #include "temoto_action_engine/mutex.h"
  #define MUTEX_TYPE action_engine::Mutex
  #define MUTEX_TYPE_R action_engine::RecursiveMutex
  #define LOCK_GUARD_TYPE action_engine::LockGuard
  #define LOCK_GUARD_TYPE_R action_engine::RecursiveLockGuard
  #define GUARDED_VARIABLE(var, mutex) var GUARDED_BY(mutex)
#elif(__GNUC__)
  #include <mutex>
  #define MUTEX_TYPE std::mutex
  #define MUTEX_TYPE_R std::recursive_mutex
  #define LOCK_GUARD_TYPE std::lock_guard<std::mutex>
  #define LOCK_GUARD_TYPE_R std::lock_guard<std::recursive_mutex>
  #define GUARDED_VARIABLE(var, mutex) var
#endif

#endif