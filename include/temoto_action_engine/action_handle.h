/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright 2019 TeMoto Telerobotics
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

/* Author: Robert Valner */

#ifndef TEMOTO_ACTION_ENGINE__ACTION_HANDLE_H
#define TEMOTO_ACTION_ENGINE__ACTION_HANDLE_H

#include <map>
#include <memory>
#include <string>
#include <future>
#include <class_loader/class_loader.hpp>
#include <boost/shared_ptr.hpp>
#include "temoto_action_engine/compiler_macros.h"
#include "temoto_action_engine/messaging.h"
#include "temoto_action_engine/umrf.h"
#include "temoto_action_engine/action_base.h"
#include "temoto_action_engine/temoto_error.h"

class ActionExecutor;

class ActionHandle
{
public:

  enum class State
  {
    UNINITIALIZED,      // Action library is not loaded
    INITIALIZED,        // Action library is loaded
    READY,              // Action instance is loaded
    RUNNING,            // Action instance is running
    STOP_REQUESTED,     // A request to stop has been registered
    FINISHED,           // Action instance has finished execution
    ERROR,              // Problems with any critical component of the action
  };

  std::map<State, std::string> state_to_str_map_ = 
  {
    {State::UNINITIALIZED, "UNINITIALIZED"},
    {State::INITIALIZED, "INITIALIZED"},
    {State::READY, "READY"},
    {State::RUNNING, "RUNNING"},
    {State::STOP_REQUESTED, "STOP_REQUESTED"},
    {State::FINISHED, "FINISHED"},
    {State::ERROR, "ERROR"},
  };

  ActionHandle(Umrf umrf, ActionExecutor* action_executor_ptr);

  ActionHandle(const ActionHandle& action_handle);

  ~ActionHandle();

  const std::string& getActionName() const;

  const std::string& getEffect() const;

  const unsigned int& getHandleId() const;

  bool addInputParameters(ActionParameters action_parameters);

  void instantiateAction();

  void executeActionThread();

  void stopAction(double timeout);

  void clearAction();

  const State& getState() const;

  bool setState(State state_to_set);

  bool futureIsReady();

  TemotoErrorStack getFutureValue();

  bool clearFuture();

private:
  TemotoErrorStack executeAction();

  bool future_retreived_;

  mutable MUTEX_TYPE state_rw_mutex_;
  GUARDED_VARIABLE(State state_, state_rw_mutex_);

  mutable MUTEX_TYPE_R action_future_rw_mutex_;
  GUARDED_VARIABLE(std::shared_ptr<std::future<TemotoErrorStack>> action_future_, action_future_rw_mutex_);

  mutable MUTEX_TYPE class_loader_rw_mutex_;
  GUARDED_VARIABLE(std::shared_ptr<class_loader::ClassLoader> class_loader_, class_loader_rw_mutex_);

  mutable MUTEX_TYPE action_instance_rw_mutex_;
  GUARDED_VARIABLE(boost::shared_ptr<ActionBase> action_instance_, action_instance_rw_mutex_);

  mutable MUTEX_TYPE_R umrf_rw_mutex_;
  GUARDED_VARIABLE(std::shared_ptr<Umrf> umrf_, umrf_rw_mutex_);

  ActionExecutor* action_executor_ptr_;
};
#endif