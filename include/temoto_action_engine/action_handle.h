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
#include "temoto_action_engine/umrf.h"
#include "temoto_action_engine/temoto_error.h"

// Forward declare the action executor object
class ActionExecutor;
class ActionBase;

/**
 * @brief 
 * 
 */
class ActionHandle
{
public:

  /// Defines all possible states for the action handle
  enum class State
  {
    UNINITIALIZED,      // Action library is not loaded or input parameters not received
    INITIALIZED,        // Action library is loaded and input parameters received
    READY,              // Action instance is loaded
    RUNNING,            // Action instance is running
    STOP_REQUESTED,     // A request to stop has been registered
    FINISHED,           // Action instance has finished execution
    ERROR,              // Problems with any critical component of the action
  };

  /// A convenience datastructure which helps to convert state names to std::string
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

  /**
   * @brief Construct a new Action Handle object
   * 
   * @param umrf Basis for the action handle
   * @param action_executor_ptr Used for notifying the action executor
   */
  ActionHandle(Umrf umrf, ActionExecutor* action_executor_ptr);

  /// Copy constructor
  ActionHandle(const ActionHandle& action_handle);

  ~ActionHandle();

  /**
   * @brief Non-blocking call for executing the action in its own thread
   * 
   */
  void executeActionThread();

  /**
   * @brief Creates an instance of the user defined action object. Does not execute the action.
   * 
   */
  void instantiateAction();

  /**
   * @brief Sets the stop request flag via BaseAction::stopRequested and waits for the specified timeout period for
   * the action to finish. If timeout is reached, then the action is set to ERROR state.
   * 
   * @param timeout period in seconds.
   */
  void stopAction(double timeout);

  /**
   * @brief Stops the action (ActionHandle::stopAction) and destroys the action instance object.
   * 
   */
  void clearAction();

  const std::string& getActionName() const;

  const std::string& getEffect() const;

  const unsigned int& getHandleId() const;

  bool addInputParameters(ActionParameters action_parameters);

  const State& getState() const;

  bool setState(State state_to_set);

  bool futureIsReady();

  TemotoErrorStack getFutureValue();

  bool clearFuture();

private:
  /**
   * @brief Blocking call. Invokes the ActionBase::executeAction method.
   * 
   * @return TemotoErrorStack 
   */
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

  /// Used for notifying the ActionExecutor when the action has finished execution.
  ActionExecutor* action_executor_ptr_;
};
#endif