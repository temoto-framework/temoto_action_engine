/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright 2023 TeMoto Telerobotics
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

#include "temoto_action_engine/action_engine_handle.h"
#include "temoto_action_engine/basic_timer.h"
#include "temoto_action_engine/messaging.h"
#include "temoto_action_engine/umrf_json.h"
#include "temoto_action_engine/umrf_node_exec.h"

UmrfNodeExec::UmrfNodeExec(const UmrfNode& umrf_node)
: UmrfNode(umrf_node)
{
  if (getActorExecTraits() == UmrfNode::ActorExecTraits::LOCAL)
  try
  {
    LOCK_GUARD_TYPE guard_class_loader(class_loader_rw_mutex_);
    std::string library_name = "lib" + getName() + ".so";
    class_loader_ = std::make_shared<class_loader::ClassLoader>(library_name, false);

    // Check if the classloader actually contains the required action
    std::vector<std::string> classes_in_classloader = class_loader_->getAvailableClasses<ActionBase>();
    bool class_found = [&]{
    for (const auto& class_in_classloader : classes_in_classloader)
    {
      if (class_in_classloader == getName())
      {
        return true;
      }
    }
    return false;
    }();

    if (!class_found)
    {
      TEMOTO_PRINT("Failed to initialize the Action Handle because the action class name is incorrect");
      setState(State::ERROR);
      return;
    }
    setLatestUmrfJsonStr(umrf_json::toUmrfJsonStr(*this));
  }
  catch(const std::exception& e)
  {
    setState(State::ERROR);
    std::cerr << "Failed to initialize the Action Handle because: " << e.what() << '\n';
  }
  else if (getActorExecTraits() == UmrfNode::ActorExecTraits::REMOTE)
  {
    // TODO:
  }
  else if (getActorExecTraits() == UmrfNode::ActorExecTraits::GRAPH)
  {
    // TODO: 
  }
}

UmrfNodeExec::~UmrfNodeExec()
{
  // TODO: finish the clear-up implementation
  clearNode();
}

UmrfNode UmrfNodeExec::asUmrfNode() const
{
  UmrfNode umrf_node = umrf_json::fromUmrfJsonStr(getLatestUmrfJsonStr());
  umrf_node.setState(getState());
  return umrf_node;
  //return UmrfNode(*this);
}

const TemotoErrorStack& UmrfNodeExec::getErrorMessages() const
{
  return error_messages_;
}

void UmrfNodeExec::clearNode()
try
{
  // TODO
  // if (getState() != State::NOT_SET)
  // {
  //   stopNode(10);
  //   LOCK_GUARD_TYPE guard_action_instance(action_instance_rw_mutex_);
  //   action_instance_.reset();
  //   setState(State::NOT_SET);
  // }
}
catch(TemotoErrorStack e)
{
  throw FORWARD_TEMOTO_ERROR_STACK(e);
}
catch(const std::exception& e)
{
  throw CREATE_TEMOTO_ERROR_STACK(std::string(e.what()));
}
catch(...)
{
  throw CREATE_TEMOTO_ERROR_STACK("Caught an unhandled error.");
}

void UmrfNodeExec::instantiate()
try
{
  LOCK_GUARD_TYPE_R guard_action_instance(action_instance_rw_mutex_);
  LOCK_GUARD_TYPE guard_class_loader(class_loader_rw_mutex_);

  if (getState() != State::NOT_SET)
  {
    throw CREATE_TEMOTO_ERROR_STACK("Cannot instantiate the action because it's not in NOT_SET state");
  }

  action_instance_ = class_loader_->createInstance<ActionBase>(getName());
  action_instance_->setUmrf(UmrfNode(*this));
}
catch(const std::exception& e)
{
  setState(State::ERROR);
  throw CREATE_TEMOTO_ERROR_STACK("Failed to create an instance of the action: " + std::string(e.what()));
}

std::string UmrfNodeExec::getLatestUmrfJsonStr() const
{
  LOCK_GUARD_TYPE guard_latest_umrf_json_string(latest_umrf_json_str_rw_mutex_);
  return latest_umrf_json_str_;
}

void UmrfNodeExec::setLatestUmrfJsonStr(const std::string& latest_umrf_json_str)
{
  LOCK_GUARD_TYPE guard_latest_umrf_json_string(latest_umrf_json_str_rw_mutex_);
  latest_umrf_json_str_ = latest_umrf_json_str;
}

void UmrfNodeExec::run()
{
  // Check if the action is already running state
  if (getState() == UmrfNode::State::RUNNING)
  {
    return;
  }

  clearThread(UmrfNode::State::RUNNING);
  LOCK_GUARD_TYPE_R l(action_threads_rw_mutex_);

  /*
   * Run the action in a thread
   */
  action_threads_[UmrfNode::State::RUNNING] = UmrfNodeExec::ThreadWrapper();
  action_threads_[UmrfNode::State::RUNNING].thread = std::make_shared<std::thread>([&]()
  {
  {
    LOCK_GUARD_TYPE_R l(action_threads_rw_mutex_);
    action_threads_[UmrfNode::State::RUNNING].is_running = true;
  }

  std::string result;

  try
  {
    // Move to the error state if the action is not in the following states
    if (getState() != UmrfNode::State::NOT_SET && 
        getState() != UmrfNode::State::INITIALIZED &&
        getState() != UmrfNode::State::PAUSED &&
        getState() != UmrfNode::State::FINISHED)
    {
      throw CREATE_TEMOTO_ERROR_STACK("Action has to be in the following states to run: NOT_SET; INITIALIZED; PAUSED; FINISHED.");
    }

    // Initialize the action
    if (getState() == UmrfNode::State::NOT_SET)
    {
      LOCK_GUARD_TYPE_R guard_action_instance(action_instance_rw_mutex_);
      instantiate();
      action_instance_->onInit();
      setState(State::INITIALIZED);
    }

    auto previous_state = getState();
    setState(State::RUNNING);

    /*
     * EXECUTE ACTION AS LOCAL
     */
    if (getActorExecTraits() == UmrfNode::ActorExecTraits::LOCAL)
    {
      result = (action_instance_->executeActionWrapped()) ? "on_true" : "on_false"; // Blocking call, returns when finished
    }

    /*
     * EXECUTE ACTION AS REMOTE
     */
    else if (getActorExecTraits() == UmrfNode::ActorExecTraits::REMOTE)
    {
      result = waitUntilFinished(Waitable{.action_name = "TODO_ACTION_NAME", .graph_name = "TODO_GRAPH_NAME"});

      // TODO: 2) send acknowledgement via syncer::acknowledge(std::string action_name) std::function
      //          blocks until consensus is reached among all actors
      // ENGINE_HANDLE.acknowledge("TODO_ACTION_NAME"); 
    }

    /*
    * EXECUTE ACTION AS GRAPH
    */
    else if (getActorExecTraits() == UmrfNode::ActorExecTraits::GRAPH)
    {
      ENGINE_HANDLE.executeUmrfGraph("TODO_GRAPH_NAME", getInputParameters());
      result = waitUntilFinished(Waitable{.action_name = "graph_exit", .graph_name = "TODO_GRAPH_NAME"});
    }

  } // try end
  catch(TemotoErrorStack e)
  {
    LOCK_GUARD_TYPE_R l(action_threads_rw_mutex_);
    action_threads_[UmrfNode::State::RUNNING].error_messages.appendError(FORWARD_TEMOTO_ERROR_STACK(e));
    setState(State::ERROR);
    result = "on_error";
  }
  catch(const std::exception& e)
  {
    LOCK_GUARD_TYPE_R l(action_threads_rw_mutex_);
    action_threads_[UmrfNode::State::RUNNING].error_messages = CREATE_TEMOTO_ERROR_STACK(std::string(e.what()));
    setState(State::ERROR);
    result = "on_error";
  }
  catch(...)
  {
    LOCK_GUARD_TYPE_R l(action_threads_rw_mutex_);
    action_threads_[UmrfNode::State::RUNNING].error_messages = CREATE_TEMOTO_ERROR_STACK("Caught an unhandled error.");
    setState(State::ERROR);
    result = "on_error";
  }

  if (getActorExecTraits() == UmrfNode::ActorExecTraits::LOCAL)
  {
    // TODO: sync_handle.syncResult(getFullName(), result);
  }

  if (getState() == UmrfNode::State::RUNNING)
  {
    start_child_nodes_cb_(getFullName(), action_instance_->getUmrfNodeConst().getOutputParameters(), result);
    setState(State::FINISHED);
  }
  else if (getState() == UmrfNode::State::ERROR)
  {
    start_child_nodes_cb_(getFullName(), action_instance_->getUmrfNodeConst().getOutputParameters(), result);
  }

  {
    LOCK_GUARD_TYPE_R l(action_threads_rw_mutex_);
    action_threads_[UmrfNode::State::RUNNING].is_running = false;
  }
  });
}

void UmrfNodeExec::stop()
{
  if (getState() == UmrfNode::State::STOPPING ||
      getState() == UmrfNode::State::FINISHED ||
      getState() == UmrfNode::State::NOT_SET)
  {
    return;
  }

  clearThread(UmrfNode::State::STOPPING);
  LOCK_GUARD_TYPE_R l(action_threads_rw_mutex_);

  /*
   * Run the stopping procedure in a thread
   */
  action_threads_[UmrfNode::State::STOPPING] = UmrfNodeExec::ThreadWrapper();
  action_threads_[UmrfNode::State::STOPPING].thread = std::make_shared<std::thread>([&]()
  {
  try
  {
    setState(State::STOPPING);

    /*
     * LOCALLY STOPPED
     */
    if (getActorExecTraits() == UmrfNode::ActorExecTraits::LOCAL)
    {
      LOCK_GUARD_TYPE_R guard_action_instance(action_instance_rw_mutex_);
      action_instance_->onStop(); // Blocking call, returns when finished
    }

    /*
     * REMOTELY_STOPPED
     */
    else if (getActorExecTraits() == UmrfNode::ActorExecTraits::REMOTE)
    {
      // TODO: sync_handle.waitForStop(getFullName())
    }

    /*
     * STOP THE SUBGRAPH
     */
    else if (getActorExecTraits() == UmrfNode::ActorExecTraits::GRAPH)
    {
      // TODO: engine_handle.stopGraph( TODO )
    }

    setState(State::FINISHED);

  } // try end
  catch(TemotoErrorStack e)
  {
    LOCK_GUARD_TYPE_R l(action_threads_rw_mutex_);
    action_threads_[UmrfNode::State::STOPPING].error_messages.appendError(FORWARD_TEMOTO_ERROR_STACK(e));
    setState(State::ERROR);
  }
  catch(const std::exception& e)
  {
    LOCK_GUARD_TYPE_R l(action_threads_rw_mutex_);
    action_threads_[UmrfNode::State::STOPPING].error_messages = CREATE_TEMOTO_ERROR_STACK(std::string(e.what()));
    setState(State::ERROR);
  }
  catch(...)
  {
    LOCK_GUARD_TYPE_R l(action_threads_rw_mutex_);
    action_threads_[UmrfNode::State::STOPPING].error_messages = CREATE_TEMOTO_ERROR_STACK("Caught an unhandled error.");
    setState(State::ERROR);
  }

  if (getActorExecTraits() == UmrfNode::ActorExecTraits::LOCAL)
  {
    // TODO: sync_handle.syncState(getFullName(), getState());
  }

  if (getState() == UmrfNode::State::ERROR)
  {
    start_child_nodes_cb_(getFullName(), action_instance_->getUmrfNodeConst().getOutputParameters(), "on_error");
  }

  {
    LOCK_GUARD_TYPE_R l(action_threads_rw_mutex_);
    action_threads_[UmrfNode::State::STOPPING].is_running = false;
  }
  });
}

void UmrfNodeExec::pause()
{
  if (getState() == UmrfNode::State::PAUSED)
  {
    return;
  }

  clearThread(UmrfNode::State::PAUSED);
  LOCK_GUARD_TYPE_R l(action_threads_rw_mutex_);

  /*
   * Run the stopping procedure in a thread
   */
  action_threads_[UmrfNode::State::PAUSED] = UmrfNodeExec::ThreadWrapper();
  action_threads_[UmrfNode::State::PAUSED].thread = std::make_shared<std::thread>([&]()
  {
  try
  {
    // Move to the error state if the action is not in the following states
    if (getState() != UmrfNode::State::RUNNING)
    {
      throw CREATE_TEMOTO_ERROR_STACK("Action has to be in the following states to be paused: RUNNING.");
    }

    setState(State::PAUSED);

    /*
     * LOCALLY STOPPED
     */
    if (getActorExecTraits() == UmrfNode::ActorExecTraits::LOCAL)
    {
      LOCK_GUARD_TYPE_R guard_action_instance(action_instance_rw_mutex_);
      action_instance_->onPause(); // Blocking call, returns when finished
    }

    /*
     * REMOTELY_STOPPED
     */
    else if (getActorExecTraits() == UmrfNode::ActorExecTraits::REMOTE)
    {
      // TODO: sync_handle.waitForPause(getFullName())
    }

    /*
     * STOP THE SUBGRAPH
     */
    else if (getActorExecTraits() == UmrfNode::ActorExecTraits::GRAPH)
    {
      // TODO: engine_handle.pauseGraph( TODO )
    }

  } // try end
  catch(TemotoErrorStack e)
  {
    LOCK_GUARD_TYPE_R l(action_threads_rw_mutex_);
    action_threads_[UmrfNode::State::PAUSED].error_messages.appendError(FORWARD_TEMOTO_ERROR_STACK(e));
    setState(State::ERROR);
  }
  catch(const std::exception& e)
  {
    LOCK_GUARD_TYPE_R l(action_threads_rw_mutex_);
    action_threads_[UmrfNode::State::PAUSED].error_messages = CREATE_TEMOTO_ERROR_STACK(std::string(e.what()));
    setState(State::ERROR);
  }
  catch(...)
  {
    LOCK_GUARD_TYPE_R l(action_threads_rw_mutex_);
    action_threads_[UmrfNode::State::PAUSED].error_messages = CREATE_TEMOTO_ERROR_STACK("Caught an unhandled error.");
    setState(State::ERROR);
  }

  if (getActorExecTraits() == UmrfNode::ActorExecTraits::LOCAL)
  {
    // TODO: sync_handle.syncState(getFullName(), getState());
  }

  if (getState() == UmrfNode::State::ERROR)
  {
    start_child_nodes_cb_(getFullName(), action_instance_->getUmrfNodeConst().getOutputParameters(), "on_error");
  }

  {
    LOCK_GUARD_TYPE_R l(action_threads_rw_mutex_);
    action_threads_[UmrfNode::State::PAUSED].is_running = false;
  }
  });
}

void UmrfNodeExec::bypass()
{
  /* TODO */
  // clearThread(UmrfNode::State::BYPASSED);
  // LOCK_GUARD_TYPE_R l(action_threads_rw_mutex_);
}

void UmrfNodeExec::clearThread(const UmrfNode::State state_name)
{
  LOCK_GUARD_TYPE_R l(action_threads_rw_mutex_);

  auto it = action_threads_.find(state_name);
  if (it == action_threads_.end())
  {
    return;
  }

  if (it->second.is_running)
  {
    throw CREATE_TEMOTO_ERROR_STACK("Cannot clear thread '" + state_to_str_map_.at(state_name) + "' because it's running");
  }

  if (it->second.thread->joinable())
  {
    it->second.thread->join();
  }

  action_threads_.erase(it);
}

void UmrfNodeExec::setGraphName(const std::string& parent_graph_name)
{
  parent_graph_name_ = parent_graph_name;
}

std::string UmrfNodeExec::waitUntilFinished(const Waitable& waitable)
{
  ENGINE_HANDLE.addWaiter(waitable, {.graph_name = parent_graph_name_, .action_name = getFullName()});

  wait_ = true;
  std::unique_lock<std::mutex> lock(wait_cv_mutex_);
  wait_cv_.wait(lock, [&]{return !wait_;});

  return remote_result_;
}

void UmrfNodeExec::notifyFinished()
{
  wait_ = false;
  wait_cv_.notify_all();
}

void UmrfNodeExec::setRemoteResult(const std::string& remote_result)
{
  remote_result_ = remote_result;
}