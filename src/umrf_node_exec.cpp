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

#include "temoto_action_engine/basic_timer.h"
#include "temoto_action_engine/messaging.h"
#include "temoto_action_engine/umrf_json.h"
#include "temoto_action_engine/umrf_node_exec.h"

#include <chrono>

using namespace std;
  
string camelToSnake(string str_camel_case)
{
  string str_snake_case = "";
  char c = tolower(str_camel_case[0]);
  str_snake_case+=(char(c));

  for (int i = 1; i < str_camel_case.length(); i++) 
  {
    char ch = str_camel_case[i];
    if (isupper(ch)) 
    {
      str_snake_case = str_snake_case + '_';
      str_snake_case+=char(tolower(ch));
    }
    else 
    {
      str_snake_case = str_snake_case + ch;
    }
  }
  return str_snake_case;
}

UmrfNodeExec::UmrfNodeExec(const UmrfNode& umrf_node)
: UmrfNode(umrf_node)
{
  if (getActorExecTraits() == UmrfNode::ActorExecTraits::LOCAL && 
      getName() != "graph_entry" &&
      getName() != "graph_exit")
  try
  {
    LOCK_GUARD_TYPE guard_class_loader(class_loader_rw_mutex_);
    std::string library_name = "lib" + camelToSnake(getName()) + ".so";
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
  if (getState() == State::INITIALIZED || getState() == State::RUNNING)
  {
    stop(true);
  }

  for (auto& action_thread : action_threads_)
  {
    while (true)
    {
      {
        LOCK_GUARD_TYPE_R l(action_threads_rw_mutex_);
        if (!action_thread.second.is_running && action_thread.second.thread->joinable())
          break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    action_thread.second.thread->join();
  }
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
  if (getName() == "graph_entry" || getName() == "graph_exit")
  {
    return;
  }

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
  if (getName() == "graph_entry" || getName() == "graph_exit")
  {
    return;
  }

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
  
  do // do-while loop for 'spontaneous' actions
  {

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
    if (getState() == UmrfNode::State::NOT_SET && 
        getActorExecTraits() == UmrfNode::ActorExecTraits::LOCAL)
    {
      LOCK_GUARD_TYPE_R guard_action_instance(action_instance_rw_mutex_);
      instantiate();
      action_instance_->onInit();
      setState(State::INITIALIZED);
    }

    setState(State::RUNNING);

    /*
     * EXECUTE ACTION AS LOCAL
     */
    if (getActorExecTraits() == UmrfNode::ActorExecTraits::LOCAL)
    {
      result = (action_instance_->executeActionWrapped()) ? "on_true" : "on_false"; // Blocking call, returns when finished
      setOutputParameters(action_instance_->getUmrfNodeConst().getOutputParameters());
    }

    /*
     * EXECUTE ACTION AS REMOTE
     */
    else if (getActorExecTraits() == UmrfNode::ActorExecTraits::REMOTE)
    {
      TEMOTO_PRINT_OF("Waiting for '" + getActor() + "' to finish action '" + getFullName() + "'", ENGINE_HANDLE.getActor());
      result = waitUntilFinished(Waitable{
        .action_name = getFullName(),
        .actor_name = getActor(),
        .graph_name = parent_graph_name_});
    }

    /*
     * EXECUTE ACTION AS GRAPH
     */
    else if (getActorExecTraits() == UmrfNode::ActorExecTraits::GRAPH)
    {
      // TODO: capture the errors
      ENGINE_HANDLE.executeUmrfGraph(getName(), getInputParameters());
      result = waitUntilFinished(Waitable{
        .action_name = GRAPH_EXIT.getFullName(), 
        .graph_name = getName()});
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

  // Clear the parameters if local
  if (getActorExecTraits() == UmrfNode::ActorExecTraits::LOCAL)
  {
    LOCK_GUARD_TYPE_R guard_action_instance(action_instance_rw_mutex_);
    action_instance_->getUmrfNode().getInputParametersNc().clearData();
    action_instance_->getUmrfNode().getOutputParametersNc().clearData();
  }

  // Let the waiters know that the action (LOCAL or GRAPH) is finished
  if (getActorExecTraits() == UmrfNode::ActorExecTraits::LOCAL ||
      getActorExecTraits() == UmrfNode::ActorExecTraits::GRAPH)
  {
    Waitable waitable{.action_name = getFullName(), .actor_name = getActor(), .graph_name = parent_graph_name_};
    if (getActorExecTraits() == UmrfNode::ActorExecTraits::LOCAL && getName() == GRAPH_EXIT.getName())
    {
      ENGINE_HANDLE.notifyFinished(waitable, result, getInputParameters());
    }
    else
    {
      ENGINE_HANDLE.notifyFinished(waitable, result, getOutputParameters());
    }
  }

  if (getState() == UmrfNode::State::ERROR)
  {
    LOCK_GUARD_TYPE_R l(action_threads_rw_mutex_);
    TEMOTO_PRINT(action_threads_[UmrfNode::State::RUNNING].error_messages.what());
  }

  if (getState() == UmrfNode::State::RUNNING || getState() == UmrfNode::State::ERROR)
  {
    start_child_nodes_cb_(asRelation(), result);
  }
  else if (getState() == UmrfNode::State::STOPPING)
  {
    start_child_nodes_cb_(asRelation(), "on_stopped");
  }

  if (getType() == "spontaneous" && getState() != UmrfNode::State::STOPPING)
  {
    setState(State::FINISHED);
  }

  } // While loop
  while(getType() == "spontaneous" &&
        getState() != UmrfNode::State::STOPPING &&
        getState() != UmrfNode::State::ERROR);

  // Delete any input and output data due to std::any virtual deleter
  getInputParametersNc().clearData();
  getOutputParametersNc().clearData();

  {
    LOCK_GUARD_TYPE_R l(action_threads_rw_mutex_);
    action_threads_[UmrfNode::State::RUNNING].is_running = false;
  }

  if (getState() != UmrfNode::State::ERROR)
  {
    setState(State::FINISHED);
  }

  });
}

void UmrfNodeExec::stop(bool ignore_result)
{
  if (getName() == "graph_entry" || getName() == "graph_exit")
  {
    return;
  }

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
      action_instance_->stopAction(); // Blocking call, returns when finished
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

  if (getState() == UmrfNode::State::ERROR && !ignore_result)
  {
    start_child_nodes_cb_(asRelation(), "on_error");
  }

  {
    LOCK_GUARD_TYPE_R l(action_threads_rw_mutex_);
    action_threads_[UmrfNode::State::STOPPING].is_running = false;
  }
  });
}

void UmrfNodeExec::pause()
{
  if (getName() == "graph_entry" || getName() == "graph_exit")
  {
    return;
  }

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
    start_child_nodes_cb_(asRelation(), "on_error");
  }

  {
    LOCK_GUARD_TYPE_R l(action_threads_rw_mutex_);
    action_threads_[UmrfNode::State::PAUSED].is_running = false;
  }
  });
}

void UmrfNodeExec::bypass(const std::string& result)
{
  /*
   * TODO: There are many uncovered scenarios for bypass behavior.
   * For example what if the action is already running or stopping
   */

  clearThread(UmrfNode::State::BYPASSED);
  LOCK_GUARD_TYPE_R l(action_threads_rw_mutex_);

  /*
   * Run the bypass procedure in a thread
   */
  action_threads_[UmrfNode::State::BYPASSED] = UmrfNodeExec::ThreadWrapper();
  action_threads_[UmrfNode::State::BYPASSED].thread = std::make_shared<std::thread>([&]()
  {
    start_child_nodes_cb_(asRelation(), result);

    LOCK_GUARD_TYPE_R l(action_threads_rw_mutex_);
    action_threads_[UmrfNode::State::BYPASSED].is_running = false;
  });
}

void UmrfNodeExec::clearThread(const UmrfNode::State state_name)
{
  if (getName() == "graph_entry" || getName() == "graph_exit")
  {
    return;
  }

  LOCK_GUARD_TYPE_R l(action_threads_rw_mutex_);

  auto it = action_threads_.find(state_name);
  if (it == action_threads_.end())
  {
    return;
  }

  if (it->second.is_running)
  {
    throw CREATE_TEMOTO_ERROR_STACK("Cannot clear thread '" + state_to_str_map_.at(state_name) +
      "' of '" + getFullName() + "' because it's running");
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
  Waiter waiter{.action_name = getFullName(), .graph_name = parent_graph_name_};
  ENGINE_HANDLE.addWaiter(waitable, waiter);

  wait_ = true;
  std::unique_lock<std::mutex> lock(wait_cv_mutex_);
  wait_cv_.wait(lock, [&]{return !wait_;});

  // send acknowledgement, blocks until consensus is reached among all actors
  ENGINE_HANDLE.acknowledge(waitable, waiter);

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