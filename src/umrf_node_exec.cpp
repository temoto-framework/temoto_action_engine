/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright 2020 TeMoto Telerobotics
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

#include "temoto_action_engine/umrf_node_exec.h"
#include "temoto_action_engine/basic_timer.h"
#include "temoto_action_engine/messaging.h"
#include "temoto_action_engine/umrf_json.h"

UmrfNodeExec::UmrfNodeExec(const UmrfNode& umrf_node)
: UmrfNode(umrf_node)
{
  if (getActorExecTraits() == UmrfNode::ActorExecTraits::LOCAL)
  try
  {
    LOCK_GUARD_TYPE guard_class_loader(class_loader_rw_mutex_);
    class_loader_ = std::make_shared<class_loader::ClassLoader>(getLibraryPath(), false);

    // Check if the classloader actually contains the required action
    std::vector<std::string> classes_in_classloader = class_loader_->getAvailableClasses<ActionBase>();
    bool class_found = [&]{
    for (const auto& class_in_classloader : classes_in_classloader)
    {
      if (class_in_classloader == getName())
      {
        return true;
      }
    }}();

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

// UmrfNodeExec::UmrfNodeExec(const UmrfNodeExec& unx)
// : UmrfNode(unx)
// , action_future_(unx.action_future_)
// //, class_loader_(unx.class_loader_)
// //, action_instance_(unx.action_instance_)
// , future_retreived_(unx.future_retreived_)
// , default_stopping_timeout_(unx.default_stopping_timeout_)
// {}

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

bool UmrfNodeExec::stopNode(float timeout)
{
  LOCK_GUARD_TYPE guard_action_instance(action_instance_rw_mutex_);
  if (getState() == State::RUNNING)
  {
    setState(State::STOP_REQUESTED);
    action_instance_->stopAction();

    // Wait until the action is finished
    Timer timeout_timer;
    while(getState() == State::STOP_REQUESTED)
    {
      if (timeout_timer.elapsed() >= timeout)
      {
        setState(State::ERROR);
        throw CREATE_TEMOTO_ERROR_STACK("Reached the timeout of " + std::to_string(timeout) + " seconds.");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    setState(State::FINISHED);
  }
  return true;
}

const TemotoErrorStack& UmrfNodeExec::getErrorMessages() const
{
  return error_messages_;
}

void UmrfNodeExec::clearNode()
{
  try
  {
    if (getState() != State::UNINITIALIZED)
    {
      stopNode(10);
      LOCK_GUARD_TYPE guard_action_instance(action_instance_rw_mutex_);
      action_instance_.reset();
      setState(State::UNINITIALIZED);
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
}

void UmrfNodeExec::instantiate()
try
{
  LOCK_GUARD_TYPE guard_action_instance(action_instance_rw_mutex_);
  LOCK_GUARD_TYPE guard_class_loader(class_loader_rw_mutex_);

  if (getState() != State::UNINITIALIZED)
  {
    throw CREATE_TEMOTO_ERROR_STACK("Cannot instantiate the action because it's not in UNINITIALIZED state");
  }

  action_instance_ = class_loader_->createInstance<ActionBase>(getName());
  action_instance_->setUmrf(UmrfNode(*this));
}
catch(const std::exception& e)
{
  setState(State::ERROR);
  throw CREATE_TEMOTO_ERROR_STACK("Failed to create an instance of the action: " + std::string(e.what()));
}

void UmrfNodeExec::startNode()
{
  // TODO: If the action is running (blocking), and the action instance is guarded
  // with a mutex at the same time, then the action cannot be stopped because the mutex is locked
  // while the action is running
  //LOCK_GUARD_TYPE guard_action_instance(action_instance_rw_mutex_);

  if (getState() != State::READY && getState() != State::FINISHED)
  {
    error_messages_ = CREATE_TEMOTO_ERROR_STACK("Cannot execute the action because it's not in READY or FINISHED state");
    return;
  }

  std::string result;
  try
  {
    setState(State::RUNNING);
    result = (action_instance_->executeActionWrapped()) ? "on_true" : "on_false"; // Blocking call, returns when finished

    // If the action finished without external interruption (stop request) then start the child nodes
    if (getState() == State::RUNNING)
    {
      start_child_nodes_cb_(getFullName(), action_instance_->getUmrfNodeConst().getOutputParameters(), result);
    }

    if (!getIsRemoteActor())
    {
      setLatestUmrfJsonStr(umrf_json::toUmrfJsonStr(action_instance_->getUmrfNodeConst()));
    }
    setState(State::FINISHED);
  }
  catch(TemotoErrorStack e)
  {
    // TODO: implement the behavior for "on_error" result
    setState(State::ERROR);
    error_messages_ = FORWARD_TEMOTO_ERROR_STACK(e);
  }
}

void UmrfNodeExec::startNodeThread()
{
  umrf_node_exec_thread_ = std::thread(&UmrfNodeExec::umrfNodeExecThread, this);
}

void UmrfNodeExec::umrfNodeExecThread()
{
  if (getState() != State::READY && getState() != State::FINISHED)
  {
    throw CREATE_TEMOTO_ERROR_STACK("Cannot execute the action because it's not in READY or FINISHED state");
  }
  try
  {
    umrf_node_exec_thread_running_ = true;
    startNode();
  }
  catch(TemotoErrorStack e)
  {
    error_messages_.appendError(FORWARD_TEMOTO_ERROR_STACK(e));
  }
  catch(const std::exception& e)
  {
    error_messages_ = CREATE_TEMOTO_ERROR_STACK(std::string(e.what()));
  }
  catch(...)
  {  
    error_messages_ = CREATE_TEMOTO_ERROR_STACK("Caught an unhandled error.");
  }

  /*
   * Notify the graph that the action has finished, regardless whether the action finished
   * with an error or not
   */
  notify_finished_cb_(getFullName());
  umrf_node_exec_thread_running_ = false;
}

void UmrfNodeExec::updateInstanceParams(const ActionParameters& ap_in)
{
  LOCK_GUARD_TYPE guard_action_instance(action_instance_rw_mutex_);
  try
  {
    if (getState() != State::INSTANTIATED && 
        getState() != State::READY && 
        getState() != State::RUNNING)
    {
      throw CREATE_TEMOTO_ERROR_STACK("Cannot update action's parameters because it's not in INSTANTIATED, READY or RUNNING state");
    }
    action_instance_->updateParameters(ap_in);

    if (action_instance_->getUmrfNodeConst().inputParametersReceived() && 
        requiredParentsFinished() &&
        getState() == State::INSTANTIATED)
    {
      setState(State::READY);
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
    throw CREATE_TEMOTO_ERROR_STACK("Caught an unhandled exception.");
  }
}

bool UmrfNodeExec::getInstanceInputParametersReceived() const
{
  LOCK_GUARD_TYPE guard_action_instance(action_instance_rw_mutex_);
  if (getState() != State::INSTANTIATED)
  {
    throw CREATE_TEMOTO_ERROR_STACK("The action is not instantiated");
  }
  return action_instance_->getUmrfNodeConst().inputParametersReceived();
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
      // TODO: 1) wait for state change notification. get the result
      // result = sync_handle.getResult(getFullName());

      // TODO: 2) send acknowledgement via syncer::acknowledge(std::string action_name) std::function
      //          blocks until consensus is reached among all actors 
    }

    /*
    * EXECUTE ACTION AS GRAPH
    */
    else if (getActorExecTraits() == UmrfNode::ActorExecTraits::GRAPH)
    {
      // TODO: result = action_handle.waitGraphResult();
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

  if (getState() == UmrfNode::State::RUNNING)
  {
    if (getActorExecTraits() == UmrfNode::ActorExecTraits::LOCAL)
    {
      // TODO: sync_handle.syncResult(getFullName(), result);
    }

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

void UmrfNode::stop()
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
      action_instance_->onStop() // Blocking call, returns when finished
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
    LOCK_GUARD_TYPE guard_action_instance(action_instance_rw_mutex_);
    // TODO: sync_handle.notify(getFullName(), getState());
    action_instance_.reset();
    setState(State::NOT_SET);
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

void UmrfNodeExec::clearThread(const UmrfNode::State state_name)
{
  LOCK_GUARD_TYPE_R l(action_threads_rw_mutex_);

  auto& it = action_threads_.find(state_name);
  if (it == action_threads_.end())
  {
    return;
  }

  if (it->is_running)
  {
    throw CREATE_TEMOTO_ERROR_STACK("Cannot clear thread '" + std::to_string(state_name) + "' because it's running");
  }

  if (it->thread.joinable())
  {
    it->thread.join();
  }

  action_threads_.erase(it);
}