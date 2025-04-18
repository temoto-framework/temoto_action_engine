/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright 2025 TeMoto Telerobotics
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
#include "temoto_action_engine/umrf_json.h"
#include "temoto_action_engine/util/basic_timer.hpp"
#include "temoto_action_engine/util/logging.hpp"

UmrfNodeExec::UmrfNodeExec(const UmrfNode& umrf_node)
: UmrfNode(umrf_node)
{
  if (getActorExecTraits() == UmrfNode::ActorExecTraits::LOCAL &&
      getName() != "graph_entry" &&
      getName() != "graph_exit")
  try
  {
    LOCK_GUARD_TYPE_R guard_action_plugin(action_plugin_rw_mutex_);
    action_plugin_ = std::make_shared<ActionPlugin<ActionBase>>(getName());

    setLatestUmrfJsonStr(umrf_json::toUmrfJsonStr(*this));
  }
  catch(const std::exception& e)
  {
    setState(State::ERROR);
    std::cerr << "Failed to initialize the Action Handle because: " << e.what() << '\n';
    notify_state_change_cb_(getFullName());
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
  std::string umrf_node_string = umrf_json::toUmrfJsonStr(*this);
  return umrf_json::fromUmrfJsonStr(umrf_node_string);
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

  for (auto& action_thread : state_threads_)
  {
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

  LOCK_GUARD_TYPE_R guard_action_plugin(action_plugin_rw_mutex_);

  if (getState() != State::UNINITIALIZED)
  {
    throw CREATE_TEMOTO_ERROR_STACK("Cannot instantiate the action because it's not in UNINITIALIZED state");
  }

  action_plugin_->load();
  action_plugin_->get()->setUmrf(UmrfNode(*this));
  action_plugin_->get()->onInit();
  setState(State::INITIALIZED);
}
catch(const std::exception& e)
{
  setState(State::ERROR);
  notify_state_change_cb_(getFullName());
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

  /*
   * Run the action in a thread
   */
  state_threads_.start(UmrfNode::State::RUNNING, std::make_shared<std::thread>([&]()
  {

  do // do-while loop for 'spontaneous' actions
  {

  std::string result;
  bool action_threw_error{false};
  try
  {
    // Move to the error state if the action is not in the following states
    if (getState() != UmrfNode::State::UNINITIALIZED &&
        getState() != UmrfNode::State::INITIALIZED   &&
        getState() != UmrfNode::State::FINISHED      &&
        getState() != UmrfNode::State::ERROR)
    {
      throw CREATE_TEMOTO_ERROR_STACK("Action has to be in the following states to run: UNINITIALIZED; INITIALIZED; FINISHED. Current state: "
      + state_to_str_map_.at(getState()));
    }

    // Initialize the action
    if (getActorExecTraits() == UmrfNode::ActorExecTraits::LOCAL)
    {
      if (getState() == UmrfNode::State::UNINITIALIZED)
      {
        // Initialize the action
        instantiate();
      }
      else if (getState() == UmrfNode::State::ERROR)
      {
        // Should the action be reset?
      }
    }

    setState(State::RUNNING);
    notify_state_change_cb_(getFullName());

    /*
     * EXECUTE ACTION AS LOCAL
     */
    if (getActorExecTraits() == UmrfNode::ActorExecTraits::LOCAL)
    {
      action_plugin_->get()->setUmrf(UmrfNode(*this));
      result = (action_plugin_->get()->executeActionWrapped()) ? "on_true" : "on_false"; // Blocking call, returns when finished
      setOutputParameters(action_plugin_->get()->getUmrfNodeConst().getOutputParameters());
    }

    /*
     * EXECUTE ACTION AS GRAPH
     */
    else if (getActorExecTraits() == UmrfNode::ActorExecTraits::GRAPH)
    {
      // Assign an unique name to the child graph
      std::string child_graph_name = getSubGraphName();

      // TODO: capture the errors
      ENGINE_HANDLE.executeUmrfGraph(getName(), getInputParameters(), "on_true", child_graph_name);

      result = waitUntilFinished(Waitable{
        .action_name = GRAPH_EXIT.getFullName(),
        .graph_name = child_graph_name,
        .actor_name = getActor()});
    }

    /*
     * EXECUTE ACTION AS REMOTE
     */
    else if (getActorExecTraits() == UmrfNode::ActorExecTraits::REMOTE)
    {
      TEMOTO_PRINT_OF("Waiting for '" + getActor() + "' to finish action '" + getFullName() + "'", ENGINE_HANDLE.getActor());
      result = waitUntilFinished(Waitable{
        .action_name = getFullName(),
        .graph_name = parent_graph_name_,
        .actor_name = getActor()});
    }

  } // try end
  catch(TemotoErrorStack e)
  {
    state_threads_.addErrorMessage(UmrfNode::State::RUNNING, FORWARD_TEMOTO_ERROR_STACK(e));
    action_threw_error = true;
  }
  catch(const std::exception& e)
  {
    state_threads_.addErrorMessage(UmrfNode::State::RUNNING, CREATE_TEMOTO_ERROR_STACK(std::string(e.what())));
    action_threw_error = true;
  }
  catch(...)
  {
    state_threads_.addErrorMessage(UmrfNode::State::RUNNING, CREATE_TEMOTO_ERROR_STACK("Caught an unhandled error."));
    action_threw_error = true;
  }

  // Get the log
  if (getActorExecTraits() == UmrfNode::ActorExecTraits::LOCAL)
  {
    log_ = action_plugin_->get()->getUmrfNodeConst().getLog();
  }

  // If the action was paused, then wait until it is resumed, even if the action threw an error
  if (getState() == State::PAUSE_REQUESTED || getState() == State::PAUSED)
  {
    wait_for_resume_local_.wait();
  }

  // If the action threw an error, change the state
  if (action_threw_error)
  {
    setState(State::ERROR);
    notify_state_change_cb_(getFullName());
    result = "on_error";
  }

  // Clear the parameters if local
  if (getActorExecTraits() == UmrfNode::ActorExecTraits::LOCAL)
  {
    LOCK_GUARD_TYPE_R quard_action_plugin(action_plugin_rw_mutex_);
    action_plugin_->get()->getUmrfNode().getInputParametersNc().clearData();
    action_plugin_->get()->getUmrfNode().getOutputParametersNc().clearData();
  }

  // Let the waiters know that the action (LOCAL or GRAPH) is finished
  if (getActorExecTraits() != UmrfNode::ActorExecTraits::REMOTE)
  {
    Waitable waitable{.action_name = getFullName(), .graph_name = parent_graph_name_, .actor_name = getActor()};
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
    TEMOTO_PRINT(state_threads_.getErrorMessages(UmrfNode::State::RUNNING));
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
    notify_state_change_cb_(getFullName());
  }

  } // While loop
  while(getType() == "spontaneous" &&
        getState() != UmrfNode::State::STOPPING &&
        getState() != UmrfNode::State::ERROR);

  // Delete any input and output data due to std::any virtual deleter

  getInputParametersNc().clearData();
  getOutputParametersNc().clearData();

  state_threads_.done(UmrfNode::State::RUNNING);

  if (getState() != UmrfNode::State::ERROR)
  {
    setState(State::FINISHED);
    notify_state_change_cb_(getFullName());
  }

  }));
}

std::future<void> UmrfNodeExec::stop(bool ignore_result)
{
  std::promise<void> promise;
  std::future<void> future = promise.get_future();

  if (getName() == "graph_entry" || getName() == "graph_exit")
  {
    promise.set_value();
    return future;
  }

  if (getState() == UmrfNode::State::STOPPING ||
      getState() == UmrfNode::State::FINISHED ||
      getState() == UmrfNode::State::ERROR    ||
      getState() == UmrfNode::State::UNINITIALIZED)
  {
    promise.set_value();
    return future;
  }

  /*
   * Run the stopping procedure in a thread
   */
  state_threads_.start(UmrfNode::State::STOPPING, std::make_shared<std::thread>([&, promise_ = std::move(promise)] () mutable
  {
  try
  {
    setState(State::STOPPING);
    notify_state_change_cb_(getFullName());

    /*
     * LOCALLY STOPPED
     */
    if (getActorExecTraits() == UmrfNode::ActorExecTraits::LOCAL)
    {
      // Invoke the stop routine inside the action plugin
      action_plugin_->get()->stopAction();

      // Wait until the action stops
      auto timeout{std::chrono::seconds(5)};
      auto start_time{std::chrono::system_clock::now()};

      while (getState() != UmrfNode::State::FINISHED && getState() != UmrfNode::State::ERROR)
      {
        if (std::chrono::system_clock::now() - start_time > timeout)
        {
          throw CREATE_TEMOTO_ERROR_STACK("Reached the timeout while waiting for action '" + getFullName() + "' to stop.");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }

    /*
     * STOP THE SUBGRAPH
     */
    else if (getActorExecTraits() == UmrfNode::ActorExecTraits::GRAPH)
    {
      ENGINE_HANDLE.stopGraph(getSubGraphName()); // Blocking call, returns when finished
    }

    /*
     * REMOTELY_STOPPED
     */
    else if (getActorExecTraits() == UmrfNode::ActorExecTraits::REMOTE)
    {
      // TODO: sync_handle.waitForStop(getFullName())
    }

  } // try end
  catch(TemotoErrorStack e)
  {
    state_threads_.addErrorMessage(UmrfNode::State::STOPPING, FORWARD_TEMOTO_ERROR_STACK(e));
    setState(State::ERROR);
    notify_state_change_cb_(getFullName());
  }
  catch(const std::exception& e)
  {
    state_threads_.addErrorMessage(UmrfNode::State::STOPPING, CREATE_TEMOTO_ERROR_STACK(std::string(e.what())));
    setState(State::ERROR);
    notify_state_change_cb_(getFullName());
  }
  catch(...)
  {
    state_threads_.addErrorMessage(UmrfNode::State::STOPPING, CREATE_TEMOTO_ERROR_STACK("Caught an unhandled error."));
    setState(State::ERROR);
    notify_state_change_cb_(getFullName());
  }

  if (getState() == UmrfNode::State::ERROR && !ignore_result)
  {
    start_child_nodes_cb_(asRelation(), "on_error");
  }

  state_threads_.done(UmrfNode::State::STOPPING);

  if (getState() == UmrfNode::State::ERROR)
  {
    try { throw(state_threads_.getErrorStack(UmrfNode::State::STOPPING)); }
    catch(...) {promise_.set_exception(std::current_exception()); }
  }
  else
  {
    promise_.set_value();
  }

  }));

  return future;
}

void UmrfNodeExec::pause()
{
  if (getName() == "graph_entry" || getName() == "graph_exit")
  {
    return;
  }

  if (getState() != UmrfNode::State::RUNNING)
  {
    return;
  }

  /*
   * Run the pausing procedure in a thread
   */
  state_threads_.start(UmrfNode::State::PAUSED, std::make_shared<std::thread>([&]()
  {
  try
  {
    setState(State::PAUSE_REQUESTED);

    /*
     * LOCALLY PAUSED
     */
    if (getActorExecTraits() == UmrfNode::ActorExecTraits::LOCAL)
    {
      LOCK_GUARD_TYPE_R quard_action_plugin(action_plugin_rw_mutex_);
      action_plugin_->get()->onPause(); // Blocking call, returns when finished

      /*
       * If the pause is handled by the action plugin (onPause method overridden). If not
       * then the state will be changed to PAUSED after the action has finished
       */
      if (!action_plugin_->get()->pause_not_handled_)
      {
        setState(State::PAUSED);
      }

      // Wait until the action is externally unpaused
      wait_for_resume_.wait();
      setState(State::RUNNING);

      // Tell the local run() thread to stop waiting
      action_plugin_->get()->onResume();
      wait_for_resume_local_.stopWaiting();
    }

    /*
     * PAUSE THE SUBGRAPH
     */
    else if (getActorExecTraits() == UmrfNode::ActorExecTraits::GRAPH)
    {
      // TODO: engine_handle.pauseGraph( TODO )
      // wait_for_resume_.wait(); // Wait until the action is externally unpaused
    }

    /*
     * REMOTELY PAUSED
     */
    else if (getActorExecTraits() == UmrfNode::ActorExecTraits::REMOTE)
    {
      wait_for_resume_.wait(); // Wait until the action is externally unpaused
    }

  } // try end
  catch(TemotoErrorStack e)
  {
    state_threads_.addErrorMessage(UmrfNode::State::PAUSED, FORWARD_TEMOTO_ERROR_STACK(e));
    setState(State::ERROR);
    notify_state_change_cb_(getFullName());
  }
  catch(const std::exception& e)
  {
    state_threads_.addErrorMessage(UmrfNode::State::PAUSED, CREATE_TEMOTO_ERROR_STACK(std::string(e.what())));
    setState(State::ERROR);
    notify_state_change_cb_(getFullName());
  }
  catch(...)
  {
    state_threads_.addErrorMessage(UmrfNode::State::PAUSED, CREATE_TEMOTO_ERROR_STACK("Caught an unhandled error."));
    setState(State::ERROR);
    notify_state_change_cb_(getFullName());
  }

  if (getActorExecTraits() == UmrfNode::ActorExecTraits::LOCAL)
  {
    // TODO: sync_handle.syncState(getFullName(), getState());
  }

  if (getState() == UmrfNode::State::ERROR)
  {
    start_child_nodes_cb_(asRelation(), "on_error");
  }

  state_threads_.done(UmrfNode::State::PAUSED);

  }));
}

void UmrfNodeExec::resume()
{
  if (getState() != UmrfNode::State::PAUSED)
  {
    return;
  }

  wait_for_resume_.stopWaiting();
}

void UmrfNodeExec::bypass(const std::string& result)
{
  /*
   * TODO: There are many uncovered scenarios for bypass behavior.
   * For example what if the action is already running or stopping
   */

  /*
   * Run the bypass procedure in a thread
   */
  state_threads_.start(UmrfNode::State::BYPASSED, std::make_shared<std::thread>([&]()
  {
    start_child_nodes_cb_(asRelation(), result);
    state_threads_.done(UmrfNode::State::BYPASSED);
  }));
}

void UmrfNodeExec::setGraphName(const std::string& parent_graph_name)
{
  parent_graph_name_ = parent_graph_name;
}

std::string UmrfNodeExec::waitUntilFinished(const Waitable& waitable)
{
  Waiter waiter{.action_name = getFullName(), .graph_name = parent_graph_name_, .actor_name = getActor(),};
  ENGINE_HANDLE.addWaiter(waitable, waiter);

  wait_for_result_.wait();

  if (getActor() != ENGINE_HANDLE.getActor() && !ENGINE_HANDLE.getActor().empty())
    ENGINE_HANDLE.acknowledge(remote_notification_id_);

  return remote_result_;
}

void UmrfNodeExec::notifyFinished(const std::string& remote_notification_id)
{
  remote_notification_id_ = remote_notification_id;
  wait_for_result_.stopWaiting();
}

void UmrfNodeExec::setRemoteResult(const std::string& remote_result)
{
  remote_result_ = remote_result;
}

std::string UmrfNodeExec::getSubGraphName() const
{
  return parent_graph_name_ + "__" + getName() + "__" + std::to_string(getInstanceId());
}
