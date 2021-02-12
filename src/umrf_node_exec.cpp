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

UmrfNodeExec::UmrfNodeExec(const UmrfNode& umrf_node)
: UmrfNode(umrf_node)
{
  try
  {
    LOCK_GUARD_TYPE guard_class_loader(class_loader_rw_mutex_);
    class_loader_ = std::make_shared<class_loader::ClassLoader>(getLibraryPath(), false);

    // Check if the classloader actually contains the required action
    std::vector<std::string> classes_in_classloader = class_loader_->getAvailableClasses<ActionBase>();
    bool class_found = false;
    for (const auto& class_in_classloader : classes_in_classloader)
    {
      if (class_in_classloader == getName())
      {
        class_found = true;
        break;
      }
    }

    if (!class_found)
    {
      TEMOTO_PRINT("Failed to initialize the Action Handle because the action class name is incorrect");
      setState(State::ERROR);
      return;
    }
  }
  catch(const std::exception& e)
  {
    setState(State::ERROR);
    std::cerr << "Failed to initialize the Action Handle because: " << e.what() << '\n';
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
  return UmrfNode(*this);
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
    while(getState() == State::RUNNING)
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
    stopNode(10);
    LOCK_GUARD_TYPE guard_action_instance(action_instance_rw_mutex_);
    action_instance_.reset();
    setState(State::UNINITIALIZED);
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

void UmrfNodeExec::instantiate(std::shared_ptr<std::condition_variable> notify_cv
, std::shared_ptr<std::mutex> notify_cv_mutex
, NotifyGraphCallback notify_graph_cb)
{ 
  LOCK_GUARD_TYPE guard_action_instance(action_instance_rw_mutex_);
  LOCK_GUARD_TYPE guard_class_loader(class_loader_rw_mutex_);

  if (getState() == State::ERROR)
  {
    throw CREATE_TEMOTO_ERROR_STACK("Cannot instantiate the action because it's in error state");
  }
  else if (getState() == State::INSTANTIATED)
  {
    throw CREATE_TEMOTO_ERROR_STACK("The action is already instantiated");
  }

  try
  {
    action_instance_ = class_loader_->createInstance<ActionBase>(getName());
    action_instance_->setUmrf(UmrfNode(*this));

    notify_cv_ = notify_cv;
    notify_cv_mutex_ = notify_cv_mutex;
    notify_graph_callback_ = notify_graph_cb;

    if (inputParametersReceived() && requiredParentsFinished())
    {
      setState(State::READY);
    }
    else
    {
      setState(State::INSTANTIATED);
    }
  }
  catch(const std::exception& e)
  {
    setState(State::ERROR);
    throw CREATE_TEMOTO_ERROR_STACK("Failed to create an instance of the action: " + std::string(e.what()));
  }
}

TemotoErrorStack UmrfNodeExec::startNode()
{
  // TODO: If the action is running (blocking), and the action instance is guarded
  // with a mutex at the same time, then the action cannot be stopped because the mutex is locked
  // while the action is running
  //LOCK_GUARD_TYPE guard_action_instance(action_instance_rw_mutex_);
  
  if (getState() != State::READY)
  {
    return CREATE_TEMOTO_ERROR_STACK("Cannot execute the action because it's not in READY state");
  }
  try
  {
    setState(State::RUNNING);
    action_instance_->executeActionWrapped(); // Blocking call, returns when finished

    if ((getState() == State::RUNNING))
    {
      notify_graph_callback_(getFullName(), action_instance_->getUmrfNodeConst().getOutputParameters());
    }
    setState(State::FINISHED);

    // Since this method is meant to be executed asynchonously, then any potential errors are passed
    // via an ErrorStack. But if there are no errors, then return an empty error stack.
    return TemotoErrorStack();
  }
  catch(TemotoErrorStack e)
  {
    setState(State::ERROR);
    return FORWARD_TEMOTO_ERROR_STACK(e);
  }
  catch(const std::exception& e)
  {
    setState(State::ERROR);
    return CREATE_TEMOTO_ERROR_STACK(std::string(e.what()));
  }
  catch(...)
  {
    setState(State::ERROR);
    return CREATE_TEMOTO_ERROR_STACK("Caught an unhandled exception.");
  }
}

void UmrfNodeExec::startNodeThread()
{
  umrf_node_exec_thread_ = std::thread(&UmrfNodeExec::umrfNodeExecThread, this);
}

void UmrfNodeExec::umrfNodeExecThread()
{
  if (getState() != State::READY)
  {
    throw CREATE_TEMOTO_ERROR_STACK("Cannot execute the action because it's not in READY state");
  }
  try
  {
    error_messages_ = startNode();
  }
  catch(const std::exception& e)
  {
    setState(State::ERROR);
    throw CREATE_TEMOTO_ERROR_STACK("Cannot start the action thread: " + std::string(e.what()));
  }

  // Notify the graph that this node has finished executing
  notify_cv_mutex_->lock();
  notify_cv_->notify_all();
  notify_cv_mutex_->unlock();
}

void UmrfNodeExec::joinUmrfNodeExecThread()
{
  umrf_node_exec_thread_.join();
}

void UmrfNodeExec::updateInstanceParams(const ActionParameters& ap_in)
{
  LOCK_GUARD_TYPE guard_action_instance(action_instance_rw_mutex_);
  try
  {
    if (getState() != State::INSTANTIATED || 
        getState() != State::READY || 
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

bool UmrfNodeExec::getInctanceInputParametersReceived() const
{
  LOCK_GUARD_TYPE guard_action_instance(action_instance_rw_mutex_);
  if (getState() != State::INSTANTIATED)
  {
    throw CREATE_TEMOTO_ERROR_STACK("The action is not instantiated");
  }
  return action_instance_->getUmrfNodeConst().inputParametersReceived();
}