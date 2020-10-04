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

#include "temoto_action_engine/action_handle.h"
#include "temoto_action_engine/action_executor.h"
#include "temoto_action_engine/basic_timer.h"
#include "temoto_action_engine/action_base.h"
#include "temoto_action_engine/messaging.h"

ActionHandle::ActionHandle(Umrf umrf, ActionExecutor* action_executor_ptr)
: state_(ActionHandle::State::UNINITIALIZED)
, action_executor_ptr_(action_executor_ptr)
, umrf_(std::make_shared<Umrf>(umrf))
, future_retreived_(false)
{
  /*
   * TODO: If the library does not exist, then handle the situation accordingly
   */
  if (!umrf_->isUmrfCorrect())
  {
    TEMOTO_PRINT("Failed to initialize the Action Handle because the UMRF is ill formated");
    setState(ActionHandle::State::ERROR);
    return;
  }

  if (action_executor_ptr_ == nullptr)
  {
    TEMOTO_PRINT("Failed to initialize the Action Handle because Action Executioner pointer is null");
    setState(ActionHandle::State::ERROR);
    return;
  }

  try
  {
    LOCK_GUARD_TYPE guard_class_loader(class_loader_rw_mutex_);
    class_loader_ = std::make_shared<class_loader::ClassLoader>(umrf.getLibraryPath(), false);

    // Check if the classloader actually contains the required action
    std::vector<std::string> classes_in_classloader = class_loader_->getAvailableClasses<ActionBase>();
    bool class_found = false;
    for (const auto& class_in_classloader : classes_in_classloader)
    {
      if (class_in_classloader == umrf_->getName())
      {
        class_found = true;
        break;
      }
    }

    if (!class_found)
    {
      TEMOTO_PRINT("Failed to initialize the Action Handle because the action class name is incorrect");
      setState(ActionHandle::State::ERROR);
      return;
    }
    if (umrf_->inputParametersReceived())
    {
      setState(ActionHandle::State::INITIALIZED);
    }
  }
  catch(const std::exception& e)
  {
    std::cerr << "Failed to initialize the Action Handle because: " << e.what() << '\n';
  }
}

ActionHandle::ActionHandle(const ActionHandle& ah)
: action_executor_ptr_(ah.action_executor_ptr_)
, state_(ah.state_)
, action_future_(ah.action_future_)
, class_loader_(ah.class_loader_)
, umrf_(ah.umrf_)
, action_instance_(ah.action_instance_)
, future_retreived_(ah.future_retreived_)
{}

const ActionHandle::State& ActionHandle::getState() const
{
  LOCK_GUARD_TYPE guard_state(state_rw_mutex_);
  return state_;
}

bool ActionHandle::setState(ActionHandle::State state_to_set)
{
  LOCK_GUARD_TYPE guard_state(state_rw_mutex_);
  LOCK_GUARD_TYPE_R guard_umrf(umrf_rw_mutex_);
  if(state_ == state_to_set)
  {
    return false;
  }
  else
  {
    // TEMOTO_PRINT_OF("Changing state from " + state_to_str_map_[state_] + " to " + state_to_str_map_[state_to_set]
    //                , umrf_->getFullName());
    state_ = state_to_set;
    return true; 
  }
}

void ActionHandle::instantiateAction()
{ 
  LOCK_GUARD_TYPE guard_action_instance(action_instance_rw_mutex_);
  LOCK_GUARD_TYPE guard_class_loader(class_loader_rw_mutex_);

  if (getState() != ActionHandle::State::INITIALIZED)
  {
    throw CREATE_TEMOTO_ERROR_STACK("Cannot instantiate the action because it's not initialized");
  }

  try
  {
    LOCK_GUARD_TYPE_R guard_umrf(umrf_rw_mutex_);
    action_instance_ = class_loader_->createInstance<ActionBase>(umrf_->getName());
    action_instance_->setUmrf(umrf_);
    setState(ActionHandle::State::READY);
  }
  catch(const std::exception& e)
  {
    setState(ActionHandle::State::ERROR);
    throw CREATE_TEMOTO_ERROR_STACK("Failed to create an instance of the action: " + std::string(e.what()));
  }
}

TemotoErrorStack ActionHandle::executeAction()
{
  // TODO: If the action will block, and the action instance is guarded
  // with a mutex at the same time, then the action cannot be stopped
  //LOCK_GUARD_TYPE guard_action_instance(action_instance_rw_mutex_);
  
  if (getState() != ActionHandle::State::READY)
  {
    return CREATE_TEMOTO_ERROR_STACK("Cannot execute the action because it's not in READY state");
  }
  try
  {
    setState(ActionHandle::State::RUNNING);
    future_retreived_ = false;
    action_instance_->executeActionWrapped(); // Blocking call, returns when finished

    if ((getState() == ActionHandle::State::RUNNING))
    {
      LOCK_GUARD_TYPE_R guard_umrf(umrf_rw_mutex_);
      action_executor_ptr_->notifyFinished(
        umrf_->getId()
      , umrf_->getOutputParameters());

      /*
       * TODO: if the parameters are not cleared over here, then action handle cleanup might crash
       * in case the parameters are shared. This could be attributed to how boost::any works but at
       * this point I am clueless why the crash happens. If the parameters are cleared in the 
       * destructor of the ActionHandle, then the crashing behavior still remains. It seems as if
       * the data which the parameter is "pointing" to is magically invalid and leads to a crash.
       * 
       * PS: this means that as long as this particular TODO exists, pointers should not be passed
       *     between actions!
       */ 
      umrf_->getOutputParametersNc().clear();
      //umrf_->getInputParametersNc().clear();
    }
    setState(ActionHandle::State::FINISHED);

    // Since this method is meant to be executed asynchonously, then any potential errors are passed
    // via an ErrorStack as a future value. But if there are no errors, then return an empty error stack.
    return TemotoErrorStack();
  }
  catch(TemotoErrorStack e)
  {
    setState(ActionHandle::State::ERROR);
    return FORWARD_TEMOTO_ERROR_STACK(e);
  }
  catch(const std::exception& e)
  {
    setState(ActionHandle::State::ERROR);
    return CREATE_TEMOTO_ERROR_STACK(std::string(e.what()));
  }
  catch(...)
  {
    setState(ActionHandle::State::ERROR);
    return CREATE_TEMOTO_ERROR_STACK("Caught an unhandled error.");
  }
}

void ActionHandle::executeActionThread()
{
  LOCK_GUARD_TYPE_R guard_action_future(action_future_rw_mutex_);
  if (getState() != ActionHandle::State::READY)
  {
    throw CREATE_TEMOTO_ERROR_STACK("Cannot execute the action because it's not in READY state");
  }
  try
  {
    action_future_ = std::make_shared<std::future<TemotoErrorStack>>(std::async( std::launch::async, &ActionHandle::executeAction, this));
  }
  catch(const std::exception& e)
  {
    setState(ActionHandle::State::ERROR);
    throw CREATE_TEMOTO_ERROR_STACK("Cannot start the action thread: " + std::string(e.what()));
  }
}

void ActionHandle::stopAction(double timeout)
{
  LOCK_GUARD_TYPE guard_action_instance(action_instance_rw_mutex_);
  if (getState() == ActionHandle::State::RUNNING)
  {
    setState(ActionHandle::State::STOP_REQUESTED);
    action_instance_->stopAction();

    // Wait until the action is finished
    Timer timeout_timer;
    while(!futureIsReady())
    {
      if (timeout_timer.elapsed() >= timeout)
      {
        setState(ActionHandle::State::ERROR);
        throw CREATE_TEMOTO_ERROR_STACK("Reached the timeout of " + std::to_string(timeout) + " seconds.");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    setState(ActionHandle::State::FINISHED);
  }
}

void ActionHandle::clearAction()
{
  try
  {
    stopAction(5);
    LOCK_GUARD_TYPE guard_action_instance(action_instance_rw_mutex_);
    LOCK_GUARD_TYPE_R guard_action_future(action_future_rw_mutex_);
    action_instance_.reset();
    action_future_.reset();
    setState(ActionHandle::State::INITIALIZED);
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

bool ActionHandle::futureIsReady()
{
  LOCK_GUARD_TYPE_R guard_action_future(action_future_rw_mutex_);
  if (action_future_.use_count() > 0)
  {
    return action_future_->wait_for(std::chrono::seconds(0)) == std::future_status::ready;
  }
  else
  {
    return true;
  }
}

TemotoErrorStack ActionHandle::getFutureValue()
{
  LOCK_GUARD_TYPE_R guard_action_future(action_future_rw_mutex_);
  if (!futureIsReady())
  {
    throw CREATE_TEMOTO_ERROR_STACK("Tried to retrieve future value before it's ready.");
  }
  if (future_retreived_)
  {
    throw CREATE_TEMOTO_ERROR_STACK("Future value is already retreived.");
  }
  future_retreived_ = true;
  return action_future_->get();
}

bool ActionHandle::clearFuture()
{
  LOCK_GUARD_TYPE_R guard_action_future(action_future_rw_mutex_);
  LOCK_GUARD_TYPE guard_action_state(state_rw_mutex_);
  if (state_ != State::RUNNING)
  {
    action_future_.reset();
    return true;
  }
  return false;
}

const std::string& ActionHandle::getEffect() const
{
  LOCK_GUARD_TYPE_R guard_umrf(umrf_rw_mutex_);
  return umrf_->getEffect();
}

const std::string& ActionHandle::getActionName() const
{
  LOCK_GUARD_TYPE_R guard_umrf(umrf_rw_mutex_);
  return umrf_->getFullName();
}

const unsigned int& ActionHandle::getHandleId() const
{
  LOCK_GUARD_TYPE_R guard_umrf(umrf_rw_mutex_);
  return umrf_->getId();
}

bool ActionHandle::addInputParameters(ActionParameters action_parameters)
{
  LOCK_GUARD_TYPE_R guard_umrf(umrf_rw_mutex_);
  bool ret_val = umrf_->copyInputParameters(action_parameters);

  if (umrf_->inputParametersReceived())
  {
    setState(ActionHandle::State::INITIALIZED);
  }
  return ret_val;
}

void ActionHandle::updateUmrf(const Umrf& umrf_in)
{
  LOCK_GUARD_TYPE_R guard_umrf(umrf_rw_mutex_);
  try
  {
    // Don't update the action if it is in error or stop request state
    if ((getState() == State::ERROR) ||
        (getState() == State::STOP_REQUESTED))
    {
      throw CREATE_TEMOTO_ERROR_STACK("Cannot update action '" + umrf_->getFullName() + "' because it is in ERROR or STOP state.");
    }

    // If any parameter was updated and action is in RUNNING or FINISHED state, then let the action now about the update
    if (umrf_->updateInputParams(umrf_in) && (getState() == State::RUNNING || getState() == State::FINISHED))
    {
      action_instance_->onParameterUpdate();
    }
  }
  catch(TemotoErrorStack e)
  {
    throw FORWARD_TEMOTO_ERROR_STACK(e);
  }
}

ActionHandle::~ActionHandle()
{
  clearAction();
}