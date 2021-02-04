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