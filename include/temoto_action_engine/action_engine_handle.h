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

#ifndef TEMOTO_ACTION_ENGINE__ACTION_ENGINE_HANDLE_H
#define TEMOTO_ACTION_ENGINE__ACTION_ENGINE_HANDLE_H

#include "temoto_action_engine/action_parameters.h"
#include "temoto_action_engine/util/mutex.hpp"
#include "temoto_action_engine/util/error.hpp"
#include "temoto_action_engine/util/waitlist.hpp"

#include <functional>
#include <map>
#include <memory>
#include <vector>

class EngineHandle
{
friend class ActionEngine;
using ExecuteGraphT = std::function<void(const std::string&, const ActionParameters&, const std::string&)>;
using AcknowledgeT  = std::function<void(const std::string&)>;
using StateChangeT  = std::function<void(const std::string&, const std::string&)>;

public:

  void acknowledge(const std::string& token)
  {
    acknowledge_fptr_(token);
  }

  void addWaiter(const Waitable& waitable, const Waiter& waiter)
  {
    add_waiter_fptr_(waitable, waiter);
  }

  void executeUmrfGraph(const std::string& graph_name
  , const ActionParameters& params
  , const std::string& result = "on_true")
  {
    execute_graph_fptr_(graph_name, params, result);
  }

  void notifyFinished(const Waitable& waitable, const std::string& result, const ActionParameters& params, const std::string& token = "")
  {
    notify_finished_fptr_(waitable, result, params, token);
  }

  void notifyStateChange(const std::string& action_name, const std::string& graph_name)
  {
    on_state_change_fptr(action_name, graph_name);
  }

  std::string getActor() const
  {
    return actor_name_;
  }

private:
  AcknowledgeT acknowledge_fptr_;
  AddWaiterT add_waiter_fptr_;
  ExecuteGraphT execute_graph_fptr_;
  NotifyFinishedT notify_finished_fptr_;
  StateChangeT on_state_change_fptr;
  std::string actor_name_;
};

inline EngineHandle ENGINE_HANDLE;

#endif
