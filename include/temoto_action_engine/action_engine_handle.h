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

#include "action_parameters.h"
#include "temoto_action_engine/temoto_error.h"
#include "temoto_action_engine/mutex.h"

#include <functional>
#include <map>
#include <memory>
#include <vector>

struct WaitlistItem
{
  std::string action_name;
  std::string graph_name;

  bool operator==(const WaitlistItem& other) const
  {
    return (action_name == other.action_name) && (graph_name == other.graph_name);
  }

  bool operator <(const WaitlistItem& other) const
  {
    return (action_name + graph_name) < (other.action_name + other.graph_name);
  }
};

typedef WaitlistItem Waiter;
typedef WaitlistItem Waitable;

class EngineHandle
{
friend class ActionEngine;
typedef std::function<void(const Waitable&, const Waiter&)> AddWaiterT;
typedef std::function<void(const std::string&, const ActionParameters&, const std::string&)> ExecuteGraphT;
typedef std::function<void(const Waitable&, const std::string&)> NotifyFinishedT;

public:

  void acknowledge(const Waitable& waitable, const Waiter& waiter)
  {
    // TODO
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

  void notifyFinished(const Waitable& waitable, const std::string& result)
  {
    notify_finished_fptr_(waitable, result);
  }

private:
  AddWaiterT add_waiter_fptr_;
  ExecuteGraphT execute_graph_fptr_;
  NotifyFinishedT notify_finished_fptr_;
};

inline EngineHandle ENGINE_HANDLE;

#endif