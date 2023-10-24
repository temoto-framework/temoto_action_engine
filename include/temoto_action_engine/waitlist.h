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

#ifndef TEMOTO_ACTION_ENGINE__WAITLIST_H
#define TEMOTO_ACTION_ENGINE__WAITLIST_H

#include <functional>
#include <string>

class ActionParameters; // Forward declaration

struct WaitlistItem
{
  std::string action_name;
  std::string actor_name;
  std::string graph_name;

  bool operator==(const WaitlistItem& other) const
  {
    return (action_name == other.action_name) && 
           (graph_name == other.graph_name) &&
           (actor_name == other.actor_name);
  }

  bool operator <(const WaitlistItem& other) const
  {
    return (action_name + graph_name) < (other.action_name + other.graph_name);
  }
};

typedef WaitlistItem Waiter;
typedef WaitlistItem Waitable;

typedef std::function<void(const Waitable&, const Waiter&)> AddWaiterT;
typedef std::function<void(const Waitable&, const std::string&, const ActionParameters&)> NotifyFinishedT;

#endif