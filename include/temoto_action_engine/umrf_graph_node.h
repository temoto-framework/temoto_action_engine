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

#ifndef TEMOTO_ACTION_ENGINE__UMRF_GRAPH_NODE_H
#define TEMOTO_ACTION_ENGINE__UMRF_GRAPH_NODE_H

#include <memory>
#include "temoto_action_engine/umrf.h"

class UmrfGraphNode
{
public:

  /// Defines all possible states for the action handle
  enum class State
  {
    UNINITIALIZED,      // Action library is not loaded or input parameters not received
    INITIALIZED,        // Action library is loaded and input parameters received
    READY,              // Action instance is loaded
    RUNNING,            // Action instance is running
    STOP_REQUESTED,     // A request to stop has been registered
    FINISHED,           // Action instance has finished execution
    ERROR,              // Problems with any critical component of the action
  };

  /// A convenience datastructure which helps to convert state names to std::string
  std::map<State, std::string> state_to_str_map_ = 
  {
    {State::UNINITIALIZED, "UNINITIALIZED"},
    {State::INITIALIZED, "INITIALIZED"},
    {State::READY, "READY"},
    {State::RUNNING, "RUNNING"},
    {State::STOP_REQUESTED, "STOP_REQUESTED"},
    {State::FINISHED, "FINISHED"},
    {State::ERROR, "ERROR"},
  };

  UmrfGraphNode(const Umrf& umrf);

  UmrfGraphNode(const UmrfGraphNode& ugn);

  virtual ~UmrfGraphNode();

  mutable MUTEX_TYPE state_rw_mutex_;
  GUARDED_VARIABLE(State state_, state_rw_mutex_);

  mutable MUTEX_TYPE_R umrf_rw_mutex_;
  GUARDED_VARIABLE(std::shared_ptr<Umrf> umrf_, umrf_rw_mutex_);

};

#endif