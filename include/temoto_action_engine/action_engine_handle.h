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

#include <functional>
#include "action_parameters.h"

typedef std::function<void(const std::string&, const ActionParameters&, const std::string&)> ExecuteGraphT;

class EngineHandle
{
friend class ActionEngine;
public:
  void executeUmrfGraph(const std::string& graph_name
  , const ActionParameters& params
  , const std::string& result = "on_true")
  {
    execute_graph_fptr_(graph_name, params, result);
  }

private:
  ExecuteGraphT execute_graph_fptr_;
};

inline EngineHandle ENGINE_HANDLE;

#endif