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

#ifndef TEMOTO_ACTION_ENGINE__ACTION_ENGINE_H
#define TEMOTO_ACTION_ENGINE__ACTION_ENGINE_H

#include "temoto_action_engine/action_executor.h"
#include "temoto_action_engine/action_indexer.h"
#include "temoto_action_engine/action_match_finder.h"
#include "temoto_action_engine/temoto_error.h"

/**
 * @brief Handles loading and execution of TeMoto Actions
 * 
 */
class ActionEngine
{
public:
  ActionEngine();

  void start();

  void executeUmrfGraph(const std::string& umrf_graph_name, const std::vector<Umrf>& umrf_vec, bool name_match_required = false);

  void stopUmrfGraph(const std::string& umrf_graph_name);
  
  void addActionsPath(const std::string& action_packages_path);

  ~ActionEngine();
private:
  ActionExecutor ae_;
  ActionIndexer ai_;
  ActionMatchFinder amf_;
};
#endif
