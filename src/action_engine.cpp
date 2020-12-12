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

#include "temoto_action_engine/action_engine.h"
#include "temoto_action_engine/messaging.h"

ActionEngine::ActionEngine()
{}

void ActionEngine::start()
{
  ae_.start();
}

void ActionEngine::executeUmrfGraph(UmrfGraph umrf_graph, bool name_match_required)
{
  std::vector<Umrf> umrf_vec_local = umrf_graph.getUmrfs();

  // Find a matching action for this UMRF
  for (auto& umrf : umrf_vec_local)
  {
    // Find a matching action for this UMRF
    bool result;
    try
    {
      result = amf_.findMatchingAction(umrf, ai_.getUmrfs(), name_match_required);
    }
    catch(TemotoErrorStack e)
    {
      throw FORWARD_TEMOTO_ERROR_STACK(e);
    }
    if (!result)
    {
      throw CREATE_TEMOTO_ERROR_STACK("Could not find a matching action for UMRF named " + umrf.getName());
    }
  }
  TEMOTO_PRINT("All actions in graph '" + umrf_graph.getName() + "' found.");

  /*
   * If the graph already exists, then try to update it. Otherwise create and execute a new graph
   */
  if (ae_.graphExists(umrf_graph.getName()))
  {
    TEMOTO_PRINT("UMRF graph '" + umrf_graph.getName() + "' is already running. Trying to update the graph ...");
    ae_.updateUmrfGraph(umrf_graph.getName(), umrf_vec_local);
    TEMOTO_PRINT("UMRF graph '" + umrf_graph.getName() + "' updated");
  }
  else
  {
    ae_.addUmrfGraph(umrf_graph.getName(), umrf_vec_local);
    TEMOTO_PRINT("UMRF graph '" + umrf_graph.getName() + "' initialized.");

    ae_.executeUmrfGraph(umrf_graph.getName());
    TEMOTO_PRINT("UMRF graph '" + umrf_graph.getName() + "' invoked successfully.");
  }
}

void ActionEngine::modifyGraph(const std::string& graph_name, const UmrfGraphDiffs& graph_diffs)
{
  ae_.modifyGraph(graph_name, graph_diffs);
}

void ActionEngine::stopUmrfGraph(const std::string& umrf_graph_name)
{
  ae_.stopUmrfGraph(umrf_graph_name);
}

void ActionEngine::addActionsPath(const std::string& action_packages_path)
{
  try
  {
    ai_.addActionPath(action_packages_path);
    ai_.indexActions();
  }
  catch(TemotoErrorStack e)
  {
    throw FORWARD_TEMOTO_ERROR_STACK(e);
  }
}

ActionEngine::~ActionEngine()
{
  try
  {
    ae_.stopAndCleanUp();
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
}
