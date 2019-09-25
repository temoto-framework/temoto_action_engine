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

void ActionEngine::executeUmrfGraph(const std::string& umrf_graph_name, const std::vector<Umrf>& umrf_vec, bool name_match_required)
{
  std::vector<Umrf> umrf_vec_local = umrf_vec;

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
  TEMOTO_PRINT("All actions in graph '" + umrf_graph_name + "' found.");

  /*
   * If the graph already exists, then try to update it. Otherwise create and execute a new graph
   */
  if (ae_.graphExists(umrf_graph_name))
  {
    TEMOTO_PRINT("UMRF graph '" + umrf_graph_name + "' is already running. Trying to update the graph ...");
    ae_.updateUmrfGraph(umrf_graph_name, umrf_vec_local);
    TEMOTO_PRINT("UMRF graph '" + umrf_graph_name + "' updated");
  }
  else
  {
    ae_.addUmrfGraph(umrf_graph_name, umrf_vec_local);
    TEMOTO_PRINT("UMRF graph '" + umrf_graph_name + "' initialized.");

    ae_.executeUmrfGraph(umrf_graph_name);
    TEMOTO_PRINT("UMRF graph '" + umrf_graph_name + "' invoked successfully.");
  }
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