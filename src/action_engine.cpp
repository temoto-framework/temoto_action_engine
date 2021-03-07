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
#include "temoto_action_engine/umrf_json_converter.h"

ActionEngine::ActionEngine()
{}

void ActionEngine::start()
{
}

bool ActionEngine::graphExists(const std::string& graph_name) const
{
  LOCK_GUARD_TYPE_R guard_graph_nodes_map_(umrf_graph_map_rw_mutex_);
  return (umrf_graph_exec_map_.find(graph_name) != umrf_graph_exec_map_.end());
}

// void ActionEngine::addUmrfGraph(const UmrfGraph& umrf_graph)
// {
//   LOCK_GUARD_TYPE_R guard_graph_nodes_map_(umrf_graph_map_rw_mutex_);
//   umrf_graph_exec_map_.emplace(umrf_graph.getName(), UmrfGraphExec(umrf_graph));
// }

void ActionEngine::addUmrfGraph(const std::string& graph_name, const std::vector<UmrfNode>& umrf_nodes)
{
  LOCK_GUARD_TYPE_R guard_graph_nodes_map_(umrf_graph_map_rw_mutex_);
  umrf_graph_exec_map_.emplace(graph_name, std::make_shared<UmrfGraphExec>(graph_name, umrf_nodes));
}

void ActionEngine::executeUmrfGraph(UmrfGraph umrf_graph, bool name_match_required)
{
  std::vector<UmrfNode> umrf_nodes_local = umrf_graph.getUmrfNodes();

  // Find a matching action for this UMRF
  for (auto& umrf_node : umrf_nodes_local)
  {
    // Find a matching action for this UMRF
    bool result;
    try
    {
      result = amf_.findMatchingAction(umrf_node, ai_.getUmrfs(), name_match_required);
    }
    catch(TemotoErrorStack e)
    {
      throw FORWARD_TEMOTO_ERROR_STACK(e);
    }
    if (!result)
    {
      throw CREATE_TEMOTO_ERROR_STACK("Could not find a matching action for UMRF named " + umrf_node.getName());
    }
  }
  TEMOTO_PRINT("All actions in graph '" + umrf_graph.getName() + "' found.");

  /*
   * If the graph already exists, then try to update it. Otherwise create and execute a new graph
   */
  if (graphExists(umrf_graph.getName()))
  {
    TEMOTO_PRINT("UMRF graph '" + umrf_graph.getName() + "' is already running.");
  }
  else
  {
    addUmrfGraph(umrf_graph.getName(), umrf_nodes_local);
    TEMOTO_PRINT("UMRF graph '" + umrf_graph.getName() + "' initialized.");

    executeUmrfGraph(umrf_graph.getName());
    TEMOTO_PRINT("UMRF graph '" + umrf_graph.getName() + "' invoked successfully.");
  }
}

void ActionEngine::executeUmrfGraph(const std::string& graph_name)
{
  LOCK_GUARD_TYPE_R guard_graph_nodes_map_(umrf_graph_map_rw_mutex_);
  try
  {
    // Check if the requested graph exists
    if (!graphExists(graph_name))
    {
      throw CREATE_TEMOTO_ERROR_STACK("Cannot execute UMRF graph '" + graph_name + "' because it doesn't exist.");
    }

    // Check if the graph is in initialized state
    if (umrf_graph_exec_map_.at(graph_name)->getState() != UmrfGraphExec::State::INITIALIZED)
    {
      throw CREATE_TEMOTO_ERROR_STACK("Cannot execute UMRF graph '" + graph_name + "' because it's not in initialized state.");
    }
    else
    {
      umrf_graph_exec_map_.at(graph_name)->startGraph();
    }
  }
  catch(TemotoErrorStack e)
  {
    stopUmrfGraph(graph_name);
    throw FORWARD_TEMOTO_ERROR_STACK(e);
  }
}

void ActionEngine::modifyGraph(const std::string& graph_name, const UmrfGraphDiffs& graph_diffs)
{
  LOCK_GUARD_TYPE_R guard_graph_nodes_map_(umrf_graph_map_rw_mutex_);

  if (!graphExists(graph_name))
  {
    TEMOTO_PRINT("Cannot modify graph '" + graph_name + "' because it does not exist.");
    return;
  }

  TEMOTO_PRINT("Received a request to modify UMRF graph '" + graph_name + "' ...");
  std::shared_ptr<UmrfGraphExec> ugh = umrf_graph_exec_map_.at(graph_name);

  /*
   * Before modyfing the graph, check if the graph contains the UMRFs which are menitoned in the diffs
   */ 
  for (const auto& graph_diff : graph_diffs)
  {
    if (graph_diff.operation == UmrfGraphDiff::Operation::add_umrf)
    {
      if (ugh->partOfGraph(graph_diff.umrf_node.getFullName()))
      {
        throw CREATE_TEMOTO_ERROR_STACK("Cannot add UMRF '" + graph_diff.umrf_node.getFullName()
          + "', as it is already part of graph '" + graph_name + "'");
      }
    }
    else
    {
      if (!ugh->partOfGraph(graph_diff.umrf_node.getFullName()))
      {
        throw CREATE_TEMOTO_ERROR_STACK("Cannot perform operation '" + graph_diff.operation
        + "' because UMRF graph '" + graph_name + "' does not contain node named '"
        + graph_diff.umrf_node.getFullName() + "'");
      }
    }
  }

  /*
   * Apply the diffs
   */ 
  for (const auto& graph_diff : graph_diffs)
  {
    if (graph_diff.operation == UmrfGraphDiff::Operation::add_umrf)
    {
      TEMOTO_PRINT("Applying an '" + graph_diff.operation + "' operation to UMRF '" + graph_diff.umrf_node.getFullName() + "' ...");
      ugh->addUmrf(graph_diff.umrf_node);
    }
    else if (graph_diff.operation == UmrfGraphDiff::Operation::remove_umrf)
    {
      TEMOTO_PRINT("Applying an '" + graph_diff.operation + "' operation to UMRF '" + graph_diff.umrf_node.getFullName() + "' ...");
      ugh->stopNode(graph_diff.umrf_node.getFullName());
      ugh->removeUmrf(graph_diff.umrf_node);
    }
    else if (graph_diff.operation == UmrfGraphDiff::Operation::add_child)
    {
      TEMOTO_PRINT("Applying an '" + graph_diff.operation + "' operation to UMRF '" + graph_diff.umrf_node.getFullName() + "' ...");
      ugh->addChild(graph_diff.umrf_node);
    }
    else if (graph_diff.operation == UmrfGraphDiff::Operation::remove_child)
    {
      TEMOTO_PRINT("Applying an '" + graph_diff.operation + "' operation to UMRF '" + graph_diff.umrf_node.getFullName() + "' ...");
      ugh->removeChild(graph_diff.umrf_node);
    }
    else
    {
      throw CREATE_TEMOTO_ERROR_STACK("No such operation as " + graph_diff.operation);
    }
    TEMOTO_PRINT("Finished with the '" + graph_diff.operation + "' operation.");
  }
}

void ActionEngine::stopUmrfGraph(const std::string& umrf_graph_name)
{
  LOCK_GUARD_TYPE_R guard_graph_nodes_map_(umrf_graph_map_rw_mutex_);

  // Check if the requested graph exists
  if (umrf_graph_exec_map_.find(umrf_graph_name) == umrf_graph_exec_map_.end())
  {
    throw CREATE_TEMOTO_ERROR_STACK("Cannot stop UMRF graph '" + umrf_graph_name + "' because it doesn't exist.");
  }

  try
  {
    umrf_graph_exec_map_.at(umrf_graph_name)->stopGraph();
  }
  catch (TemotoErrorStack e)
  {
    throw FORWARD_TEMOTO_ERROR_STACK(e);
  }
  catch (std::exception& e)
  {
    throw CREATE_TEMOTO_ERROR_STACK(e.what());
  }
  umrf_graph_exec_map_.erase(umrf_graph_name);
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

bool ActionEngine::stop()
{
  LOCK_GUARD_TYPE_R guard_graph_nodes_map_(umrf_graph_map_rw_mutex_);

  // Stop all actions
  for (auto& umrf_graph : umrf_graph_exec_map_)
  {
    TEMOTO_PRINT("Stopping umrf graph " + umrf_graph.second->getName());
    umrf_graph.second->stopGraph();
  }

   // Clear all actions
  for (auto& umrf_graph : umrf_graph_exec_map_)
  {
    TEMOTO_PRINT("Clearing umrf graph " + umrf_graph.second->getName());
    umrf_graph.second->clearGraph();
  }

  TEMOTO_PRINT("Action Engine is stopped");
  return true;
}

std::vector<std::string> ActionEngine::getGraphJsons() const
{
  LOCK_GUARD_TYPE_R guard_graph_nodes_map_(umrf_graph_map_rw_mutex_);

  std::vector<std::string> umrf_graph_jsons;
  try
  {
    for (const auto& umrf_graph_exec : umrf_graph_exec_map_)
    {
      std::string umrf_graph_json = umrf_json_converter::toUmrfGraphJsonStr(umrf_graph_exec.second->toUmrgGraphCommon()); 
      umrf_graph_jsons.push_back(umrf_graph_json);     
    }
  }
  catch(TemotoErrorStack e)
  {
    throw FORWARD_TEMOTO_ERROR_STACK(e);
  }
  return umrf_graph_jsons;
}

ActionEngine::~ActionEngine()
{
  try
  {
    stop();
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
}
