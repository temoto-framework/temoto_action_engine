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

#include "temoto_action_engine/umrf_graph_exec.h"

// UmrfGraphExec::UmrfGraphExec(const UmrfGraphExec& ug)
// : UmrfGraphBase(ug)
// , monitoring_thread_(ug.monitoring_thread_)
// {}

UmrfGraphExec::UmrfGraphExec(const std::string& graph_name)
: UmrfGraphBase(graph_name)
{}

UmrfGraphExec::UmrfGraphExec(const UmrfGraphCommon& ugc)
: UmrfGraphBase(ugc)
{}

UmrfGraphExec::UmrfGraphExec(const std::string& graph_name, const std::vector<UmrfNode>& umrf_nodes_vec)
: UmrfGraphBase(graph_name, umrf_nodes_vec)
{}

UmrfGraphExec::~UmrfGraphExec()
{
  stopGraph();
  clearGraph();
}

void UmrfGraphExec::startGraph()
{
  LOCK_GUARD_TYPE_R guard_graph_nodes(graph_nodes_map_rw_mutex_);

  // First start the monitoring loop in a separate thread
  monitoring_thread_ = std::thread(&UmrfGraphExec::monitoringLoop, this);

  // Start the root nodes
  startNodes(getRootNodeNames(), true);
}

void UmrfGraphExec::stopGraph()
{
  LOCK_GUARD_TYPE_R guard_graph_nodes(graph_nodes_map_rw_mutex_);
  for (auto& graph_node : graph_nodes_map_)
  {
    graph_node.second->stopNode(10);
  }
}

void UmrfGraphExec::clearGraph()
{
  LOCK_GUARD_TYPE_R guard_graph_nodes(graph_nodes_map_rw_mutex_);
  stopGraph();

  // Stop the monitoring thread
  stop_requested_ = true;
  monitoring_thread_.join();

  // Clear the nodes
  for (auto& graph_node : graph_nodes_map_)
  {
    graph_node.second->clearNode();
  }
}

void UmrfGraphExec::stopNode(const std::string& umrf_name)
{
  LOCK_GUARD_TYPE_R guard_graph_nodes(graph_nodes_map_rw_mutex_);
  graph_nodes_map_.at(umrf_name)->stopNode(10);
}

void UmrfGraphExec::monitoringLoop()
{
  while(!stop_requested_)
  {
    {
    LOCK_GUARD_TYPE_R guard_graph_nodes(graph_nodes_map_rw_mutex_);
    if (!graph_nodes_map_.empty())
    {
      for (auto& graph_node : graph_nodes_map_)
      {
        try
        {
          if (graph_node.second->futureReceived() && !graph_node.second->futureRetreived())
          {
            std::string error_message = graph_node.second->getFutureValue().getMessage();
            if (!error_message.empty())
            {
              std::cout << error_message << std::endl;
            }
          }
          if (graph_node.second->getState() == UmrfNode::State::FINISHED &&
              graph_node.second->getEffect() == "synchronous")
          {
            graph_node.second->clearNode();
          }
        }
        catch(TemotoErrorStack e)
        {
          std::cout << e.what() << '\n';
        }
      }
    }
    } // Lock guard scope
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

void UmrfGraphExec::startNodes(const std::vector<std::string> umrf_node_names, bool initialized_requrired)
{
  LOCK_GUARD_TYPE_R guard_graph_nodes(graph_nodes_map_rw_mutex_);

  std::set<std::string> action_rollback_list;
  std::set<std::string> init_action_ids;
  try
  {
    /*
     * Load the action handles
     */ 
    UmrfGraphNodeMap named_action_handles_tmp;

    /*
     * Load/Instantiate the actions. If there are any problems with instantiating
     * the graph, then the whole graph is rolled back (uninitialized)
     */
    for (const auto& umrf_node_name : umrf_node_names)
    {
      // Instantiate the action
      try
      {
        graph_nodes_map_.at(umrf_node_name)->instantiate();
        action_rollback_list.insert(umrf_node_name);
      }
      catch(TemotoErrorStack e)
      {
        throw FORWARD_TEMOTO_ERROR_STACK(e);
      } 
      catch(const std::exception& e)
      {
        throw CREATE_TEMOTO_ERROR_STACK("Cannot initialize the actions because: " + std::string(e.what()));
      }
    }

    /*
     * Execute the actions. If there are any problems with executing
     * the graph, then the whole graph is rolled back
     */
    for (const auto& umrf_node_name : umrf_node_names)
    {
      // Execute the action
      try
      {
        graph_nodes_map_.at(umrf_node_name)->startNodeThread();
        action_rollback_list.insert(umrf_node_name);
      }
      catch(TemotoErrorStack e)
      {
        throw FORWARD_TEMOTO_ERROR_STACK(e);
      } 
      catch(const std::exception& e)
      {
        throw CREATE_TEMOTO_ERROR_STACK("Cannot execute the actions because: " + std::string(e.what()));
      }
    }
  }
  catch(TemotoErrorStack e)
  {
    std::cout << "Rollbacking actions" << std::endl;
    for (const auto& umrf_node_name : action_rollback_list)
    {
      graph_nodes_map_.at(umrf_node_name)->clearNode();
    }
    throw FORWARD_TEMOTO_ERROR_STACK(e);
  }
}