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
, notify_cv_(std::make_shared<std::condition_variable>())
, notify_cv_mutex_(std::make_shared<std::mutex>())
{}

UmrfGraphExec::UmrfGraphExec(const UmrfGraphCommon& ugc)
: UmrfGraphBase(ugc)
, notify_cv_(std::make_shared<std::condition_variable>())
, notify_cv_mutex_(std::make_shared<std::mutex>())
{}

UmrfGraphExec::UmrfGraphExec(const std::string& graph_name, const std::vector<UmrfNode>& umrf_nodes_vec)
: UmrfGraphBase(graph_name, umrf_nodes_vec)
, notify_cv_(std::make_shared<std::condition_variable>())
, notify_cv_mutex_(std::make_shared<std::mutex>())
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
    // Wait until a thread is finished
    std::unique_lock<std::mutex> notify_cv_lock(*notify_cv_mutex_);
    std::cout << __func__ << "waiting for notification" << std::endl; // TODO remove
    notify_cv_->wait(notify_cv_lock, [&]{return !finished_nodes_.empty();});
    std::cout << __func__ << "got a notification" << std::endl; // TODO remove

    LOCK_GUARD_TYPE_R guard_graph_nodes(graph_nodes_map_rw_mutex_);
    for (const auto& finished_node : finished_nodes_)
    {
      try
      {
        auto graph_node = graph_nodes_map_.at(finished_node);
        TEMOTO_PRINT(graph_node->getFullName() + " finished"); // TODO: remove
        graph_node->joinUmrfNodeExecThread();
        TEMOTO_PRINT(graph_node->getFullName() + " thread joined"); // TODO: remove

        // Print the error messages if any
        std::string error_messages = graph_node->getErrorMessages().getMessage();
        if (!error_messages.empty())
        {
          std::cout << graph_node->getErrorMessages().getMessage() << std::endl;
        }

        // Clear the umrf node if it's synchronous
        if (graph_node->getEffect() == "synchronous")
        {
          graph_node->clearNode();
        }
      }
      catch(TemotoErrorStack e)
      {
        std::cout << e.what() << '\n';
      }
      catch(const std::exception& e)
      {
        std::cerr << e.what() << '\n';
      }
      catch(...)
      {
        std::cerr << "[" << __func__ << "] caught an unhandled exception" << '\n';
      }
    }

    finished_nodes_.clear();
  }
}

void UmrfGraphExec::startNodes(const std::vector<std::string> umrf_node_names, bool all_ready_requrired)
{
  LOCK_GUARD_TYPE_R guard_graph_nodes(graph_nodes_map_rw_mutex_);
  std::set<std::string> action_rollback_list;
  std::set<std::string> umrf_node_names_exec(umrf_node_names.begin(), umrf_node_names.end());
  try
  {
    std::cout << "D0" << std::endl; // TODO remove
    /*
     * Check if the actions have received all required parameters and parent signals
     */
    for (const auto& umrf_node_name : umrf_node_names)
    {
      std::shared_ptr<UmrfNodeExec> umrf_node = graph_nodes_map_.at(umrf_node_name);
      if (umrf_node->getState() == UmrfNode::State::UNINITIALIZED)
      {
        if (!umrf_node->inputParametersReceived() || !umrf_node->requiredParentsFinished())
        {
          umrf_node_names_exec.erase(umrf_node_name);
        }
      }
      else if (umrf_node->getState() == UmrfNode::State::INSTANTIATED)
      {
        if (!umrf_node->getInctanceInputParametersReceived() || !umrf_node->requiredParentsFinished())
        {
          umrf_node_names_exec.erase(umrf_node_name);
        }
      }
    }

    std::cout << "D1" << std::endl; // TODO remove

    if (all_ready_requrired && umrf_node_names_exec.size() != umrf_node_names.size())
    {
      throw CREATE_TEMOTO_ERROR_STACK("Cannot execute the actions because all actions were not fully initialized.");
    }

    /*
     * Load/Instantiate the actions. If there are any problems with instantiating
     * the graph, then the whole graph is rolled back (uninitialized)
     */
    for (const auto& umrf_node_name : umrf_node_names_exec)
    {
      std::cout << "D2" << std::endl; // TODO remove
      // Instantiate the action
      try
      {
        if (graph_nodes_map_.at(umrf_node_name)->getState() == UmrfNode::State::UNINITIALIZED)
        {
          std::cout << "D3" << std::endl; // TODO remove
          graph_nodes_map_.at(umrf_node_name)->instantiate(notify_cv_
          , notify_cv_mutex_
          , std::bind(&UmrfGraphExec::notifyFinished, this, std::placeholders::_1, std::placeholders::_2));
          action_rollback_list.insert(umrf_node_name);
        }
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

    std::cout << "D4" << std::endl; // TODO remove
    /*
     * Execute the actions. If there are any problems with executing
     * the graph, then the whole graph is rolled back
     */
    for (const auto& umrf_node_name : umrf_node_names_exec)
    {
      // Execute the action
      try
      {
        std::cout << "D5" << std::endl; // TODO remove
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
  std::cout << "D6" << std::endl; // TODO remove
}

void UmrfGraphExec::notifyFinished(const std::string& parent_node_name, const ActionParameters& parent_action_parameters)
{
  // TODO
  std::unique_lock<std::mutex> notify_cv_lock(*notify_cv_mutex_);
  finished_nodes_.push_back(parent_node_name);
  notify_cv_lock.unlock();

  /*
   * Pass the output parameters of the parent action to child actions
   */
  try
  {
    LOCK_GUARD_TYPE_R guard_graph_nodes(graph_nodes_map_rw_mutex_);
    std::shared_ptr<UmrfNodeExec> parent_node = graph_nodes_map_.at(parent_node_name);
    std::vector<std::string> child_node_names;

    if (parent_node->getChildren().empty())
    {
      return;
    }
    else
    {
      for (const auto& child_node_relation : parent_node->getChildren())
      {
        child_node_names.push_back(child_node_relation.getFullName());
      }
    }
    
    // Notify the childred that the parent has finished
    for (const auto& child_node_name : child_node_names)
    {
      graph_nodes_map_.at(child_node_name)->setParentReceived(parent_node->asRelation());
    }

    // If the parent has output parameters then pass them to the children
    if (!parent_action_parameters.empty())
    {
      for (const auto& child_node_name : child_node_names)
      {
        std::shared_ptr<UmrfNodeExec> child_node = graph_nodes_map_.at(child_node_name);
        std::set<std::string> transferable_param_names = child_node->getInputParameters().getTransferableParams(parent_action_parameters);

        if (transferable_param_names.empty())
        {
          continue;
        }
        /*
         * Parameters are directly passed to the child action instance in the loaded shared library,
         * because only the child action instance knows how to exactly copy the parameters without
         * depending on the virtual deleter that resides in the parent action. Hence first the child
         * action is instantiated and then the parameters are passed over.
         */
        if (child_node->getState() == UmrfNode::State::UNINITIALIZED)
        {
          child_node->instantiate(notify_cv_
          , notify_cv_mutex_
          , std::bind(&UmrfGraphExec::notifyFinished, this, std::placeholders::_1, std::placeholders::_2));
        }

        ActionParameters transferable_params;
        for (const auto& transf_param_name : transferable_param_names)
        {
          transferable_params.setParameter(parent_action_parameters.getParameter(transf_param_name));
        }

        child_node->updateInstanceParams(transferable_params);
      }
    }

    /*
     * Run the children actions
     */
    startNodes(child_node_names, false);
  }
  catch(TemotoErrorStack e)
  {
    throw FORWARD_TEMOTO_ERROR_STACK(e);
  }
  catch(const std::exception& e)
  {
    throw CREATE_TEMOTO_ERROR_STACK(std::string(e.what()));
  }
  
  return;
}