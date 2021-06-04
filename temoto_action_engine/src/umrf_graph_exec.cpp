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

void UmrfGraphExec::startGraph(NotifyFinishedCb notify_graph_finished_cb)
{
  LOCK_GUARD_TYPE_R guard_graph_nodes(graph_nodes_map_rw_mutex_);
  notify_graph_finished_cb_ = notify_graph_finished_cb;

  setState(State::ACTIVE);

  // First start the monitoring loop in a separate thread
  monitoring_thread_ = std::thread(&UmrfGraphExec::monitoringLoop, this);

  // Start the root nodes
  startNodes(getRootNodeNames(), true);
}

void UmrfGraphExec::stopGraph()
{
  if (getState() == State::UNINITIALIZED)
  {
    return;
  }
  else
  {
    setState(State::STOP_REQUESTED);
  }

  LOCK_GUARD_TYPE_R guard_graph_nodes(graph_nodes_map_rw_mutex_);

  for (auto& graph_node : graph_nodes_map_)
  {
    graph_node.second->stopNode(10);
  }
}

void UmrfGraphExec::clearGraph()
{
  if (getState() == State::UNINITIALIZED)
  {
    return;
  }
  stopGraph();

  // Stop the monitoring thread
  notify_cv_->notify_all();
  monitoring_thread_.join();

  LOCK_GUARD_TYPE_R guard_graph_nodes(graph_nodes_map_rw_mutex_);
  graph_nodes_map_.clear();
  setState(State::UNINITIALIZED);
}

void UmrfGraphExec::stopNode(const std::string& umrf_name)
{
  LOCK_GUARD_TYPE_R guard_graph_nodes(graph_nodes_map_rw_mutex_);
  graph_nodes_map_.at(umrf_name)->stopNode(10);
}

void UmrfGraphExec::monitoringLoop()
{
  monitoring_thread_running_ = true;
  while(getState() != State::STOP_REQUESTED)
  {
    // Wait until a thread is finished
    std::unique_lock<std::mutex> notify_cv_lock(*notify_cv_mutex_);
    notify_cv_->wait(notify_cv_lock
    , [&]{return !finished_nodes_.empty() || getState() == State::STOP_REQUESTED;});

    // If the graph is requested to stop then go throug all actions and join all running threads
    if (getState() == State::STOP_REQUESTED)
    {
      notify_cv_lock.unlock();
      bool all_threads_finished = true;
      do
      {
        {
          LOCK_GUARD_TYPE_R guard_graph_nodes(graph_nodes_map_rw_mutex_);
          all_threads_finished = true;
          for (auto& graph_node : graph_nodes_map_)
          {
            if (graph_node.second->threadRunning())
            {
              all_threads_finished = false;
            }
            else if (graph_node.second->threadJoinable())
            {
              graph_node.second->joinUmrfNodeExecThread();
            }
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }
      while (!all_threads_finished);
      break;
    }

    LOCK_GUARD_TYPE_R guard_graph_nodes(graph_nodes_map_rw_mutex_);
    for (const auto& finished_node : finished_nodes_)
    {
      try
      {
        auto graph_node = graph_nodes_map_.at(finished_node);
        graph_node->joinUmrfNodeExecThread();

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

    // If all actions in the graph are synchronous and have finished, then stop the graph
    bool all_actions_finished = true;
    for (const auto& graph_node : graph_nodes_map_)
    {
      if (graph_node.second->getEffect() != "synchronous" || 
          graph_node.second->getState() != UmrfNode::State::UNINITIALIZED)
      {
        all_actions_finished = false;
        break;
      }
    }

    if (all_actions_finished)
    {
      notify_graph_finished_cb_(graph_name_);
      break;
    }
  }
  monitoring_thread_running_ = false;
}

void UmrfGraphExec::startNodes(const std::vector<std::string> umrf_node_names, bool all_ready_requrired)
{
  LOCK_GUARD_TYPE_R guard_graph_nodes(graph_nodes_map_rw_mutex_);

  if (getState() == State::STOP_REQUESTED)
  {
    return;
  }

  std::set<std::string> action_rollback_list;
  std::set<std::string> umrf_node_names_exec(umrf_node_names.begin(), umrf_node_names.end());
  try
  {
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
      // Instantiate the action
      try
      {
        if (graph_nodes_map_.at(umrf_node_name)->getState() == UmrfNode::State::UNINITIALIZED)
        {
          action_rollback_list.insert(umrf_node_name);
          graph_nodes_map_.at(umrf_node_name)->instantiate(
            std::bind(&UmrfGraphExec::notifyFinished, this, std::placeholders::_1)
          , std::bind(&UmrfGraphExec::startChildNodes, this, std::placeholders::_1, std::placeholders::_2));
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

    /*
     * Execute the actions. If there are any problems with executing
     * the graph, then the whole graph is rolled back
     */
    for (const auto& umrf_node_name : umrf_node_names_exec)
    {
      // Execute the action
      try
      {
        action_rollback_list.insert(umrf_node_name);
        graph_nodes_map_.at(umrf_node_name)->startNodeThread();
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

void UmrfGraphExec::startChildNodes(const std::string& parent_node_name, const ActionParameters& parent_action_parameters)
{
  /*
   * Pass the output parameters of the parent action to child actions
   */
  try
  {
    if (getState() == State::STOP_REQUESTED)
    {
      return;
    }

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
    
    // Notify the children that the parent has finished
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
          child_node->instantiate(
            std::bind(&UmrfGraphExec::notifyFinished, this, std::placeholders::_1)
          , std::bind(&UmrfGraphExec::startChildNodes, this, std::placeholders::_1, std::placeholders::_2));
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
  catch(...)
  {
    throw CREATE_TEMOTO_ERROR_STACK("Caught an unhandled exception");
  }
  return;
}

void UmrfGraphExec::notifyFinished(const std::string& node_name)
{
  {
    std::lock_guard<std::mutex> notify_cv_lock(*notify_cv_mutex_);
    finished_nodes_.push_back(node_name);
  }
  notify_cv_->notify_all();
}