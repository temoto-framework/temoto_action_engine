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

void UmrfGraphExec::startGraph(NotifyFinishedCb notify_graph_finished_cb, const std::string& result
, const ActionParameters& params)
{
  LOCK_GUARD_TYPE_R guard_graph_nodes(graph_nodes_map_rw_mutex_);
  notify_graph_finished_cb_ = notify_graph_finished_cb;

  // Tell each action the name of the graph they're part of. Needed for synchronization procedures
  for (const auto& node : graph_nodes_map_)
  {
    node.second->setGraphName(getName());
  }

  // Transfer the parameters
  ActionParameters transferable_params;
  std::shared_ptr<UmrfNodeExec> root_node = graph_nodes_map_.at("graph_entry");

  for (const auto& transf_param_name: root_node->getInputParameters().getTransferableParams(params))
  {
    transferable_params.setParameter(params.getParameter(transf_param_name));
  }
  root_node->updateInputParams(transferable_params);

  // Check if all required parameters are received
  if (!root_node->inputParametersReceived())
  {
    return;
  }

  setState(State::RUNNING);
  root_node->setOutputParameters(root_node->getInputParameters());
  startChildNodes("graph_entry", result);
}

void UmrfGraphExec::stopGraph()
{
  if (getState() == State::UNINITIALIZED)
  {
    return;
  }
  else
  {
    setState(State::STOPPING);
  }

  LOCK_GUARD_TYPE_R guard_graph_nodes(graph_nodes_map_rw_mutex_);

  for (auto& graph_node : graph_nodes_map_)
  {
    graph_node.second->stop();
  }
}

void UmrfGraphExec::clearGraph()
{
  if (getState() == State::UNINITIALIZED)
  {
    return;
  }
  stopGraph();

  LOCK_GUARD_TYPE_R guard_graph_nodes(graph_nodes_map_rw_mutex_);
  graph_nodes_map_.clear();
  setState(State::UNINITIALIZED);
}

void UmrfGraphExec::stopNode(const std::string& umrf_name)
{
  LOCK_GUARD_TYPE_R guard_graph_nodes(graph_nodes_map_rw_mutex_);
  graph_nodes_map_.at(umrf_name)->stop();
}

void UmrfGraphExec::startChildNodes(const std::string& parent_node_name, const std::string& result)
try
{
  /*
   * Pass the output parameters of the parent action to child actions
   */

  if (getState() == State::STOPPING)
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

  std::set<std::string> child_node_names_exec(child_node_names.begin(), child_node_names.end());

  /*
   * Update the child nodes and check which children are ready
   */
  for (const auto& child_node_name : child_node_names)
  {
    std::shared_ptr<UmrfNodeExec> child_node = graph_nodes_map_.at(child_node_name);

    // Mark that the parent has finished
    child_node->setParentReceived(parent_node->asRelation());

    // Check if the parent should be ignored or not
    const auto child_response = child_node->getParentRelation(parent_node_name)->getResponse(result);

    if (child_response == "ignore")
    {
      continue;
    }
    else if (child_response == "bypass")
    {
      /*TODO: carry the results over to the "grand children" */
      child_node->bypass();
      continue;
    }

    // Transfer parent's output paramas to the child
    ActionParameters transferable_params;
    const auto& parent_parameters = parent_node->getOutputParameters();

    for (const auto& transf_param_name: child_node->getInputParameters().getTransferableParams(parent_parameters))
    {
      transferable_params.setParameter(parent_parameters.getParameter(transf_param_name));
    }
    child_node->updateInputParams(transferable_params);

    // Check if child node is ready
    if (!child_node->inputParametersReceived() || !child_node->requiredParentsFinished())
    {
      // child_node_names_exec.erase(child_node_name);
      continue;
    }

    if (child_node_name == "graph_exit")
    {
      setState(State::FINISHED);
      notify_graph_finished_cb_(getName());
      return;
    }

    // Execute the child action
    if (child_response == "run")
    {
      child_node->run();
    }
    else if (child_response == "stop")
    {
      child_node->stop();
    }
    else if (child_response == "pause")
    {
      child_node->pause();
    }
  }

  return;
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