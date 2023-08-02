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
{}

UmrfGraphExec::UmrfGraphExec(const UmrfGraphCommon& ugc)
: UmrfGraphBase(ugc)
{
  setState(UmrfGraphCommon::State::UNINITIALIZED);
  initialize();
}

UmrfGraphExec::UmrfGraphExec(const std::string& graph_name, const std::vector<UmrfNode>& umrf_nodes_vec)
: UmrfGraphBase(graph_name, umrf_nodes_vec)
{}

UmrfGraphExec::~UmrfGraphExec()
{
  stopGraph();
  clearGraph();
}

void UmrfGraphExec::startGraph(const std::string& result, const ActionParameters& params)
{
  LOCK_GUARD_TYPE_R guard_graph_nodes(graph_nodes_map_rw_mutex_);

  // Tell each action the name of the graph they're part of. Needed for synchronization procedures
  for (const auto& node : graph_nodes_map_)
  {
    node.second->setGraphName(getName());
  }

  std::shared_ptr<UmrfNodeExec> root_node = graph_nodes_map_.at(GRAPH_ENTRY.getFullName());

  if (!params.empty())
  {
    // Transfer the parameters
    ActionParameters transferable_params;
    for (const auto& transf_param_name: root_node->getInputParameters().getTransferableParams(params))
    {
      transferable_params.setParameter(params.getParameter(transf_param_name));
    }
    root_node->updateInputParams(transferable_params);
  }

  // Check if all required parameters are received
  if (!root_node->inputParametersReceived())
  {
    return;
  }

  setState(State::RUNNING);
  root_node->setOutputParameters(root_node->getInputParameters());
  startChildNodes(GRAPH_ENTRY, result);
}

std::string UmrfGraphExec::stopGraph()
{
  if (getState() == State::UNINITIALIZED ||
      getState() == State::STOPPING ||  
      getState() == State::STOPPED ||
      getState() == State::FINISHED ||
      getState() == State::ERROR)
  {
    return (getState() == State::ERROR ? "on_error" : "on_true");
  }

  setState(State::STOPPING);
  LOCK_GUARD_TYPE_R guard_graph_nodes(graph_nodes_map_rw_mutex_);

  for (auto& graph_node : graph_nodes_map_)
  {
    graph_node.second->stop(true);
  }

  bool had_error = false;
  for (auto& graph_node : graph_nodes_map_)
  {
    while (graph_node.second->getState() != UmrfNode::State::FINISHED &&
      graph_node.second->getState() != UmrfNode::State::ERROR)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (graph_node.second->getState() == UmrfNode::State::ERROR)
    {
      had_error = true;
    }
  }

  if (had_error)
  {
    setState(State::ERROR);
    return "on_error";
  }
  else
  {
    setState(State::STOPPED);
    return "on_true";
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

void UmrfGraphExec::startChildNodes(const UmrfNode::Relation& parent_node_relation, const std::string& result)
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
  std::shared_ptr<UmrfNodeExec> parent_node = graph_nodes_map_.at(parent_node_relation.getFullName());

  std::cout << "D1\n";

  /*
   * Update the child nodes and check which children are ready
   */
  for (const auto& child_node_relation : parent_node->getChildren())
  {
    std::cout << "D1_1a\n";
    std::shared_ptr<UmrfNodeExec> child_node = graph_nodes_map_.at(child_node_relation.getFullName());
    std::cout << "D1_1b\n";
    // Mark that the parent has finished
    child_node->setParentReceived(parent_node_relation);
    std::cout << "D1_1c\n";
    // Check if the parent should be ignored or not
    const auto child_response = child_node->getParentRelation(parent_node_relation)->getResponse(result);

    std::cout << "D1_2\n";

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

    std::cout << "D1_3\n";

    // Transfer parent's output paramas to the child
    ActionParameters transferable_params;
    const auto& parent_parameters = parent_node->getOutputParameters();
    
    std::cout << "parent_param size: " << parent_parameters.getParameterCount() << std::endl;
    for (const auto& p : parent_parameters)
    {
      std::cout << " * name: " << p.getName() << std::endl;
      std::cout << " * type: " << p.getType() << std::endl;
      std::cout << " * data: " << p.getDataSize() << std::endl;
    }

    for (const auto& transf_param_name: child_node->getInputParameters().getTransferableParams(parent_parameters))
    {
      transferable_params.setParameter(parent_parameters.getParameter(transf_param_name));
    }

    std::cout << "transferable_params size: " << transferable_params.getParameterCount() << std::endl; 

    child_node->updateInputParams(transferable_params);

    // Check if child node is ready
    if (!child_node->inputParametersReceived() || !child_node->requiredParentsFinished())
    {
      std::cout << "D1_3a\n";
      continue;
    }

    std::cout << "D1_4\n";

    if (child_node->getName() == GRAPH_EXIT.getName())
    {
      setState(State::FINISHED);
      ENGINE_HANDLE.notifyFinished(Waitable{.action_name = GRAPH_EXIT.getFullName(), .graph_name = getName()}, result);
      return;
    }

    std::cout << "D1_5\n";

    child_node->start_child_nodes_cb_ = std::bind(&UmrfGraphExec::startChildNodes, this
    , std::placeholders::_1, std::placeholders::_2);

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