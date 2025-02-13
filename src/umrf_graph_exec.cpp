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

  std::shared_ptr<UmrfNodeExec> graph_entry = graph_nodes_map_.at(GRAPH_ENTRY.getFullName());

  if (!params.empty())
  {
    // Transfer the parameters
    ActionParameters transferable_params;
    for (const auto& transf_param_name: graph_entry->getInputParameters().getTransferableParams(params))
    {
      transferable_params.setParameter(params.getParameter(transf_param_name));
    }
    graph_entry->updateInputParams(transferable_params);
  }

  // Check if all required parameters are received
  if (!graph_entry->inputParametersReceived())
  {
    return;
  }

  setState(State::RUNNING);
  graph_entry->setOutputParameters(graph_entry->getInputParameters());
  startChildNodes(GRAPH_ENTRY, result);

  /*
   * In order to avoid interdependencies of input and output parameters
   * between different hierarchical graphs, remove the input and output data of
   * the 'graph_entry' action. This is again related to the quirks of the 'std::any'.
   * The reason why the data is removed here is to make sure that the children of
   * 'graph_entry' got the data.
   */
  graph_entry->getInputParametersNc().clearData();
  graph_entry->getOutputParametersNc().clearData();
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

  // Stop all nodes
  for (auto& graph_node : graph_nodes_map_)
  {
    graph_node.second->stop(true);
  }

  // Make sure to remove the parameters from the graph exit first. Entry has already been
  // cleared after the graph was started
  graph_nodes_map_.at(GRAPH_EXIT.getFullName())->getInputParametersNc().clearData();
  graph_nodes_map_.at(GRAPH_EXIT.getFullName())->getOutputParametersNc().clearData();

  // Wait until all nodes finish
  bool had_error = false;
  for (auto& graph_node : graph_nodes_map_)
  {
    while (graph_node.second->getState() != UmrfNode::State::FINISHED &&
      graph_node.second->getState() != UmrfNode::State::ERROR &&
      graph_node.second->getState() != UmrfNode::State::UNINITIALIZED)
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

std::set<std::string> UmrfGraphExec::getLinkedActions(const UmrfNode::Relation& parent) const
{
  LOCK_GUARD_TYPE_R guard_graph_nodes(graph_nodes_map_rw_mutex_);
  std::set<std::string> stoplist;

  for (const auto& child : graph_nodes_map_.at(parent.getFullName())->getChildren())
  {
    if (stoplist.find(child.getFullName()) != stoplist.end())
      continue;

    stoplist.insert(child.getFullName());
    auto sub_stoplist = getLinkedActions(child);
    stoplist.insert(sub_stoplist.begin(), sub_stoplist.end());
  }

  return stoplist;
}

void UmrfGraphExec::startChildNodes(const UmrfNode::Relation& parent_node_relation, const std::string& result)
try
{
  /*
   * Pass the output parameters of the parent action to child actions
   */
  if (getState() != State::RUNNING)
  {
    return;
  }

  /*
   * If the parent action's type is 'spontaneous', then stop all the actions
   * that come after the children
   */
  const std::string& parent_node_type = [&]
  {
    LOCK_GUARD_TYPE_R guard_graph_nodes(graph_nodes_map_rw_mutex_);
    return graph_nodes_map_.at(parent_node_relation.getFullName())->getType();
  }();

  if (parent_node_type == "spontaneous")
  {
    for(const auto& n : getLinkedActions(parent_node_relation))
    {
      graph_nodes_map_.at(n)->stop();
    }

    for(const auto& n : getLinkedActions(parent_node_relation))
    {
      while (true)
      {
        {
          LOCK_GUARD_TYPE_R guard_graph_nodes(graph_nodes_map_rw_mutex_);
          if (graph_nodes_map_.at(n)->getState() == UmrfNode::State::FINISHED ||
              graph_nodes_map_.at(n)->getState() == UmrfNode::State::ERROR ||
              graph_nodes_map_.at(n)->getState() == UmrfNode::State::UNINITIALIZED)
          {
            break;
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
  }

  LOCK_GUARD_TYPE_R guard_graph_nodes(graph_nodes_map_rw_mutex_);
  std::shared_ptr<UmrfNodeExec> parent_node = graph_nodes_map_.at(parent_node_relation.getFullName());

  /*
   * Update the child nodes and check which children are ready
   */
  for (const auto& child_node_relation : parent_node->getChildren())
  {
    std::shared_ptr<UmrfNodeExec> child_node = graph_nodes_map_.at(child_node_relation.getFullName());

    // Mark that the parent has finished
    child_node->setParentReceived(parent_node_relation);

    // Assign this method as a callback to the child action
    if (child_node->start_child_nodes_cb_ == NULL)
    {
      child_node->start_child_nodes_cb_ = std::bind(&UmrfGraphExec::startChildNodes, this
      , std::placeholders::_1, std::placeholders::_2);
    }

    // Check if the parent should be ignored or not
    const auto child_response = child_node->getParentRelation(parent_node_relation)->getResponse(result);
    // std::stringstream ss;
    // ss << ": result(" << parent_node->getFullName() << "):" << result << ", response(" << child_node->getFullName() << "): " << child_response << std::endl;
    // TEMOTO_PRINT_OF(ss.str(), ENGINE_HANDLE.getActor() + " in " + getName());

    if (child_response == "bypass" && child_node->getName() == GRAPH_EXIT.getName())
    {
      setState(State::FINISHED);
      ENGINE_HANDLE.notifyFinished(Waitable{.action_name = GRAPH_EXIT.getFullName(), .graph_name = getName()}
      , result
      , child_node->getInputParameters());



      return;
    }
    else if (child_response == "bypass" && child_node->getName() != GRAPH_EXIT.getName())
    {
      child_node->bypass(result);
      continue;
    }
    else if (child_response == "ignore")
    {
      continue;
    }

    // Transfer parent's output paramas to the child
    ActionParameters transferable_params;
    ActionParameters parent_parameters_remapped = parent_node->getOutputParameters();

    // Remap the parameter names
    const auto& cr = parent_node->getChildren();
    for (const auto& rel : std::find(cr.begin(), cr.end(), child_node->asRelation())->getParameterRemappings())
    {
      parent_parameters_remapped.getParameterNc(rel.first).setName(rel.second);
    }

    for (const auto& transf_param_name: child_node->getInputParameters().getTransferableParams(parent_parameters_remapped))
    {
      transferable_params.setParameter(parent_parameters_remapped.getParameter(transf_param_name));
    }

    child_node->updateInputParams(transferable_params);

    // Check if child node is ready
    if (!child_node->inputParametersReceived() || !child_node->requiredParentsFinished())
    {
      continue;
    }

    if (child_node->getName() == GRAPH_EXIT.getName())
    {
      setState(State::FINISHED);
      ENGINE_HANDLE.notifyFinished(Waitable{.action_name = GRAPH_EXIT.getFullName(), .graph_name = getName()}
      , result
      , child_node->getInputParameters());



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
