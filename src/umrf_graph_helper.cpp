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

#include "temoto_action_engine/umrf_graph_helper.h"
#include <iostream>

GraphNode::GraphNode(const Umrf& umrf)
: umrf_(umrf)
, state_(GraphNode::State::UNINITIALIZED)
{}

GraphNode::GraphNode(const GraphNode& gn)
: umrf_(gn.umrf_)
, state_(gn.state_)
{}

UmrfGraphHelper::UmrfGraphHelper(const std::string& graph_name, const std::vector<Umrf>& umrfs_vec)
: state_(State::UNINITIALIZED)
, graph_name_(graph_name)
, umrfs_vec_(umrfs_vec)
{
  if (!createMaps())
  {
    std::cout << "probs with map\n";
    return;
  }
  if (!findRootNodes())
  {
    std::cout << "probs with root nodes\n";
    return;
  }

  /*
   * TODO: Check that parents are pointing to existing children and vice versa
   */ 

  LOCK_GUARD_TYPE guard_state(state_rw_mutex_);
  state_ = UmrfGraphHelper::State::INITIALIZED;
}

UmrfGraphHelper::UmrfGraphHelper(const UmrfGraphHelper& ugh)
: graph_nodes_map_(ugh.graph_nodes_map_)
, name_id_map_(ugh.name_id_map_)
, root_node_ids_(ugh.root_node_ids_)
, state_(ugh.state_)
, graph_name_(ugh.graph_name_)
, umrfs_vec_(ugh.umrfs_vec_)
{}

bool UmrfGraphHelper::createMaps()
{
  LOCK_GUARD_TYPE_R guard_graph_nodes_map_(graph_nodes_map_rw_mutex_);
  LOCK_GUARD_TYPE_R guard_name_id_map_(name_id_map_rw_mutex_);

  for (const auto& umrf : umrfs_vec_)
  {
    try
    {
      bool success = graph_nodes_map_.emplace(umrf.getId(), umrf).second &&
        name_id_map_.emplace(umrf.getFullName(), umrf.getId()).second;
      if (!success)
      {
        return false;
      }
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
      return false;
    }
  }
  return true;
}

bool UmrfGraphHelper::findRootNodes()
{
  LOCK_GUARD_TYPE_R guard_graph_nodes_map_(graph_nodes_map_rw_mutex_);
  LOCK_GUARD_TYPE guard_root_node_ids_(root_node_ids_rw_mutex_);
  
  try
  {
    for (const auto& graph_node_pair : graph_nodes_map_)
    {
      // A node is considered root if it does not have parents
      if (graph_node_pair.second.umrf_.getParents().empty())
      {
        root_node_ids_.push_back(graph_node_pair.first);
      }
    }
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
    return false;
  }
  
  if (root_node_ids_.empty())
  {
    return false;
  }
  else
  {
    return true;
  }
}

const std::vector<unsigned int>& UmrfGraphHelper::getRootNodes() const
{
  LOCK_GUARD_TYPE guard_root_node_ids_(root_node_ids_rw_mutex_);
  return root_node_ids_;
}

bool UmrfGraphHelper::partOfGraph(const unsigned int& node_id) const
{
  LOCK_GUARD_TYPE_R guard_graph_nodes_map_(graph_nodes_map_rw_mutex_);
  return (graph_nodes_map_.find(node_id) != graph_nodes_map_.end());
}

std::vector<unsigned int> UmrfGraphHelper::getChildrenOf(const unsigned int& node_id) const
{
  LOCK_GUARD_TYPE_R guard_graph_nodes_map_(graph_nodes_map_rw_mutex_);
  LOCK_GUARD_TYPE_R guard_name_id_map_(name_id_map_rw_mutex_);

  std::vector<unsigned int> child_node_ids;
  if (partOfGraph(node_id))
  {
    for (const auto& child_node_full_name : graph_nodes_map_.at(node_id).umrf_.getChildren())
    {
      child_node_ids.push_back(name_id_map_.at(child_node_full_name));
    }
  }
  return std::move(child_node_ids);
}

bool UmrfGraphHelper::setNodeState(const unsigned int& node_id, GraphNode::State node_state)
{
  LOCK_GUARD_TYPE_R guard_graph_nodes_map_(graph_nodes_map_rw_mutex_);
  if (!partOfGraph(node_id))
  {
    return false;
  }
  else
  {
    graph_nodes_map_.at(node_id).state_ = node_state;
    return true;
  }
}

bool UmrfGraphHelper::setNodeActive(const unsigned int& node_id)
{
  return setNodeState(node_id, GraphNode::State::ACTIVE);
}

bool UmrfGraphHelper::setNodeFinished(const unsigned int& node_id)
{
  return setNodeState(node_id, GraphNode::State::FINISHED);
}

bool UmrfGraphHelper::setNodeError(const unsigned int& node_id)
{
  return setNodeState(node_id, GraphNode::State::ERROR);
}

UmrfGraphHelper::State UmrfGraphHelper::checkState()
{
  LOCK_GUARD_TYPE_R guard_graph_nodes_map_(graph_nodes_map_rw_mutex_);
  LOCK_GUARD_TYPE guard_state_(state_rw_mutex_);

  nr_of_uninitialized_nodes_ = 0;
  nr_of_initialized_nodes_ = 0;
  nr_of_active_nodes_ = 0;
  nr_of_finished_nodes_ = 0;
  nr_of_errored_nodes_ = 0;

  for (const auto& graph_node_pair : graph_nodes_map_)
  {
    switch(graph_node_pair.second.state_)
    {
      case GraphNode::State::UNINITIALIZED : nr_of_uninitialized_nodes_++; break;
      case GraphNode::State::INITIALIZED   : nr_of_initialized_nodes_++; break;
      case GraphNode::State::ACTIVE        : nr_of_active_nodes_++; break;
      case GraphNode::State::FINISHED      : nr_of_finished_nodes_++; break;
      case GraphNode::State::ERROR         : nr_of_errored_nodes_++; break;
    }
  }

  if (nr_of_errored_nodes_ > 0)
  {
    state_ = UmrfGraphHelper::State::ERROR;
  }
  else if (nr_of_active_nodes_ > 0)
  {
    state_ = UmrfGraphHelper::State::ACTIVE;
  }
  else if (nr_of_finished_nodes_ == graph_nodes_map_.size())
  {
    state_ = UmrfGraphHelper::State::FINISHED;
  }
  return state_;
}

const Umrf& UmrfGraphHelper::getUmrfOf(const unsigned int& node_id) const
{
  LOCK_GUARD_TYPE_R guard_graph_nodes_map_(graph_nodes_map_rw_mutex_);
  if (!partOfGraph(node_id))
  {
    std::cout << "Problems in getUmrf!" << std::endl;
  }
  return graph_nodes_map_.at(node_id).umrf_;
}

Umrf& UmrfGraphHelper::getUmrfOfNonconst(const unsigned int& node_id)
{
  // TODO: Use const cast
  LOCK_GUARD_TYPE_R guard_graph_nodes_map_(graph_nodes_map_rw_mutex_);
  if (!partOfGraph(node_id))
  {
    std::cout << "Problems in getUmrf!" << std::endl;
  }
  return graph_nodes_map_.at(node_id).umrf_;
}

const std::vector<Umrf>& UmrfGraphHelper::getUmrfs() const
{
  return umrfs_vec_;
}