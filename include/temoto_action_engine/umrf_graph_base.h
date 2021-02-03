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

/*
 * TODO: 
 *  - Graph visualization
 *  - Graph analysis (loop detection, potential error analysis[guaranteed vs required])
 */ 

#ifndef TEMOTO_ACTION_ENGINE__UMRF_GRAPH_BASE_H
#define TEMOTO_ACTION_ENGINE__UMRF_GRAPH_BASE_H

#include <string>
#include <vector>
#include <map>
#include <iostream>
#include "compiler_macros.h"
#include "temoto_action_engine/umrf_node.h"

template <class UMRF_NODE_T>
class UmrfGraphBase
{
public:
  enum class State: unsigned int
  {
    UNINITIALIZED,
    INITIALIZED,
    ACTIVE,
    FINISHED,
    ERROR
  };
  
  UmrfGraphBase(const std::string& graph_name)
  : state_(State::UNINITIALIZED)
  , graph_name_(graph_name)
  {}

  UmrfGraphBase(const std::string& graph_name, const std::vector<Umrf>& umrfs_vec)
  : state_(State::UNINITIALIZED)
  , graph_name_(graph_name)
  , umrf_nodes_vec_(umrfs_vec)
  {
    initialize(umrfs_vec);
  }

  UmrfGraphBase(const UmrfGraphBase& ugh);
  : graph_nodes_map_(ugh.graph_nodes_map_)
  , root_node_names_(ugh.root_node_ids_)
  , state_(ugh.state_)
  , graph_name_(ugh.graph_name_)
  , graph_description_(ugh.graph_name_)
  , umrf_nodes_vec_ (ugh.umrf_nodes_vec_)
  {}

  virtual ~UmrfGraphBase()
  {}

  const std::string& getName() const
  {
    return graph_name_;
  }

  void setDescription(const std::string& description)
  {
    graph_description_ = description;
  }

  const std::string& UmrfGraph::getDescription() const
  {
    return graph_description_;
  }

  void setState(UmrfGraphBase::State state)
  {
    // TODO: check the state
    state_ = state;
  }

  const State& getState() const
  {
    return state_;
  }

  std::vector<std::string> getChildrenOf(const std::string& node_name) const
  {
    LOCK_GUARD_TYPE_R guard_graph_nodes_map_(graph_nodes_map_rw_mutex_);

    std::vector<std::string> child_node_names;
    if (partOfGraph(node_name))
    {
      for (const auto& child_node_relation : graph_nodes_map_.at(node_name).getChildren())
      {
        child_node_names.push_back(child_node_relation.getFullName());
      }
    }
    return std::move(child_node_names);
  }

  const std::vector<UmrfNode>& getUmrfNodes() const
  {
    LOCK_GUARD_TYPE_R guard_graph_nodes_map_(graph_nodes_map_rw_mutex_);

    if (!graph_nodes_map_.empty())
    {
      umrf_nodes_vec_.clear();
      for (const auto& graph_node_it : graph_nodes_map_)
      {
        umrf_nodes_vec_.push_back(*(graph_node_it.second.asUmrfNode()));
      }
    }
    return umrf_nodes_vec_;
  }

  bool partOfGraph(const std::string& node_name) const
  {
    LOCK_GUARD_TYPE_R guard_graph_nodes_map_(graph_nodes_map_rw_mutex_);
    return (graph_nodes_map_.find(node_name) != graph_nodes_map_.end());
  }

  /*
   * Methods for modifying the graph
   */

  void addUmrf(const UmrfNode& umrf_node)
  {
    LOCK_GUARD_TYPE_R guard_graph_nodes_map_(graph_nodes_map_rw_mutex_);

    if (partOfGraph(umrf_node.getFullName()))
    {
      throw CREATE_TEMOTO_ERROR_STACK("Cannot add UMRF '" + umrf_node.getFullName() + "', as it is already part of graph '" 
        + graph_name_ + "'");
    }

    graph_nodes_map_.emplace(umrf_node.getFullName(), UMRF_T(umrf_node));

    // If the new UMRF has parents then modify the parent UMRFs accordingly
    for (const auto& parent_umrf_relation : umrf_node.getParents())
    {
      auto parent_node_itr = graph_nodes_map_.find(parent_umrf_relation.getFullName());
      parent_node_itr->second.addChild(umrf_node.asRelation());
    }

    // If the new UMRF has children then modify the child UMRFs accordingly
    for (const auto& child_umrf_relation : umrf_node.getChildren())
    {
      auto child_node_itr = graph_nodes_map_.find(child_umrf_relation.getFullName());
      child_node_itr->second.addParent(umrf_node.asRelation());
    }
    return;
  }

  void removeUmrf(const UmrfNode& umrf_node)
  {
    LOCK_GUARD_TYPE_R guard_graph_nodes_map_(graph_nodes_map_rw_mutex_);

    if (!partOfGraph(umrf_node.getFullName()))
    {
      throw CREATE_TEMOTO_ERROR_STACK("UMRF graph '" + graph_name_ + "' does not contain node named '" 
        + umrf_node.getFullName() + "'");
    }

    auto umrf_node_itr = graph_nodes_map_.find(umrf_node.getFullName());

    // Detach this umrf_node as a parent of any children
    for (const auto& child_umrf_relation : umrf_node_itr->second.getChildren())
    {
      auto child_node_itr = graph_nodes_map_.find(child_umrf_relation.getFullName());
      child_node_itr->second.removeParent(umrf_node_itr->second.asRelation());
    }

    // Detach this umrf_node as a child of any parents
    for (const auto& parent_umrf_relation : umrf_node_itr->second.getParents())
    {
      auto parent_node_itr = graph_nodes_map_.find(parent_umrf_relation.getFullName());
      parent_node_itr->second.removeChild(umrf_node_itr->second.asRelation());
    }

    //Remove the umrf_node node
    graph_nodes_map_.erase(umrf_node_itr);
  }

  void addChild(const UmrfNode& umrf_node)
  {
    LOCK_GUARD_TYPE_R guard_graph_nodes_map_(graph_nodes_map_rw_mutex_);

    if (!partOfGraph(umrf_node.getFullName()))
    {
      throw CREATE_TEMOTO_ERROR_STACK("UMRF graph '" + graph_name_ + "' does not contain node named '" 
        + umrf_node.getFullName() + "'");
    }

    auto umrf_node_itr = graph_nodes_map_.find(umrf_node.getFullName());

    for (const auto& child_umrf_relation : umrf_node.getChildren())
    {
      umrf_node_itr->second.addChild(child_umrf_relation);
      auto child_node_itr = graph_nodes_map_.find(child_umrf_relation.getFullName());
      child_node_itr->second.addParent(umrf_node.asRelation());
    }
  }

  void removeChild(const UmrfNode& umrf_node)
  {
    // TODO check if both locks are actually needed
    LOCK_GUARD_TYPE_R guard_graph_nodes_map_(graph_nodes_map_rw_mutex_);

    if (!partOfGraph(umrf_node.getFullName()))
    {
      throw CREATE_TEMOTO_ERROR_STACK("UMRF graph '" + graph_name_ + "' does not contain node named '" 
        + umrf_node.getFullName() + "'");
    }

    auto umrf_node_itr = graph_nodes_map_.find(umrf_node.getFullName());

    for (const auto& child_umrf_relation : umrf_node.getChildren())
    {
      umrf_node_itr->second.removeChild(child_umrf_relation);
      auto child_node_itr = graph_nodes_map_.find(child_umrf_relation.getFullName();
      child_node_itr->second.removeParent(umrf_node.asRelation());
    }
  }

private:

  bool initialize(const std::vector<Umrf>& umrfs_vec)
  {
    LOCK_GUARD_TYPE guard_state(state_rw_mutex_);
    if (state_ != UmrfGraph::State::UNINITIALIZED)
    {
      return true;
    }

    // Initialize the umrf_node nodes map
    for (const auto& umrf_node : umrfs_vec)
    {
      try
      {
        if (!graph_nodes_map_.emplace(umrf_node.getFullName(), UMRF_T(umrf_node)).second)
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

    if (!findRootNodes())
    {
      std::cout << "Could not find root nodes. UMRF graph must have at least one acyclic root node." << std::endl;
      return false;
    }

    /*
     * TODO: Check that parents are pointing to existing children and vice versa
     */

    state_ = UmrfGraph::State::INITIALIZED;
    return true;
  }

  bool findRootNodes()
  {
    LOCK_GUARD_TYPE_R guard_graph_nodes_map_(graph_nodes_map_rw_mutex_);
    try
    {
      for (const auto& graph_node_pair : graph_nodes_map_)
      {
        // A node is considered root if it does not have parents
        if (graph_node_pair.second.getParents().empty())
        {
          root_node_names_.push_back(graph_node_pair.first);
        }
      }
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
      return false;
    }
    
    return !root_node_names_.empty();
  }

  /// Helps to resolve UMRF name to a GraphNode
  typedef std::map<std::string, UMRF_NODE_T> UmrfGraphNodeMap;
  mutable MUTEX_TYPE_R graph_nodes_map_rw_mutex_;
  GUARDED_VARIABLE(UmrfGraphNodeMap graph_nodes_map_, graph_nodes_map_rw_mutex_);

  mutable MUTEX_TYPE root_node_names_rw_mutex_;
  GUARDED_VARIABLE(std::vector<std::string> root_node_names_, root_node_names_rw_mutex_);

  mutable MUTEX_TYPE state_rw_mutex_;
  GUARDED_VARIABLE(State state_, state_rw_mutex_);

  // This variable is part of a hacky bugfix, where umrfs (getUmrfs) returned by value got corrupted (currently returned 
  // via const&) (visible only in generated json strings, where umrf_node child/parent relation names were overwritten by 
  // random characters). The actual source of the problem is prolly related to rapidjson's memory allocation
  // but the bug is just too mysterious and I am a mortal human with deadlines 
  mutable std::vector<UmrfNode> umrf_nodes_vec_;

  std::string graph_name_;
  std::string graph_description_;
};
#endif


