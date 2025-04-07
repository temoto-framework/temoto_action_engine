/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright 2023 TeMoto Framework
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
#include "temoto_action_engine/util/compiler_macros.hpp"
#include "temoto_action_engine/umrf_node.h"

class UmrfGraphCommon
{
public:
  enum class State: unsigned int
  {
    UNINITIALIZED,
    INITIALIZED,
    RUNNING,
    PAUSED,
    STOPPING,
    STOPPED,
    FINISHED,
    ERROR,
    HALTED,
    CLEARING
  };

  const static inline std::map<State, std::string> state_to_str_map_{
    {State::UNINITIALIZED, "UNINITIALIZED"},
    {State::INITIALIZED, "INITIALIZED"},
    {State::RUNNING, "RUNNING"},
    {State::PAUSED, "PAUSED"},
    {State::STOPPING, "STOPPING"},
    {State::STOPPED, "STOPPED"},
    {State::FINISHED, "FINISHED"},
    {State::ERROR, "ERROR"},
    {State::HALTED, "HALTED"},
    {State::CLEARING, "CLEARING"}};

  const static inline std::map<std::string, State> str_to_state_map_{
    {"UNINITIALIZED", State::UNINITIALIZED},
    {"INITIALIZED", State::INITIALIZED},
    {"RUNNING", State::RUNNING},
    {"PAUSED", State::PAUSED},
    {"STOPPING", State::STOPPING},
    {"STOPPED", State::STOPPED},
    {"FINISHED", State::FINISHED},
    {"ERROR", State::ERROR},
    {"HALTED", State::HALTED},
    {"CLEARING", State::CLEARING}};

  UmrfGraphCommon(const std::string& graph_name)
  : graph_name_(graph_name)
  , state_(State::UNINITIALIZED)
  {}

  UmrfGraphCommon(const std::string& graph_name, const std::vector<UmrfNode>& umrf_nodes_vec)
  : graph_name_(graph_name)
  , state_(State::UNINITIALIZED)
  , umrf_nodes_vec_(umrf_nodes_vec)
  {}

  UmrfGraphCommon(const UmrfGraphCommon& ugc)
  : graph_name_(ugc.graph_name_)
  , state_(ugc.state_)
  , umrf_nodes_vec_(ugc.umrf_nodes_vec_)
  , graph_description_(ugc.graph_description_)
  {}

  void operator=(const UmrfGraphCommon& ug)
  {
    graph_name_ = ug.graph_name_;
    state_ = ug.state_;
    umrf_nodes_vec_ = ug.umrf_nodes_vec_;
    graph_description_ = ug.graph_description_;
  }

  void setName(const std::string& name)
  {
    LOCK_GUARD_TYPE_R guard_name(graph_name_rw_mutex_);
    graph_name_ = name;
  }

  const std::string& getName() const
  {
    LOCK_GUARD_TYPE_R guard_name(graph_name_rw_mutex_);
    return graph_name_;
  }

  void setDescription(const std::string& description)
  {
    LOCK_GUARD_TYPE_R guard_description(graph_description_rw_mutex_);
    graph_description_ = description;
  }

  const std::string& getDescription() const
  {
    LOCK_GUARD_TYPE_R guard_description(graph_description_rw_mutex_);
    return graph_description_;
  }

  void setState(State state)
  {
    LOCK_GUARD_TYPE_R guard_state(state_rw_mutex_);
    // TODO: check the state
    state_ = state;
  }

  const State& getState() const
  {
    LOCK_GUARD_TYPE_R guard_state(state_rw_mutex_);
    return state_;
  }

protected:

  mutable MUTEX_TYPE_R graph_name_rw_mutex_;
  GUARDED_VARIABLE(std::string graph_name_, graph_name_rw_mutex_);

  mutable MUTEX_TYPE_R state_rw_mutex_;
  GUARDED_VARIABLE(State state_, state_rw_mutex_);

  mutable MUTEX_TYPE_R umrf_nodes_vec_rw_mutex_;
  GUARDED_VARIABLE(mutable std::vector<UmrfNode> umrf_nodes_vec_, umrf_nodes_vec_rw_mutex_);

  mutable MUTEX_TYPE_R graph_description_rw_mutex_;
  GUARDED_VARIABLE(std::string graph_description_, graph_description_rw_mutex_);

};

/**
 * @brief
 *
 * @tparam UMRF_NODE_T Type of UMRF nodes maintained in this class
 */
template <class UMRF_NODE_T>
class UmrfGraphBase : public UmrfGraphCommon
{
public:
  UmrfGraphBase(const std::string& graph_name)
  : UmrfGraphCommon(graph_name)
  {
  }

  UmrfGraphBase(const UmrfGraphBase& ugb)
  : UmrfGraphCommon(ugb)
  , graph_nodes_map_(ugb.graph_nodes_map_)
  {
  }

  UmrfGraphBase(const UmrfGraphCommon& ugc)
  : UmrfGraphCommon(ugc)
  {
    initialize();
  }

  UmrfGraphBase(const std::string& graph_name, const std::vector<UmrfNode>& umrf_nodes_vec)
  : UmrfGraphCommon(graph_name, umrf_nodes_vec)
  {
    initialize();
  }

  void operator=(const UmrfGraphBase& ug)
  {
    UmrfGraphCommon::operator= (ug);
    graph_nodes_map_ = ug.graph_nodes_map_;
  }

  virtual ~UmrfGraphBase()
  {}

  UmrfGraphCommon toUmrfGraphCommon()
  {
    updateUmrfNodes();
    return UmrfGraphCommon(*this);
  }

  std::vector<std::string> getChildrenOf(const std::string& node_name) const
  {
    LOCK_GUARD_TYPE_R guard_graph_nodes_map_(graph_nodes_map_rw_mutex_);

    std::vector<std::string> child_node_names;
    if (partOfGraph(node_name))
    {
      for (const auto& child_node_relation : graph_nodes_map_.at(node_name)->getChildren())
      {
        child_node_names.push_back(child_node_relation.getFullName());
      }
    }
    return std::move(child_node_names);
  }

  const std::shared_ptr<UmrfNode> getUmrfNode(const std::string umrf_name) const
  {
    LOCK_GUARD_TYPE_R guard_graph_nodes_map_(graph_nodes_map_rw_mutex_);
    return graph_nodes_map_.at(umrf_name);
  }

  // TODO: https://github.com/temoto-telerobotics/temoto_action_engine/issues/1
  const std::vector<UmrfNode>& getUmrfNodes() const
  {
    updateUmrfNodes();
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
  void modifyGraph(const UmrfGraphBase& new_graph)
  {
    LOCK_GUARD_TYPE_R guard_graph_nodes_map_(graph_nodes_map_rw_mutex_);

    if (getState() != State::PAUSED && getState() != State::HALTED)
    {
      throw CREATE_TEMOTO_ERROR_STACK("Cannot modify graph '" + graph_name_
      + "' because it is not in PAUSED state. Current state: '"
      + state_to_str_map_.at(getState()) + "'");
    }

    /*
     * Find out if any actions are to be deleted.
     * If the action to-be-deleted is in an active state, then abort the graph modification process
     */
    std::vector<std::string> actions_to_be_deleted;
    for (const auto& action_current : getUmrfNodes())
    {
      if (new_graph.partOfGraph(action_current.getFullName()))
      {
        continue;
      }

      if (action_current.getState() == UmrfNode::State::PAUSED || action_current.getState() == UmrfNode::State::RUNNING)
      {
        throw CREATE_TEMOTO_ERROR_STACK("Cannot modify graph '" + graph_name_ + "': Action '"
          + action_current.getFullName() + "' cannot be deleted, because it is in an active state ("
          + UmrfNode::state_to_str_map_.at(action_current.getState()) + ")");
      }

      actions_to_be_deleted.push_back(action_current.getFullName());
    }

    /*
     * Apply the modifications by overwriting the UMRF's
     */

    // Delete actions
    for (const auto& action_to_be_deleted : actions_to_be_deleted)
    {
      graph_nodes_map_.erase(action_to_be_deleted);
    }

    // Update / Add actions
    for (const auto& action_new : new_graph.getUmrfNodes())
    {
      if (!partOfGraph(action_new.getFullName()))
      {
        graph_nodes_map_.emplace(action_new.getFullName(), std::make_shared<UMRF_NODE_T>(action_new));
      }
      else
      {
        auto action_current_itr = graph_nodes_map_.find(action_new.getFullName());
        action_current_itr->second->setParents(action_new.getParents());
        action_current_itr->second->setChildren(action_new.getChildren());
        action_current_itr->second->updateInputParams(action_new.getInputParameters());
      }
    }
  }

  std::set<std::string> getActors()
  {
    LOCK_GUARD_TYPE_R guard_graph_nodes_map_(graph_nodes_map_rw_mutex_);

    std::set<std::string> actors;
    for(const auto& node : graph_nodes_map_)
    {
      if (!node.second->getActor().empty())
      {
        actors.insert(node.second->getActor());
      }
    }

    return actors;
  }

protected:

  bool initialize()
  {
    // Initialize the umrf_node nodes map
    for (const auto& umrf_node : umrf_nodes_vec_)
    try
    {
      if (!graph_nodes_map_.emplace(umrf_node.getFullName(), std::make_shared<UMRF_NODE_T>(umrf_node)).second)
      {
        return false;
      }
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
      return false;
    }

    /*
     * TODO: Check that parents are pointing to existing children and vice versa
     */

    if (state_ == State::UNINITIALIZED) // Needed to preserve the state
    {
      setState(State::INITIALIZED);
    }

    return true;
  }

  void updateUmrfNodes() const
  {
    LOCK_GUARD_TYPE_R guard_graph_nodes_map_(graph_nodes_map_rw_mutex_);
    if (!graph_nodes_map_.empty())
    {
      umrf_nodes_vec_.clear();
      for (const auto& graph_node_it : graph_nodes_map_)
      {
        umrf_nodes_vec_.push_back(graph_node_it.second->asUmrfNode());
      }
    }

  }

  /// Helps to resolve UMRF name to a GraphNode
  typedef std::map<std::string, std::shared_ptr<UMRF_NODE_T>> UmrfGraphNodeMap;
  mutable MUTEX_TYPE_R graph_nodes_map_rw_mutex_;
  GUARDED_VARIABLE(UmrfGraphNodeMap graph_nodes_map_, graph_nodes_map_rw_mutex_);
};
#endif
