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

/*
 * TODO: 
 *  - Graph visualization
 *  - Graph analysis (loop detection, potential error analysis[guaranteed vs required])
 */ 

#ifndef TEMOTO_ACTION_ENGINE__UMRF_GRAPH_H
#define TEMOTO_ACTION_ENGINE__UMRF_GRAPH_H

#include <string>
#include <vector>
#include <map>
#include "temoto_action_engine/umrf.h"
#include "compiler_macros.h"

struct GraphNode
{
  enum class State: unsigned int
  {
    UNINITIALIZED,
    INITIALIZED,
    ACTIVE,
    FINISHED,
    ERROR
  };

  GraphNode(const Umrf& umrf);
  Umrf umrf_;
  State state_;

  GraphNode(const GraphNode& gn);
};

class UmrfGraph
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
  
  UmrfGraph(const std::string& graph_name);

  UmrfGraph(const std::string& graph_name, const std::vector<Umrf>& umrfs_vec, bool initialize_graph = true);

  UmrfGraph(const UmrfGraph& ugh);

  bool initialize();

  const std::string getName() const;

  const std::string getDescription() const;

  void setDescription(const std::string description);

  std::vector<unsigned int> getChildrenOf(const unsigned int& node_id) const;

  const Umrf& getUmrfOf(const unsigned int& node_id) const;

  Umrf& getUmrfOfNonconst(const unsigned int& node_id);

  const std::vector<unsigned int>& getRootNodes() const;

  bool setNodeActive(const unsigned int& node_id);

  bool setNodeFinished(const unsigned int& node_id);

  bool setNodeError(const unsigned int& node_id);

  const std::vector<Umrf>& getUmrfs() const;

  bool partOfGraph(const unsigned int& node_id) const;

  const unsigned int& getNodeId(const std::string& node_name) const;

  std::vector<unsigned int> getNodeIds() const;

  State checkState();

private:
  bool setNodeState(const unsigned int& node_id, GraphNode::State node_state);

  bool findRootNodes();

  /**
   * @brief Populates the graph_nodes_map_ and name_id_map_
   * 
   * @return true 
   * @return false 
   */
  bool createMaps();

  /// Helps to resolve UMRF id to a GraphNode
  typedef std::map<unsigned int, GraphNode> GraphNodeMap;
  mutable MUTEX_TYPE_R graph_nodes_map_rw_mutex_;
  GUARDED_VARIABLE(GraphNodeMap graph_nodes_map_, graph_nodes_map_rw_mutex_);

  /// Helps to resolve UMRF (GraphNode) name to its ID
  typedef std::map<std::string, unsigned int> NameToIdMap;
  mutable MUTEX_TYPE_R name_id_map_rw_mutex_;
  GUARDED_VARIABLE(NameToIdMap name_id_map_, name_id_map_rw_mutex_);

  mutable MUTEX_TYPE root_node_ids_rw_mutex_;
  GUARDED_VARIABLE(std::vector<unsigned int> root_node_ids_, root_node_ids_rw_mutex_);

  mutable MUTEX_TYPE state_rw_mutex_;
  GUARDED_VARIABLE(State state_, state_rw_mutex_);

  std::string graph_name_;
  std::string graph_description_;
  const std::vector<Umrf> umrfs_vec_;

  mutable unsigned int nr_of_uninitialized_nodes_ = 0;
  mutable unsigned int nr_of_initialized_nodes_ = 0;
  mutable unsigned int nr_of_active_nodes_ = 0;
  mutable unsigned int nr_of_finished_nodes_ = 0;
  mutable unsigned int nr_of_errored_nodes_ = 0;
};
#endif