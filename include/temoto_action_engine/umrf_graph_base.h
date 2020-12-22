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
#include "temoto_action_engine/umrf.h"
#include "compiler_macros.h"
#include "temoto_action_engine/umrf_graph_node.h"

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
  
  UmrfGraphBase(const std::string& graph_name);

  UmrfGraphBase(const std::string& graph_name, const std::vector<Umrf>& umrfs_vec);

  UmrfGraphBase(const UmrfGraphBase& ugh);

  bool initialize(const std::vector<Umrf>& umrfs_vec);

  const std::string getName() const;

  const std::string getDescription() const;

  void setDescription(const std::string description);

  std::vector<unsigned int> getChildrenOf(const unsigned int& node_id) const;

  const Umrf& getUmrfOf(const unsigned int& node_id) const;

  Umrf& getUmrfOfNonconst(const unsigned int& node_id);

  std::vector<Umrf> getUmrfs() const;

  bool partOfGraph(const std::string& node_name) const;

  State checkState();

  /*
   * Methods for modifying the graph
   */

  void addUmrf(const Umrf& umrf);

  void addChild(const Umrf& umrf);

  unsigned int removeUmrf(const Umrf& umrf);

  void removeChild(const Umrf& umrf);

private:

  bool findRootNodes();

  /**
   * @brief Populates the graph_nodes_map_ and name_id_map_
   * 
   * @return true 
   * @return false 
   */
  virtual bool createMaps(const std::vector<Umrf>& umrfs_vec);

  mutable MUTEX_TYPE root_node_names_rw_mutex_;
  GUARDED_VARIABLE(std::vector<std::string> root_node_names_, root_node_names_rw_mutex_);

  mutable MUTEX_TYPE state_rw_mutex_;
  GUARDED_VARIABLE(State state_, state_rw_mutex_);

  std::string graph_name_;
  std::string graph_description_;

  mutable unsigned int nr_of_uninitialized_nodes_ = 0;
  mutable unsigned int nr_of_initialized_nodes_ = 0;
  mutable unsigned int nr_of_active_nodes_ = 0;
  mutable unsigned int nr_of_finished_nodes_ = 0;
  mutable unsigned int nr_of_errored_nodes_ = 0;
};
#endif