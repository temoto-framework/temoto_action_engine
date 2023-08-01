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

#ifndef TEMOTO_ACTION_ENGINE__ACTION_INDEXER_H
#define TEMOTO_ACTION_ENGINE__ACTION_INDEXER_H

#include <vector>
#include <mutex>
#include <utility>
#include "boost/filesystem.hpp"
#include "temoto_action_engine/umrf_node.h"
#include "temoto_action_engine/umrf_graph.h"

/**
 * @brief Responsible for finding action packages from the filesystem.
 * 
 */
class ActionIndexer
{
public:

  struct Summary
  {
    long unsigned int actions;
    long unsigned int graphs;
  };

  ActionIndexer();
  /**
   * @brief Adds a base path where the ActionIndexer should look for action packages.
   * 
   * @param path 
   */
  void addActionPath(const std::string& path);

  /**
   * @brief Adds a base path where the ActionIndexer should look for action packages.
   * 
   * @param path 
   */
  void addActionPath(const std::vector<std::string>& paths);

  unsigned int containsActions(const std::string& actions_path) const;

  /**
   * @brief Goes through all ActionIndexer::action_paths_ and looks for actions recursively (see ActionIndexer::indexActions).
   * Clears all previously found actions before indexing
   * 
   * @return unsigned int Number of found actions
   */
  Summary indexActions();

  /**
   * @brief Returns all UMRFs that were found during last indexing
   * 
   * @return const std::vector<UmrfNode>& 
   */
  const std::vector<UmrfNode>& getUmrfs() const;

  const std::vector<UmrfGraph>& getGraphs() const;

private:

  /**
   * @brief Finds actions on local filesystem.
   * 
   * @param action_to_find If not specified then all actions are indexed.
   * @param base_path Base path where the search is started.
   * @param search_depth Specifies the folder level depth for the search.
   */
  std::vector<UmrfNode> findActionFilesys( std::string action_to_find
  , boost::filesystem::directory_entry base_path
  , int search_depth) const;

  std::vector<UmrfGraph> findGraphs(boost::filesystem::directory_entry base_path, int search_depth) const;

  mutable MUTEX_TYPE graphs_mutex_;
  GUARDED_VARIABLE(std::vector<UmrfGraph> indexed_graphs_, graphs_mutex_);

  mutable MUTEX_TYPE action_sfs_mutex_;
  GUARDED_VARIABLE(std::vector<UmrfNode> indexed_umrfs_, action_sfs_mutex_);

  mutable MUTEX_TYPE action_paths_mutex_;
  GUARDED_VARIABLE(std::vector<std::string> action_paths_, action_paths_mutex_);

  const std::string umrf_file_extension_ = "umrf.json";
};

#endif