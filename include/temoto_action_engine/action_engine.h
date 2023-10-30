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

#ifndef TEMOTO_ACTION_ENGINE__ACTION_ENGINE_H
#define TEMOTO_ACTION_ENGINE__ACTION_ENGINE_H

#include "temoto_action_engine/action_engine_handle.h"
#include "temoto_action_engine/action_indexer.h"
#include "temoto_action_engine/action_match_finder.h"
#include "temoto_action_engine/action_synchronizer.h"
#include "temoto_action_engine/temoto_error.h"
#include "temoto_action_engine/umrf_graph.h"
#include "temoto_action_engine/umrf_graph_exec.h"
#include "temoto_action_engine/umrf_graph_diff.h"
#include <condition_variable>
#include <thread>

/**
 * @brief Handles loading and execution of TeMoto Actions
 * 
 */
class ActionEngine
{
public:
  ActionEngine(const std::string& actor_name, std::vector<std::string> sync_plugin_names = {});

  // TODO: this method is prolly deprecated and should be removed
  void start();

  void executeUmrfGraphA(UmrfGraph umrf_graph, const std::string& result = "on_true"
  , bool name_match_required = false);

  void executeUmrfGraph(const std::string& graph_name, const ActionParameters& params = ActionParameters()
  , const std::string& result = "on_true");

  /**
   * @brief Modifies a UMRF graph according to the graph_diffs
   * 
   * @param graph_name 
   * @param graph_diffs 
   */
  void modifyGraph(const std::string& graph_name, const UmrfGraphDiffs& graph_diffs);

  void stopUmrfGraph(const std::string& umrf_graph_name);
  
  bool addActionsPath(const std::string& action_packages_path);

  std::vector<std::string> getGraphJsons() const;

  std::string waitForGraph(const std::string& graph_name);

  ~ActionEngine();

  /**
   * @brief Stops all graphs
   * 
   * @return true 
   * @return false 
   */
  bool stop();

private:

  void addWaiter(const Waitable& waitable, const Waiter& waiter);

  bool graphExists(const std::string& graph_name) const;

  void addUmrfGraph(const std::string& graph_name, const std::vector<UmrfNode>& umrf_nodes);

  void monitoringLoop();

  void notifyFinished(const Waitable& waitable, const std::string& result, const ActionParameters& params);

  bool matchGraph(UmrfGraph& g, std::set<std::string> g_blacklist);

  bool synchronizerAvailable() const;

  // void updateUmrfGraph(const std::string& graph_name, std::vector<UmrfNode> umrfs_vec);
  ActionIndexer ai_;
  ActionMatchFinder amf_;
  std::unique_ptr<ActionSynchronizer> as_;

  typedef std::map<std::string, std::shared_ptr<UmrfGraphExec>> UmrfGraphExecMap;
  mutable MUTEX_TYPE_R umrf_graph_map_rw_mutex_;
  GUARDED_VARIABLE(UmrfGraphExecMap umrf_graph_exec_map_, umrf_graph_map_rw_mutex_);

  typedef std::map<Waitable, std::vector<Waiter>> SyncMap;
  mutable MUTEX_TYPE sync_map_rw_mutex_;
  GUARDED_VARIABLE(SyncMap sync_map_, sync_map_rw_mutex_);

  mutable MUTEX_TYPE notify_cv_mutex_;
  GUARDED_VARIABLE(std::condition_variable notify_cv_, notify_cv_mutex_);

  std::string actor_name_;

  std::map<std::string, std::string> finished_graphs_;

  std::thread monitoring_thread_;

  bool stop_monitoring_thread_;
};
#endif
