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
#include "temoto_action_engine/umrf_graph.h"
#include "temoto_action_engine/umrf_graph_exec.h"
#include "temoto_action_engine/util/error.hpp"

#include <condition_variable>
#include <thread>

/**
 * @brief Handles loading and execution of TeMoto Actions
 *
 */
class ActionEngine
{
public:
  ActionEngine(
    const std::string& actor_name,
    const unsigned int& indexing_rate = 0,
    const std::string& sync_plugin_name = "");

  void executeUmrfGraphA(UmrfGraph umrf_graph, const std::string& result = "on_true"
  , bool name_match_required = false);

  void executeUmrfGraph(const std::string& graph_name, const ActionParameters& params = ActionParameters()
  , const std::string& result = "on_true");

  void modifyGraph(const UmrfGraph& graph_new);

  void pauseUmrfGraph(const std::string& umrf_graph_name);

  void resumeUmrfGraph(const std::string& umrf_graph_name);

  void stopUmrfGraph(const std::string& umrf_graph_name);

  bool addActionsPath(const std::string& action_packages_path);

  std::vector<std::string> getGraphJsonsIndexed() const;

  std::vector<std::string> getGraphJsonsRunning() const;

  std::vector<std::string> getUmrfJsons() const;

  std::string waitForGraph(const std::string& graph_name);

  std::vector<std::string> readFeedbackBuffer();

  ~ActionEngine();

  /**
   * @brief Stops all graphs
   *
   * @return true
   * @return false
   */
  bool stop();

private:

  void acknowledge(const std::string& token);

  void addWaiter(const Waitable& waitable, const Waiter& waiter);

  bool graphExists(const std::string& graph_name) const;

  void addUmrfGraph(const std::string& graph_name, const std::vector<UmrfNode>& umrf_nodes);

  void notifyFinished(const Waitable& waitable, const std::string& result, const ActionParameters& params, const std::string& token = "");

  bool matchGraph(UmrfGraph& g, std::set<std::string> g_blacklist);

  bool synchronizerAvailable() const;

  void onStateChange(const std::string& action_name, const std::string& graph_name);

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

};
#endif
