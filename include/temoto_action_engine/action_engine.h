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

#ifndef TEMOTO_ACTION_ENGINE__ACTION_ENGINE_H
#define TEMOTO_ACTION_ENGINE__ACTION_ENGINE_H

#include "temoto_action_engine/action_indexer.h"
#include "temoto_action_engine/action_match_finder.h"
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
  ActionEngine();

  // TODO: this method is prolly deprecated and should be removed
  void start();

  void executeUmrfGraph(UmrfGraph umrf_graph, bool name_match_required = false);

  /**
   * @brief Modifies a UMRF graph according to the graph_diffs
   * 
   * @param graph_name 
   * @param graph_diffs 
   */
  void modifyGraph(const std::string& graph_name, const UmrfGraphDiffs& graph_diffs);

  void stopUmrfGraph(const std::string& umrf_graph_name);
  
  bool addActionsPath(const std::string& action_packages_path);

  void setActorSynchronizerUmrf(const UmrfNode& actor_synchronizer_umrf);

  std::vector<std::string> getGraphJsons() const;

  ~ActionEngine();

  /**
   * @brief Stops all graphs
   * 
   * @return true 
   * @return false 
   */
  bool stop();

private:
  bool graphExists(const std::string& graph_name) const;

  void addUmrfGraph(const std::string& graph_name, const std::vector<UmrfNode>& umrf_nodes);

  void executeUmrfGraph(const std::string& graph_name);

  void monitoringLoop();

  void notifyGraphFinished(const std::string& graph_name);

  // void updateUmrfGraph(const std::string& graph_name, std::vector<UmrfNode> umrfs_vec);
  ActionIndexer ai_;
  ActionMatchFinder amf_;

  typedef std::map<std::string, std::shared_ptr<UmrfGraphExec>> UmrfGraphExecMap;
  mutable MUTEX_TYPE_R umrf_graph_map_rw_mutex_;
  GUARDED_VARIABLE(UmrfGraphExecMap umrf_graph_exec_map_, umrf_graph_map_rw_mutex_);

  mutable MUTEX_TYPE notify_cv_mutex_;
  GUARDED_VARIABLE(std::condition_variable notify_cv_, notify_cv_mutex_);

  std::vector<std::string> finished_graphs_;

  std::thread monitoring_thread_;

  bool stop_monitoring_thread_;
};
#endif
