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

#ifndef TEMOTO_ACTION_ENGINE__UMRF_GRAPH_EXEC_H
#define TEMOTO_ACTION_ENGINE__UMRF_GRAPH_EXEC_H

#include "temoto_action_engine/umrf_graph_base.h"
#include "temoto_action_engine/umrf_node_exec.h"
#include <thread>
#include <memory>
#include <mutex>
#include <condition_variable>

/**
 * @brief Implements the functions needed for executing a UMRF graph
 * 
 */
class UmrfGraphExec : public UmrfGraphBase<UmrfNodeExec>
{
typedef std::function<void(const std::string&)> NotifyFinishedCb;
public:

  UmrfGraphExec(const UmrfGraphExec& ug) = delete;

  UmrfGraphExec(const std::string& graph_name);

  UmrfGraphExec(const UmrfGraphCommon& ugc);

  UmrfGraphExec(const std::string& graph_name, const std::vector<UmrfNode>& umrf_nodes_vec);

  virtual ~UmrfGraphExec();

  void startGraph(NotifyFinishedCb notify_graph_finished_cb, const std::string& result
  , const ActionParameters& params);

  void stopGraph();

  void clearGraph();

  void stopNode(const std::string& umrf_name);

  void clearNode(const std::string& umrf_name);

  void startChildNodes(const std::string& parent_node_name, const std::string& result);

  void notifyFinished(const std::string& node_name);

private:

  void monitoringLoop();

  std::thread monitoring_thread_;

  bool monitoring_thread_running_ = false;

  std::shared_ptr<std::condition_variable> notify_cv_;

  std::shared_ptr<std::mutex> notify_cv_mutex_; 

  std::vector<std::string> finished_nodes_;

  NotifyFinishedCb notify_graph_finished_cb_ = NULL;
  
};

#endif