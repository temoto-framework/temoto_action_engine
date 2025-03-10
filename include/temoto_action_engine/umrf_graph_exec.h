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
friend class ActionEngine;
public:

  UmrfGraphExec(const UmrfGraphExec& ug) = delete;

  UmrfGraphExec(const std::string& graph_name);

  UmrfGraphExec(const UmrfGraphCommon& ugc);

  UmrfGraphExec(const std::string& graph_name, const std::vector<UmrfNode>& umrf_nodes_vec);

  virtual ~UmrfGraphExec();

  void startGraph(const std::string& result, const ActionParameters& params);

  void pauseGraph();

  void resumeGraph();

  std::string stopGraph();

  void clearGraph();

  void stopNode(const std::string& umrf_name);

  void clearNode(const std::string& umrf_name);

  std::set<std::string> getLinkedActions(const UmrfNode::Relation& parent) const;

  void startChildNodes(const UmrfNode::Relation& parent_node_relation, const std::string& result);

};

#endif
