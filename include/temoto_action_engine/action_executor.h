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

#ifndef TEMOTO_ACTION_ENGINE__ACTION_EXECUTOR_H
#define TEMOTO_ACTION_ENGINE__ACTION_EXECUTOR_H

#include <iostream>
#include <future>
#include <vector>
#include <map>
#include <algorithm>
#include "temoto_action_engine/compiler_macros.h"
#include "temoto_action_engine/umrf.h"
#include "temoto_action_engine/umrf_graph_helper.h"
#include "temoto_action_engine/action_handle.h"

/**
 * @brief Handles loading and execution of TeMoto Actions
 * 
 */
class ActionExecutor
{
public:
  ActionExecutor();

  /**
   * @brief Starts the action executor
   * 
   */
  void start();
  
  /**
   * @brief Invoked after an action returns from the ActionHandle::executeAction method, notifying the ActionExecutor that this
   * particular action has finished execution. ActionExecutor then checks if the particular action is part of an UMRF
   * graph and executes children actions of according graphs.
   * 
   * @param int 
   * @param parent_action_parameters 
   */
  void notifyFinished(const unsigned int& parent_action_id, const ActionParameters& parent_action_parameters);

  /**
   * @brief Executes actions in a graph specified its' unique ID.
   * 
   * @param ids Identifiers of the actions to be executed
   * @param ugh Graph where the actions are part of
   * @param initialized_requrired If true then non of the actions are executed if some action is still in
   * uninitialized state (required input parameters not received). This is a required state when root nodes
   * of the action graph are executed.
   */
  void executeById(const std::vector<unsigned int> ids, UmrfGraphHelper& ugh, bool initialized_requrired = false);

  /**
   * @brief Returns the number of actions that are active in the ActionExecutor
   * 
   * @return unsigned int 
   */
  unsigned int getActionCount() const;

  /**
   * @brief Checks if any action is actively running 
   */
  bool isActive() const;

  /**
   * @brief Stops all actively running actions and the synchronous action cleanup loop
   * 
   * @return true 
   * @return false 
   */
  bool stopAndCleanUp();

  /**
   * @brief Checks if the UMRF graph with the given name already exists
   * 
   * @param graph_name 
   * @return true 
   * @return false 
   */
  bool graphExists(const std::string& graph_name);

  /**
   * @brief Creates and stores a UMRF graph object
   * 
   * @param graph_name 
   * @param umrfs_vec Umrf vector based on which the graph will be built
   */
  void addUmrfGraph(const std::string& graph_name, std::vector<Umrf> umrfs_vec);

  /**
   * @brief Updates the pvf_updatable parameters of UMRFs in existing graph
   * 
   * @param graph_name 
   * @param umrfs_vec 
   */
  void updateUmrfGraph(const std::string& graph_name, std::vector<Umrf> umrfs_vec);

  /**
   * @brief Executes UMRF graph based on graph name
   * 
   * @param graph_name 
   */
  void executeUmrfGraph(const std::string& graph_name);

  /**
   * @brief Stops all actions associated with this graph
   * 
   * @param graph_name 
   */
  void stopUmrfGraph(const std::string& graph_name);

private:
  /**
   * @brief Updates UMRFs of the associated action handles
   * 
   * @param umrf_vec 
   */
  void updateActionHandles(const UmrfGraphHelper& ugh, const std::vector<Umrf>& umrf_vec);

  /**
   * @brief Executes the cleanup loop which will remove all synchronous actions that have
   * finished executing.
   * 
   */
  void cleanupLoop();

  /**
   * @brief Starts the cleanup loop in its own thread
   * 
   * @return true Cleanup loop successfully started
   * @return false 
   */
  bool startCleanupLoopThread();

  unsigned int createId();

  std::string name_ = "Action Engine Name Test";
  std::future<void> cleanup_loop_future_;
  bool cleanup_loop_spinning_ = false;
  unsigned int action_handle_id_count_ = 0;

  /*
   * TODO: These typedefs are there because otherwise the GUARDED_VARIABLE
   * macro will treat the comma in the map<> definition as a variable separator.
   * There should be more approproate way for doing this
   */ 
  typedef std::map<unsigned int, ActionHandle> HandleMap;
  typedef std::map<std::string, UmrfGraphHelper> UmrfGraphMap;

  mutable MUTEX_TYPE_R named_action_handles_rw_mutex_;
  GUARDED_VARIABLE(HandleMap named_action_handles_, named_action_handles_rw_mutex_);

  mutable MUTEX_TYPE_R named_umrf_graphs_rw_mutex_;
  GUARDED_VARIABLE(UmrfGraphMap named_umrf_graphs_, named_umrf_graphs_rw_mutex_);
};

#endif
