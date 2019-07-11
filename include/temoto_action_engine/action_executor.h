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
#include "temoto_action_engine/threadsafe_print.h"
#include "temoto_action_engine/action_handle.h"

// // Forward declare ActionHandle
// class ActionHandle;

/**
 * @brief Handles loading and execution of TeMoto Actions
 * 
 */
class ActionExecutor
{
public:
  ActionExecutor();
  
  /**
   * @brief Executes all children actions of the specified parent
   * 
   * @tparam T Payload type of the parameters
   * @param parent_action_name Requested
   * @param parent_action_parameters Requested
   */
  void notifyFinished(const unsigned int& parent_action_id, const ActionParameters& parent_action_parameters);

  void executeById(const std::vector<unsigned int> ids, UmrfGraphHelper& ugh, bool initialized_requrired = false);

  /**
   * @brief Returns the number of actions which are running in their own thread
   * 
   * @return unsigned int 
   */
  unsigned int getFutureCount() const;

  /**
   * @brief Returns the number of actions that are in the ActionExecutor
   * 
   * @return unsigned int 
   */
  unsigned int getActionCount() const;

  /**
   * @brief Checks if any action is actively running 
   */
  bool isActive() const;

  // /**
  //  * @brief Stops all actions and removes the action threads. It's a blocking call which waits
  //  * for all actions to finish.
  //  * TODO: Make the function non-blocking by adding a timeout functionality.
  //  */
  bool stopAndCleanUp();

  void addUmrfGraph(const std::string& graph_name, std::vector<Umrf> umrf_jsons_vec);

  void executeUmrfGraph(const std::string& graph_name);

  void stopUmrfGraph(const std::string& graph_name);

private:
  template<typename T>
  bool futureIsReady(const std::future<T>& t);

  /**
   * @brief Executes the cleanup loop which will remove all actions that have
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

/*
 * Include the template implementation file
 */ 
#include "temoto_action_engine/action_executor.tpp"
#endif
