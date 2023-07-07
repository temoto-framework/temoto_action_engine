/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright 2020 TeMoto Telerobotics
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

#ifndef TEMOTO_ACTION_ENGINE__UMRF_NODE_EXEC_H
#define TEMOTO_ACTION_ENGINE__UMRF_NODE_EXEC_H

#include <memory>
#include <string>
#include <thread>
#include <class_loader/class_loader.hpp>
#include <boost/shared_ptr.hpp>
#include <functional>
#include "temoto_action_engine/compiler_macros.h"
#include "temoto_action_engine/temoto_error.h"
#include "temoto_action_engine/umrf_node.h"
#include "temoto_action_engine/action_base.h"

class UmrfNodeExec : public UmrfNode
{
struct ThreadWrapper
{
  std::shared_ptr<std::thread> thread;
  bool is_running;
  TemotoErrorStack error_messages;
};

typedef std::map<std::string, ThreadWrapper> ActionThreads;
typedef std::function<void(const std::string&, const ActionParameters&, const std::string&)> StartChildNodesCb;
typedef std::function<void(const std::string&)> NotifyFinishedCb;


public:

  UmrfNodeExec(const UmrfNode& umrf_node);

  UmrfNodeExec(const UmrfNodeExec& unx) = delete;

  virtual ~UmrfNodeExec();

  virtual UmrfNode asUmrfNode() const;

  /**
   * @brief Initializes the action
   * 
   */
  void initializeNode();

  /**
   * @brief Executes the action
   * 
   */
  void startNode();

  /**
   * @brief Executes the action in a new thread
   * 
   */
  void startNodeThread();

  /**
   * @brief Wraps the UmrfNodeExec::startNode() and notifies the graph via condition variable
   * when finished 
   * 
   */
  void umrfNodeExecThread();



  void run();

  void pause();

  void restart();

  void stop();



  /**
   * @brief Creates an instance of the underlying action
   * 
   */
  void instantiate(NotifyFinishedCb notify_finished_cb
  , StartChildNodesCb start_child_nodes_cb);

  /**
   * @brief Sets the stop request flag via BaseAction::stopRequested and waits for the specified timeout period for
   * the action to finish. If timeout is reached, then the action is set to ERROR state.
   * 
   * @param timeout period in seconds.
   */
  bool stopNode(float timeout);

  /**
   * @brief Stops the action (UmrfNodeExec::stopNode) and destroys the action instance object.
   * 
   */
  void clearNode();

  bool threadRunning() const;

  bool threadJoinable() const;

  void joinUmrfNodeExecThread();

  void updateInstanceParams(const ActionParameters& ap_in);

  bool getInstanceInputParametersReceived() const;

  const TemotoErrorStack& getErrorMessages() const;

  std::string getLatestUmrfJsonStr() const;
  void setLatestUmrfJsonStr(const std::string& latest_umrf_json_str);

private:

  mutable MUTEX_TYPE class_loader_rw_mutex_;
  GUARDED_VARIABLE(std::shared_ptr<class_loader::ClassLoader> class_loader_, class_loader_rw_mutex_);

  mutable MUTEX_TYPE action_instance_rw_mutex_;
  GUARDED_VARIABLE(boost::shared_ptr<ActionBase> action_instance_, action_instance_rw_mutex_);

  mutable MUTEX_TYPE action_threads_rw_mutex_;
  GUARDED_VARIABLE(ActionThreads action_threads_, action_threads_rw_mutex_);

  ThreadWrapper monitoring_thread_;

  // std::thread run_thread_;
  // bool run_thread_running_;

  // std::thread pause_thread_;
  // bool pause_thread_running_;

  // std::thread restart_thread_;
  // bool restart_thread_running_;

  // std::thread stop_thread_;
  // bool stop_thread_running_;

  std::thread umrf_node_exec_thread_;

  bool umrf_node_exec_thread_running_ = false; 

  float default_stopping_timeout_ = 5;

  TemotoErrorStack error_messages_;

  NotifyFinishedCb notify_finished_cb_ = NULL;

  StartChildNodesCb start_child_nodes_cb_ = NULL;
  
  mutable MUTEX_TYPE latest_umrf_json_str_rw_mutex_;
  GUARDED_VARIABLE(std::string latest_umrf_json_str_, latest_umrf_json_str_rw_mutex_);
};

#endif