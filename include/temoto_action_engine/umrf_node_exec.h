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
#include <condition_variable>
#include "temoto_action_engine/compiler_macros.h"
#include "temoto_action_engine/temoto_error.h"
#include "temoto_action_engine/umrf_node.h"
#include "temoto_action_engine/action_base.h"

typedef std::function<void(const std::string&, const ActionParameters&)> StartChildNodesCb;
typedef std::function<void(const std::string&)> NotifyFinishedCb;

class UmrfNodeExec : public UmrfNode
{
public:

  UmrfNodeExec(const UmrfNode& umrf_node);

  UmrfNodeExec(const UmrfNodeExec& unx) = delete;

  virtual ~UmrfNodeExec();

  virtual UmrfNode asUmrfNode() const;

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

  void joinUmrfNodeExecThread();

  void updateInstanceParams(const ActionParameters& ap_in);

  bool getInctanceInputParametersReceived() const;

  const TemotoErrorStack& getErrorMessages() const;

private:

  mutable MUTEX_TYPE class_loader_rw_mutex_;
  GUARDED_VARIABLE(std::shared_ptr<class_loader::ClassLoader> class_loader_, class_loader_rw_mutex_);

  mutable MUTEX_TYPE action_instance_rw_mutex_;
  GUARDED_VARIABLE(boost::shared_ptr<ActionBase> action_instance_, action_instance_rw_mutex_);

  std::thread umrf_node_exec_thread_;

  float default_stopping_timeout_ = 5;

  TemotoErrorStack error_messages_;

  NotifyFinishedCb notify_finished_cb_ = NULL;

  StartChildNodesCb start_child_nodes_cb_ = NULL;

};

#endif