/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright 2025 TeMoto Framework
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
#include <condition_variable>
#include <functional>

#include "temoto_action_engine/action_base.h"
#include "temoto_action_engine/action_engine_handle.h"
#include "temoto_action_engine/action_plugin.hpp"
#include "temoto_action_engine/umrf_node.h"
#include "temoto_action_engine/util/compiler_macros.hpp"
#include "temoto_action_engine/util/thread_wrapper.hpp"
#include "temoto_action_engine/util/error.hpp"

class UmrfNodeExec : public UmrfNode
{
friend class UmrfGraphExec;

struct CvWrapper
{
  void wait();
  void stopWaiting();

private:

  std::condition_variable cv_;
  std::mutex cv_mutex_;
  bool wait_{false};
};

using StartChildNodesCb = std::function<void(const UmrfNode::Relation&, const std::string&)>;

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

  void run();

  void pause();

  void resume();

  void stop(bool ignore_result = false);

  void bypass(const std::string& result);

  // void restart();

  /**
   * @brief Creates an instance of the underlying action
   *
   */
  void instantiate();

  /**
   * @brief Stops the action (UmrfNodeExec::stopNode) and destroys the action instance object.
   *
   */
  void clearNode();

  const TemotoErrorStack& getErrorMessages() const;

  std::string getLatestUmrfJsonStr() const;
  void setLatestUmrfJsonStr(const std::string& latest_umrf_json_str);

  void setGraphName(const std::string& parent_graph_name);

  void notifyFinished(const std::string& remote_notification_id = "");

  void setRemoteResult(const std::string& remote_result);

private:

  mutable MUTEX_TYPE_R action_plugin_rw_mutex_;
  GUARDED_VARIABLE(std::shared_ptr<ActionPlugin<ActionBase>> action_plugin_, action_plugin_rw_mutex_);

  mutable MUTEX_TYPE latest_umrf_json_str_rw_mutex_;
  GUARDED_VARIABLE(std::string latest_umrf_json_str_, latest_umrf_json_str_rw_mutex_);

  temoto::util::Threads<UmrfNode::State> state_threads_;

  std::string parent_graph_name_;

  float default_stopping_timeout_ = 5;

  TemotoErrorStack error_messages_;

  StartChildNodesCb start_child_nodes_cb_ = NULL;

  CvWrapper wait_for_result_;
  CvWrapper wait_for_resume_;
  CvWrapper wait_for_resume_local_;

  std::string remote_result_;
  std::string remote_notification_id_;

  std::string waitUntilFinished(const Waitable& waitable);

};

#endif
