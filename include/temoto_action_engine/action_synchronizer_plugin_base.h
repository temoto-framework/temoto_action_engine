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

#ifndef TEMOTO_ACTION_ENGINE__ACTION_SYNCHRONIZER_PLUGIN_BASE_H
#define TEMOTO_ACTION_ENGINE__ACTION_SYNCHRONIZER_PLUGIN_BASE_H

#include "temoto_action_engine/waitlist.h"

struct Notification
{
  std::string parameters;
  std::string result;
  Waitable    waitable;
};

struct GraphDescriptor
{
  std::string actor_name;
  std::string graph_name;
  std::string parameters;
  std::string result;
};

class ActionSynchronizerPluginBase
{
public:

  virtual void sendNotification(const Waitable& waitable, const std::string& result, const std::string& params) = 0;

  void setNotificationReceivedCallback(std::function<void(const Notification&)> notification_received_cb)
  {
    notification_received_cb_ = notification_received_cb;
  }

  void setExecuteGraphCallback(std::function<void(const GraphDescriptor&)> execute_graph_cb)
  {
    execute_graph_cb_ = execute_graph_cb;
  }

private:

  std::function<void(const Notification&)> notification_received_cb_;

  std::function<void(const GraphDescriptor&)> execute_graph_cb_;
};

#endif