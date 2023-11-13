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

#ifndef TEMOTO_ACTION_ENGINE__ACTION_SYNCHRONIZER_H
#define TEMOTO_ACTION_ENGINE__ACTION_SYNCHRONIZER_H

#include "temoto_action_engine/waitlist.h"
#include "temoto_action_engine/action_synchronizer_plugin_base.h"
#include "temoto_action_engine/action_engine_handle.h"
#include "temoto_action_engine/umrf_json.h"

#include <class_loader/multi_library_class_loader.hpp>
#include <memory>
#include <mutex>
#include <vector>

class ActionSynchronizer
{
public:

  ActionSynchronizer(const std::string& plugin_name, const std::string actor_name)
  {
    class_loader_ = std::make_shared<class_loader::MultiLibraryClassLoader>(false);
    class_loader_->loadLibrary("lib" + plugin_name + ".so");
    sync_plugin_ = class_loader_->createSharedInstance<ActionSynchronizerPluginBase>(plugin_name);
    sync_plugin_->setNotificationReceivedCallback(std::bind(&ActionSynchronizer::onNotificationReceived, this, std::placeholders::_1));
    sync_plugin_->setExecuteGraphCallback(std::bind(&ActionSynchronizer::onExecuteGraph, this, std::placeholders::_1));
    sync_plugin_->setName(actor_name);
  }

  void sendNotify(const Waitable& waitable, const std::string& result, const ActionParameters& params)
  {
    std::string params_json_str{params.empty() ? std::string() :
      umrf_json::toUmrfParametersJsonStr(params)};
    std::lock_guard<std::mutex> l(mutex_notify_);
    
    Notification n;
    n.parameters = params_json_str;
    n.result = result;
    n.waitable = waitable;

    sync_plugin_->notify(n);
  }

  bool multiActorHandshake(const std::string& handshake_token, const std::set<std::string> other_actors, size_t timeout)
  {
    std::lock_guard<std::mutex> l(mutex_handshake_);
    return sync_plugin_->handshake(handshake_token, other_actors, timeout);
  }

private:

  void onNotificationReceived(const Notification& n)
  {
    /*
     * TODO: Make sure that the same notification has not already been received from
     * other synchronizer plugin
     */

    std::cout << "[" << ENGINE_HANDLE.getActor() << "::" << __func__ << "]" << " got params from '" << n.waitable.actor_name << "::" << n.waitable.action_name << "' " << n.parameters << std::endl;

    if (n.parameters.empty())
    {
      ENGINE_HANDLE.notifyFinished(n.waitable
      , n.result
      , ActionParameters());
    }
    else
    {
      ENGINE_HANDLE.notifyFinished(n.waitable
      , n.result
      , umrf_json::fromUmrfParametersJsonStr(n.parameters));
    }
  }

  void onExecuteGraph(const GraphDescriptor& gd)
  {
    /*
     * TODO: Make sure that the same graph has not already been received from
     * other synchronizer plugin
     */

    ENGINE_HANDLE.executeUmrfGraph(gd.graph_name
    , umrf_json::fromUmrfParametersJsonStr(gd.parameters)
    , gd.result);
  }

  std::shared_ptr<class_loader::MultiLibraryClassLoader> class_loader_;
  std::shared_ptr<ActionSynchronizerPluginBase> sync_plugin_;
  std::mutex mutex_notify_;
  std::mutex mutex_handshake_;

};

#endif