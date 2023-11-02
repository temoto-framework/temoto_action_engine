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

  ActionSynchronizer(std::vector<std::string> plugin_names)
  {
    class_loader_ = std::make_shared<class_loader::MultiLibraryClassLoader>(false);

    for(const auto& plugin_name : plugin_names)
    {
      class_loader_->loadLibrary("lib" + plugin_name + ".so");
      auto plugin = class_loader_->createSharedInstance<ActionSynchronizerPluginBase>(plugin_name);
      plugin->setNotificationReceivedCallback(std::bind(&ActionSynchronizer::onNotificationReceived, this, std::placeholders::_1));
      plugin->setExecuteGraphCallback(std::bind(&ActionSynchronizer::onExecuteGraph, this, std::placeholders::_1));
      sync_plugins_.push_back(plugin);
    }
  }

  void sendNotify(const Waitable& waitable, const std::string& result, const ActionParameters& params)
  {
    std::string params_json_str = umrf_json::toUmrfParametersJsonStr(params);
    std::lock_guard<std::mutex> l(m_);
    
    for (const auto& plugin : sync_plugins_)
    {
      plugin->sendNotification(waitable, result, params_json_str);
    }

    /*
     * TODO: Wait for acknowledgement by all agents
     */
  }

private:

  void onNotificationReceived(const Notification& n)
  {
    /*
     * TODO: Make sure that the same notification has not already been received from
     * other synchronizer plugin
     */

    ENGINE_HANDLE.notifyFinished(n.waitable
    , n.result
    , umrf_json::fromUmrfParametersJsonStr(n.parameters));
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
  std::vector<std::shared_ptr<ActionSynchronizerPluginBase>> sync_plugins_;
  std::mutex m_;

};

#endif