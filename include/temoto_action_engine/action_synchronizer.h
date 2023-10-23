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

#include <class_loader/multi_library_class_loader.hpp>
#include <memory>
#include <mutex>
#include <vector>

std::string camelToSnake(std::string str_camel_case)
{
  std::string str_snake_case = "";
  char c = tolower(str_camel_case[0]);
  str_snake_case += (char(c));

  for (int i = 1; i < str_camel_case.length(); i++) 
  {
    char ch = str_camel_case[i];
    if (isupper(ch)) 
    {
      str_snake_case = str_snake_case + '_';
      str_snake_case += char(tolower(ch));
    }
    else 
    {
      str_snake_case = str_snake_case + ch;
    }
  }
  return str_snake_case;
}

class ActionSynchronizer
{
public:

  ActionSynchronizer(std::vector<std::string> plugin_names)
  {
    class_loader_ = std::make_shared<class_loader::MultiLibraryClassLoader>(false);

    for(const auto& plugin_name : plugin_names)
    {
      auto plugin = class_loader_->createSharedInstance<ActionSynchronizerPluginBase>(plugin_name, plugin_name + ".so");
      // TODO: plugin->setNotificationReceivedCallback();
      sync_plugins_.push_back(plugin);
    }
  }

  void sendNotify(const Waitable& waitable)
  {
    std::lock_guard<std::mutex> l(m_);
    for (const auto& plugin : sync_plugins_)
    {
      plugin->sendNotification(waitable);
    }
  }

  void onNotificationReceived(/*todo*/)
  {
    ENGINE_HANDLE.notifyFinished()
  }

private:

std::shared_ptr<class_loader::MultiLibraryClassLoader> class_loader_;
std::vector<std::shared_ptr<ActionSynchronizerPluginBase>> sync_plugins_;
std::mutex m_;

};

#endif