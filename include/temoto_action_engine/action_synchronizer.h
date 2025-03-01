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
#include "temoto_action_engine/action_plugin.hpp"

#include <memory>
#include <mutex>
#include <vector>
#include <thread>
#include <condition_variable>
#include <deque>

class ActionSynchronizer
{
public:

  ActionSynchronizer(const std::string& plugin_name, const std::string actor_name)
  : sync_plugin_(plugin_name)
  {
    sync_plugin_.load();
    sync_plugin_.get()->setNotificationReceivedCallback(std::bind(&ActionSynchronizer::onNotificationReceived, this, std::placeholders::_1));
    sync_plugin_.get()->setExecuteGraphCallback(std::bind(&ActionSynchronizer::onExecuteGraph, this, std::placeholders::_1));
    sync_plugin_.get()->setName(actor_name);

    th_ = std::thread{[&]
    {
      auto buffer_empty = [&]
      {
        std::lock_guard<std::mutex> l(m_);
        return notification_buffer_.empty();
      };

      while (!th_stop_)
      {
        std::unique_lock<std::mutex> l(m_);
        cv_.wait(l, [&]{return !notification_buffer_.empty() || th_stop_;});
        l.unlock();

        if (th_stop_)
          break;

        while (!buffer_empty())
        {
          auto n{[&]
          {
            std::lock_guard<std::mutex> l(m_);
            auto n{notification_buffer_.back()};
            notification_buffer_.pop_back();
            return n;
          }()};

          std::cout << "[" << ENGINE_HANDLE.getActor() << "::" << __func__ << "]" << " processing notification " << n.id << std::endl;

          if (n.parameters.empty())
          {
            ENGINE_HANDLE.notifyFinished(n.waitable
            , n.result
            , ActionParameters()
            , n.id);
          }
          else
          {
            ENGINE_HANDLE.notifyFinished(n.waitable
            , n.result
            , umrf_json::fromUmrfParametersJsonStr(n.parameters)
            , n.id);
          }
        }
      }
    }};
  }

  ~ActionSynchronizer()
  {
    th_stop_ = true;
    cv_.notify_all();
    while (!th_.joinable()){}
    th_.join();
  }

  bool sendNotify(const Waitable& waitable
  , const std::string& result
  , const ActionParameters& params
  , const std::set<std::string> other_actors
  , const size_t timeout = 5000)
  {
    std::string params_json_str{params.empty() ? std::string() :
      umrf_json::toUmrfParametersJsonStr(params)};

    Notification n;
    n.parameters = params_json_str;
    n.result = result;
    n.waitable = waitable;

    std::lock_guard<std::mutex> l(mutex_notify_);
    return sync_plugin_.get()->notify(n, other_actors, timeout);
  }

  bool bidirHandshake(const std::string& handshake_token, const std::set<std::string> other_actors, size_t timeout)
  {
    std::lock_guard<std::mutex> l(mutex_handshake_);
    return sync_plugin_.get()->bidirHandshake(handshake_token, other_actors, timeout);
  }

  bool unidirHandshake(const std::string& handshake_token)
  {
    std::lock_guard<std::mutex> l(mutex_handshake_);
    return sync_plugin_.get()->unidirHandshake(handshake_token);
  }

private:

  void onNotificationReceived(const Notification& n)
  {
    // std::cout << "[" << ENGINE_HANDLE.getActor() << "::" << __func__ << "]" << " got params from '" << n.waitable.actor_name << "::" << n.waitable.action_name << "' " << n.parameters << std::endl;
    std::lock_guard<std::mutex> l(m_);
    notification_buffer_.push_front(n);
    cv_.notify_all();
  }

  void onExecuteGraph(const GraphDescriptor& gd)
  {
    ENGINE_HANDLE.executeUmrfGraph(gd.graph_name
    , umrf_json::fromUmrfParametersJsonStr(gd.parameters)
    , gd.result);
  }

  ActionPlugin<ActionSynchronizerPluginBase> sync_plugin_;
  std::mutex mutex_notify_;
  std::mutex mutex_handshake_;

  std::deque<Notification> notification_buffer_;
  std::thread th_;
  std::mutex m_;
  std::condition_variable cv_;
  bool th_stop_ = false;

};

#endif
