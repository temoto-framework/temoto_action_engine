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

#include "temoto_action_engine/action_engine_ros1.h"
#include "temoto_action_engine/temoto_error.h" 
#include "temoto_action_engine/umrf_json_converter.h"
#include "temoto_action_engine/umrf_graph_diff.h"
#include "temoto_action_engine/messaging.h"
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <sstream>

namespace temoto_action_engine
{

ActionEngineRos1::ActionEngineRos1()
{}

void ActionEngineRos1::initialize()
{
  // Set up the UMRF graph subscriber to a globally namespaced topic
  start_umrf_graph_sub_ = nh_.subscribe("/broadcast_start_umrf_graph", 1, &ActionEngineRos1::broadcastStartUmrfGraphCallback, this);
  stop_umrf_graph_sub_ = nh_.subscribe("/broadcast_stop_umrf_graph", 1, &ActionEngineRos1::broadcastStopUmrfGraphCallback, this);

  start_umrf_graph_srv_ = nh_.advertiseService("start_umrf_graph"
  , &ActionEngineRos1::StartUmrfGraphSrvCallback
  , this);

  stop_umrf_graph_srv_ = nh_.advertiseService("stop_umrf_graph"
  , &ActionEngineRos1::StopUmrfGraphSrvCallback
  , this);

  get_umrf_graphs_server_ = nh_.advertiseService("get_umrf_graphs"
  , &ActionEngineRos1::GetUmrfGraphsCb
  , this);

  // Set the default action paths
  int successful_paths = 0;
  for (const auto& ap : action_paths_)
  {
    try
    {
      if (ae_.addActionsPath(ap))
      {
        successful_paths++;
      }
    }
    catch(const std::exception& e)
    {
      TEMOTO_PRINT(e.what());
    }
  }

  if (successful_paths == 0)
  {
    throw CREATE_TEMOTO_ERROR("None of the indicated directories contained TeMoto actions, exiting.");
  }

  // Start the action engine
  ae_.start();
}

void ActionEngineRos1::parseCmdArguments(int argc, char** argv)
{
  namespace po = boost::program_options;
  po::variables_map vm;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("wake-word", po::value<std::string>(), "Required. Main wake word.")
    ("actions-path", po::value<std::string>(), "Required. Action packages root path")
    ("extra-wake-words", po::value<std::string>(), "Optional. Additional wake words. Indicates to which wake words the action engine will respond to.");

  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  /*
   * Get the wake words
   */ 
  if (vm.count("extra-wake-words"))
  {
    std::string wake_words_str = vm["extra-wake-words"].as<std::string>();
    boost::replace_all(wake_words_str, " ", "");
    boost::split(wake_words_, wake_words_str, boost::is_any_of(","));
  }

  /*
   * Get the main wake word
   */ 
  if (vm.count("wake-word"))
  {
    std::string main_wake_word = vm["wake-word"].as<std::string>();
    wake_words_.push_back(main_wake_word);

    TEMOTO_PRINT("Wake words that this Action Engine Node responds to:");
    for (const auto& ww : wake_words_)
    {
      std::cout << " * " << ww << std::endl;
    }
    std::cout << std::endl;
  }
  else
  {
    std::stringstream ss;
    ss << "Missing the main wake word\n" << desc;
    throw CREATE_TEMOTO_ERROR(ss.str());
  }

  /*
   * Get the action packages path file and get the paths    
   */
  if (vm.count("actions-path"))
  {
    action_paths_.push_back(vm["actions-path"].as<std::string>());
  }
  else
  {
    std::stringstream ss;
    ss << "Missing action packages path file\n" << desc;
    throw CREATE_TEMOTO_ERROR(ss.str());
  }
}

bool ActionEngineRos1::containsWakeWord(const std::vector<std::string>& wake_words_in) const
{
  for (const auto& target : wake_words_in)
  {
    for (const auto& ww : wake_words_)
    {
      if (ww == target)
      {
        return true;
      }  
    }
  }
  return false;
}

void ActionEngineRos1::broadcastStartUmrfGraphCallback(const temoto_action_engine::BroadcastStartUmrfGraph& msg)
{
  std::lock_guard<std::mutex> lock(start_umrf_graph_mutex_);
  TEMOTO_PRINT("Received a UMRF graph message ...");

  // If the wake word was not found then return
  if (!containsWakeWord(msg.targets))
  {
    TEMOTO_PRINT("The UMRF graph message was not targeted at this Action Engine.");
    return;
  }

  /*
   * Check wether it's a diff request or new graph request
   */ 
  if (!msg.umrf_graph_json.empty())
  {
    /*
     * Instantiate a new umrf graph
     */ 
    try
    {
      UmrfGraph umrf_graph = umrf_json_converter::fromUmrfGraphJsonStr(msg.umrf_graph_json);
      ae_.executeUmrfGraph(umrf_graph, bool(msg.name_match_required));
    }
    catch(const std::exception& e)
    {
      TEMOTO_PRINT(std::string(e.what()));
    }
  }
  else if (!msg.umrf_graph_diffs.empty())
  {
    /*
     * Modify an existing umrf graph according to the diff specifiers
     */
    try
    {
      UmrfGraphDiffs umrf_graph_diffs;
      for(const auto& umrf_graph_diff_msg : msg.umrf_graph_diffs)
      {
        UmrfNode umrf_diff = umrf_json_converter::fromUmrfJsonStr(umrf_graph_diff_msg.umrf_json);
        umrf_graph_diffs.emplace_back(umrf_graph_diff_msg.operation, umrf_diff);
      }

      ae_.modifyGraph(msg.umrf_graph_name, umrf_graph_diffs);
    }
    catch(const std::exception& e)
    {
      TEMOTO_PRINT(std::string(e.what()));
    }
  }
  else
  {
    TEMOTO_PRINT("The UMRF graph message has no content, aborting the request.");
  }
}

void ActionEngineRos1::broadcastStopUmrfGraphCallback(const temoto_action_engine::BroadcastStopUmrfGraph& msg)
{
  std::lock_guard<std::mutex> lock(stop_umrf_graph_mutex_);
  TEMOTO_PRINT("Received a UMRF graph STOPPING message ...");

  // If the wake word was not found then return
  if (!containsWakeWord(msg.targets))
  {
    TEMOTO_PRINT("The stop message was not targeted at this Action Engine.");
    return;
  }

  TEMOTO_PRINT("Stopping UMRF graph '" + msg.umrf_graph_name + "' ...");
  try
  {
    ae_.stopUmrfGraph(msg.umrf_graph_name);
  }
  catch(const std::exception& e)
  {
    TEMOTO_PRINT(std::string(e.what()));
  }
}

bool ActionEngineRos1::StartUmrfGraphSrvCallback(temoto_action_engine::StartUmrfGraph::Request& req
, temoto_action_engine::StartUmrfGraph::Response& res)
try
{
  std::lock_guard<std::mutex> lock(start_umrf_graph_mutex_);

  TEMOTO_PRINT("Starting UMRF graph '" + req.umrf_graph_name + "' ...");
  UmrfGraph umrf_graph = umrf_json_converter::fromUmrfGraphJsonStr(req.umrf_graph_json);
  ae_.executeUmrfGraph(umrf_graph, bool(req.name_match_required));

  res.success = true;
  return true;
}
catch(const std::exception& e)
{
  TEMOTO_PRINT(std::string(e.what()));
  res.success = false;
  return true;
}

bool ActionEngineRos1::StopUmrfGraphSrvCallback(temoto_action_engine::StopUmrfGraph::Request& req
, temoto_action_engine::StopUmrfGraph::Response& res)
try
{
  std::lock_guard<std::mutex> lock(stop_umrf_graph_mutex_);
  TEMOTO_PRINT("Stopping UMRF graph '" + req.umrf_graph_name + "' ...");
  ae_.stopUmrfGraph(req.umrf_graph_name);
  res.success = true;
  return true;
}
catch(const std::exception& e)
{
  TEMOTO_PRINT(std::string(e.what()));
  res.success = false;
  return true;
}

bool ActionEngineRos1::GetUmrfGraphsCb(temoto_action_engine::GetUmrfGraphs::Request& req
, temoto_action_engine::GetUmrfGraphs::Response& res)
{
  res.umrf_graph_jsons = ae_.getGraphJsons();
  return true;
}

} // temoto_action_engine namespace