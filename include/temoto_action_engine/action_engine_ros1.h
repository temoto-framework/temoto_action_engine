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

#ifndef TEMOTO_ACTION_ENGINE__ACTION_ENGINE_ROS1_H
#define TEMOTO_ACTION_ENGINE__ACTION_ENGINE_ROS1_H

#include "ros/ros.h"
#include "temoto_action_engine/action_engine.h"
#include "temoto_action_engine/UmrfGraphRos1.h"
#include "temoto_action_engine/StopUmrfGraph.h"
#include "temoto_action_engine/GetUmrfGraphs.h"
#include <string>
#include <vector>

namespace temoto_action_engine
{
class ActionEngineRos1
{
public:
  ActionEngineRos1();
  
  /**
   * @brief Reads in the commandline arguments for action base path and initiates the subscriber
   * 
   * @param argc 
   * @param argv 
   */
  void initialize();

  /**
   * @brief Parses the command line parameters
   * 
   */
  void parseCmdArguments(int argc, char** argv);

private:

  bool containsWakeWord(const std::vector<std::string>& wake_words_in) const;

  /**
   * @brief Callback for executing UMRF graphs
   * 
   * @param msg 
   */
  void umrfGraphCallback(const temoto_action_engine::UmrfGraphRos1& msg);

  /**
   * @brief Callback for stopping UMRF graphs
   * 
   * @param msg 
   */
  void stopUmrfGraphCallback(const temoto_action_engine::StopUmrfGraph& msg);

  bool GetUmrfGraphsCb(temoto_action_engine::GetUmrfGraphs::Request& req
  , temoto_action_engine::GetUmrfGraphs::Response& res);

  ActionEngine ae_;
  ros::NodeHandle nh_;
  ros::Subscriber umrf_graph_sub_;
  ros::Subscriber stop_umrf_graph_sub_;
  ros::ServiceServer get_umrf_graphs_server_;

  // Action Engine setup parameters
  std::vector<std::string> action_paths_;
  std::vector<std::string> wake_words_; 
};
} // temoto_action_engine namespace

#endif