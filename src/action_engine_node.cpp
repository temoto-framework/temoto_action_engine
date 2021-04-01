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

#include "ros/ros.h"
#include "temoto_action_engine/action_engine_ros1.h"

int main(int argc, char** argv)
try
{
  /*
   * Initialize ROS
   */
  ros::init(argc, argv, "temoto_action_engine_node");

  /*
   * Start the Action Engine ROS wrapper
   */ 
  temoto_action_engine::ActionEngineRos1 action_engine;
  action_engine.parseCmdArguments(argc, argv);
  action_engine.initialize();

  /*
   * Set up the spinner
   */
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
catch (const std::exception& e)
{
  std::cerr << e.what() << std::endl;
  return 1;
}

