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

#include <iostream>
#include <chrono>
#include <fstream>
#include "temoto_action_engine/action_engine.h"
#include "temoto_action_engine/umrf_json_converter.h"
#include <boost/program_options.hpp>

namespace po = boost::program_options;

int main(int argc, char** argv)
try
{
  po::variables_map vm;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "Show help message")
    ("actions-path", po::value<std::string>(), "Action packages root path")
    ("umrf-graph", po::value<std::string>(), "Input UMRF graph path");

  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  std::string actions_path;
  std::string umrf_graph_path;

  // Print help message
  if (vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 0;
  }

  // Get the action packages path file and get the paths    
  if (vm.count("actions-path"))
  {
    actions_path = vm["actions-path"].as<std::string>();
  }
  else
  {
    TEMOTO_PRINT("Missing action packages path file");
    std::cout << desc << std::endl;
    return 1;
  }

  // Get the path to the input umrf graph
  if (vm.count("umrf-graph"))
  {
    umrf_graph_path = vm["umrf-graph"].as<std::string>();
  }
  else
  {
    TEMOTO_PRINT("Missing path to input UMRF graph");
    std::cout << desc << std::endl;
    return 1;
  }

  std::string umrf_graph_name = argv[1];
  ActionEngine action_engine;

  // Tell the Action Engine where to look for the actions (in temoto_action_engine/build/actions)
  if (!action_engine.addActionsPath(actions_path))
  {
    throw CREATE_TEMOTO_ERROR(actions_path + " does not contain any TeMoto actions");
  }

  // Get the UMRF graph json
  std::ifstream umrf_graph_json_fs(umrf_graph_path);
  std::string umrf_graph_json_str;
  umrf_graph_json_str.assign(std::istreambuf_iterator<char>(umrf_graph_json_fs), std::istreambuf_iterator<char>());
  std::cout << "Got UMRF graph:\n" << umrf_graph_json_str << std::endl;

  // Convert UMRF graph json string to UmrfGraph datastructure
  UmrfGraph umrf_graph = umrf_json_converter::fromUmrfGraphJsonStr(umrf_graph_json_str);

  // Execute the UMRF graph
  action_engine.executeUmrfGraph(umrf_graph);

  // As this is not a blocking call, we can do something "useful" while the UMRF graph is being run
  while(true)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  return 0;
}
catch (const std::exception& e)
{
  std::cerr << e.what() << std::endl;
  return 1;
}