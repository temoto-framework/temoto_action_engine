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

#include <iostream>
#include <chrono>
#include "temoto_action_engine/action_engine.h"
#include "temoto_action_engine/umrf_json.h"
#include <boost/program_options.hpp>

namespace po = boost::program_options;

int main(int argc, char** argv)
try
{
  po::variables_map vm;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "Show help message")
    ("actor",        po::value<std::string>(), "Name of this action engine instance")
    ("actions-path", po::value<std::string>(), "Action packages root path")
    ("sync-plugin",  po::value<std::string>(), "Name of the action synchronization plugin")
    ("graph-name",   po::value<std::string>(), "Input UMRF graph path");

  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  std::string actor;
  std::string actions_path;
  std::string sync_plugin;
  std::string graph_name;

  // Print help message
  if (vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 0;
  }

  // Get the name of this action engine instance
  if (vm.count("actor"))
  {
    actor = vm["actor"].as<std::string>();
  }
  else
  {
    TEMOTO_PRINT("Missing the name of the actor");
    std::cout << desc << std::endl;
    return 1;
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

  // Get name of the action synchronization plugin
  if (vm.count("sync-plugin"))
  {
    sync_plugin = vm["sync-plugin"].as<std::string>();
  }

  // Get the path to the input umrf graph
  if (vm.count("graph-name"))
  {
    graph_name = vm["graph-name"].as<std::string>();
  }
  else
  {
    TEMOTO_PRINT("Missing path to input UMRF graph");
    std::cout << desc << std::endl;
    return 1;
  }

  std::unique_ptr<ActionEngine> action_engine;

  if (!sync_plugin.empty())
  {
    action_engine = std::make_unique<ActionEngine>(actor, 0, sync_plugin);
  }
  else
  {
    action_engine = std::make_unique<ActionEngine>(actor);
  }

  // Tell the Action Engine where to look for the actions (in temoto_action_engine/build/actions)
  if (!action_engine->addActionsPath(actions_path))
  {
    throw CREATE_TEMOTO_ERROR(actions_path + " does not contain any TeMoto actions");
  }

  // Execute the UMRF graph and wait for the result
  action_engine->startGraph(graph_name);
  std::string result = action_engine->waitForGraph(graph_name);

  return (result == "on_true") ? 0 : 1;
}
catch (const std::exception& e)
{
  std::cerr << e.what() << std::endl;
  return 1;
}
