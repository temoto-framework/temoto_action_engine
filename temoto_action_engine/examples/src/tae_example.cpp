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

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    std::cout << "missing the UMRF graph name argument" << std::endl;
    return 1;
  }

  std::string umrf_graph_name = argv[1];
  ActionEngine action_engine;

  // Tell the Action Engine where to look for the actions (in temoto_action_engine/build/actions)
  action_engine.addActionsPath("actions");

  // Get the UMRF graph json
  std::ifstream umrf_graph_json_fs("../examples/umrf_graphs/" + umrf_graph_name);
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