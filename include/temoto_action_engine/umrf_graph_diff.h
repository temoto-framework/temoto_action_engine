/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright 2020 TeMoto Telerobotics
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

#ifndef TEMOTO_ACTION_ENGINE__UMRF_GRAPH_DIFF_H
#define TEMOTO_ACTION_ENGINE__UMRF_GRAPH_DIFF_H

#include <string>
#include <vector>
#include "temoto_action_engine/umrf_node.h"

struct UmrfGraphDiff
{
  static struct Operation
  {
    static constexpr const char* add_umrf = "add umrf";
    static constexpr const char* add_child = "add child";
    static constexpr const char* remove_umrf = "remove umrf";
    static constexpr const char* remove_child = "remove child";
  }OP;

  UmrfGraphDiff(const std::string& operation_in, const UmrfNode& umrf_in)
  : operation(operation_in)
  , umrf_node(umrf_in)
  {  
  }

  std::string operation;
  UmrfNode umrf_node;
};

typedef std::vector<UmrfGraphDiff> UmrfGraphDiffs;

#endif