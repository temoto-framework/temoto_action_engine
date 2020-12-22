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

#ifndef TEMOTO_ACTION_ENGINE__UMRF_GRAPH_NODE_EXEC_H
#define TEMOTO_ACTION_ENGINE__UMRF_GRAPH_NODE_EXEC_H

#include <memory>
#include <string>
#include <future>
#include <class_loader/class_loader.hpp>
#include <boost/shared_ptr.hpp>
#include "temoto_action_engine/compiler_macros.h"
#include "temoto_action_engine/temoto_error.h"
#include "temoto_action_engine/umrf_graph_node.h"

class UmrfGraphNodeExec : public UmrfGraphNode
{
public:
  UmrfGraphNodeExec();

};

#endif