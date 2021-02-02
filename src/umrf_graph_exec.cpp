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

#include "temoto_action_engine/umrf_graph_exec.h"

UmrfGraphExec::UmrfGraphExec(const UmrfGraphExec& ug)
: UmrfGraphBase(ug)
{}

UmrfGraphExec::UmrfGraphExec(const UmrfGraphBase& ugb)
: UmrfGraphBase(ugb)
{}

UmrfGraphExec::UmrfGraphExec(const std::string& graph_name)
: UmrfGraphBase(graph_name)
{}

UmrfGraphExec::UmrfGraphExec(const std::string& graph_name, const std::vector<Umrf>& umrfs_vec)
: UmrfGraphBase(graph_name, umrfs_vec)
{}

UmrfGraphExec::~UmrfGraphExec()
{}

// UmrfGraphBase UmrfGraphExec::toUmrfGraphBase()
// {
//   return UmrfGraphBase(*this);
// }