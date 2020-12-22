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

#ifndef TEMOTO_ACTION_ENGINE__UMRF_GRAPH_BASE_H
#define TEMOTO_ACTION_ENGINE__UMRF_GRAPH_BASE_H

#include "temoto_action_engine/umrf_graph_base.h"
#include "temoto_action_engine/umrf_graph_node.h"

class UmrfGraph : public UmrfGraphBase
{
public:
 

private:

  virtual bool createMaps(const std::vector<Umrf>& umrfs_vec);

  /// Helps to resolve UMRF name to a GraphNode
  typedef std::map<std::string, UmrfGraphNode> UmrfGraphNodeMap;
  mutable MUTEX_TYPE_R graph_node_map_rw_mutex_;
  GUARDED_VARIABLE(UmrfGraphNodeMap graph_node_map_, graph_node_map_rw_mutex_);
};
#endif