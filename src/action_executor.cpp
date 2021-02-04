// /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
//  * Copyright 2019 TeMoto Telerobotics
//  * 
//  * Licensed under the Apache License, Version 2.0 (the "License");
//  * you may not use this file except in compliance with the License.
//  * You may obtain a copy of the License at
//  * 
//  *     http://www.apache.org/licenses/LICENSE-2.0
//  * 
//  * Unless required by applicable law or agreed to in writing, software
//  * distributed under the License is distributed on an "AS IS" BASIS,
//  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  * See the License for the specific language governing permissions and
//  * limitations under the License.
//  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// /* Author: Robert Valner */

// #include "temoto_action_engine/action_executor.h"
// #include "temoto_action_engine/temoto_error.h"
// #include "temoto_action_engine/messaging.h"
// #include <set>
 

// void ActionExecutor::notifyFinished(const unsigned int& parent_action_id, const ActionParameters& parent_action_parameters)
// {
//   LOCK_GUARD_TYPE_R guard_handles(named_action_handles_rw_mutex_);
//   LOCK_GUARD_TYPE_R guard_graphs(named_umrf_graphs_rw_mutex_);
//   /*
//    * Check the UMRF graphs and execute sequential actions if necessary
//    */
//   try
//   {
//     for (auto& umrf_graph_pair : named_umrf_graphs_)
//     {
//       if ((umrf_graph_pair.second.checkState() != UmrfGraph::State::ACTIVE) ||
//           (umrf_graph_pair.second.getChildrenOf(parent_action_id).empty()))
//       {
//         continue;
//       }

//       /*
//        * Transfer the parameters from parent to child action
//        */
//       for (const auto& child_id : umrf_graph_pair.second.getChildrenOf(parent_action_id))
//       {
//         Umrf& child_umrf = umrf_graph_pair.second.getUmrfOfNonconst(child_id);
//         child_umrf.copyInputParameters(parent_action_parameters);
//         child_umrf.setParentReceived(umrf_graph_pair.second.getUmrfOf(parent_action_id).asRelation());
//       }
//       executeById(umrf_graph_pair.second.getChildrenOf(parent_action_id), umrf_graph_pair.second);
//     }
//   }
//   catch(TemotoErrorStack e)
//   {
//     throw FORWARD_TEMOTO_ERROR_STACK(e);
//   }
//   catch(const std::exception& e)
//   {
//     throw CREATE_TEMOTO_ERROR_STACK(std::string(e.what()));
//   }
// }

// void ActionExecutor::executeUmrfGraph(const std::string& graph_name)
// {
//   LOCK_GUARD_TYPE_R guard_handles(named_action_handles_rw_mutex_);
//   LOCK_GUARD_TYPE_R guard_graphs(named_umrf_graphs_rw_mutex_);
//   try
//   {
//     // Check if the requested graph exists
//     if (!graphExists(graph_name))
//     {
//       throw CREATE_TEMOTO_ERROR_STACK("Cannot execute UMRF graph '" + graph_name + "' because it doesn't exist.");
//     }

//     // Check if the graph is in initialized state
//     /*
//      * TODO: Also check if the graph is in running state and could it be updated
//      * i.e., does any of its actions are accepting parameters that can be updated (pvf_updatable = true)
//      */
//     if (named_umrf_graphs_.at(graph_name).checkState() != UmrfGraph::State::INITIALIZED)
//     {
//       throw CREATE_TEMOTO_ERROR_STACK("Cannot execute UMRF graph '" + graph_name + "' because it's not in initialized state.");
//     }

//     UmrfGraph& ugh = named_umrf_graphs_.at(graph_name);
//     std::vector<unsigned int> action_ids = ugh.getRootNodes();
//     executeById(action_ids, ugh, true);
//   }
//   catch(TemotoErrorStack e)
//   {
//     throw FORWARD_TEMOTO_ERROR_STACK(e);
//   }
// }
