/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright 2025 TeMoto Framework
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

#include "temoto_action_engine/action_engine.h"
#include "temoto_action_engine/util/logging.hpp"
#include "temoto_action_engine/umrf_json.h"

#include <boost/circular_buffer.hpp>

boost::circular_buffer<std::string> feedback_buffer(100);
std::mutex feedback_buffer_m;

ActionEngine::ActionEngine(
  const std::string& actor_name,
  const unsigned int& indexing_rate,
  const std::string& sync_plugin_name)
: actor_name_(actor_name)
, ai_{indexing_rate}
{
  ENGINE_HANDLE.actor_name_ = actor_name_;

  ENGINE_HANDLE.acknowledge_fptr_ = std::bind(&ActionEngine::acknowledge, this
  , std::placeholders::_1);

  ENGINE_HANDLE.execute_graph_fptr_ = std::bind(&ActionEngine::executeUmrfGraph, this
  , std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

  ENGINE_HANDLE.notify_finished_fptr_ = std::bind(&ActionEngine::notifyFinished, this
  , std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);

  ENGINE_HANDLE.add_waiter_fptr_ = std::bind(&ActionEngine::addWaiter, this
  , std::placeholders::_1, std::placeholders::_2);

  ENGINE_HANDLE.on_state_change_fptr = std::bind(&ActionEngine::onStateChange, this
  , std::placeholders::_1, std::placeholders::_2);

  if (!sync_plugin_name.empty())
  {
    as_ = std::make_unique<ActionSynchronizer>(sync_plugin_name, actor_name_);
  }
}

void ActionEngine::onStateChange([[maybe_unused]] const std::string& action_name, const std::string& graph_name)
{
  // If graphs are being modified, then do not send any state updates
  if (umrf_graph_map_rw_mutex_.try_lock())
  {
    umrf_graph_map_rw_mutex_.unlock();
  }
  else
  {
    return;
  }

  LOCK_GUARD_TYPE l(feedback_buffer_m);
  feedback_buffer.push_front([&]
  {
    LOCK_GUARD_TYPE_R guard_graphs_map_(umrf_graph_map_rw_mutex_);
    return umrf_json::toUmrfGraphJsonStr(umrf_graph_exec_map_.at(graph_name)->toUmrfGraphCommon());
  }());
}

std::vector<std::string> ActionEngine::readFeedbackBuffer()
{
  LOCK_GUARD_TYPE l(feedback_buffer_m);

  std::vector<std::string> vec(feedback_buffer.begin(), feedback_buffer.end());
  feedback_buffer.clear();

  return vec;
}

bool ActionEngine::graphExists(const std::string& graph_name) const
{
  LOCK_GUARD_TYPE_R guard_graph_map_(umrf_graph_map_rw_mutex_);
  return (umrf_graph_exec_map_.find(graph_name) != umrf_graph_exec_map_.end());
}

void ActionEngine::addUmrfGraph(const std::string& graph_name, const std::vector<UmrfNode>& umrf_nodes)
{
  LOCK_GUARD_TYPE_R guard_graph_map_(umrf_graph_map_rw_mutex_);
  umrf_graph_exec_map_.emplace(graph_name, std::make_shared<UmrfGraphExec>(graph_name, umrf_nodes));
}

void ActionEngine::executeUmrfGraphA(UmrfGraph umrf_graph, const std::string& result, [[maybe_unused]] bool name_match_required)
try
{
  LOCK_GUARD_TYPE_R guard_graph_map_(umrf_graph_map_rw_mutex_);

  const auto& graph_name = umrf_graph.getName();

  if (!graphExists(graph_name))
  {
    if (!matchGraph(umrf_graph, {graph_name}))
    {
      throw CREATE_TEMOTO_ERROR_STACK("Could not resolve graph '" + graph_name + "'");
    }

    TEMOTO_PRINT("All actions in graph '" + graph_name + "' found.");
  }
  else if (umrf_graph_exec_map_.at(graph_name)->getState() == UmrfGraphExec::State::FINISHED)
  {
    finished_graphs_.erase(graph_name);
    umrf_graph_exec_map_.erase(graph_name);
  }

  umrf_graph_exec_map_.emplace(graph_name, std::make_shared<UmrfGraphExec>(umrf_graph));
  TEMOTO_PRINT("UMRF graph '" + graph_name + "' initialized.");

  executeUmrfGraph(graph_name, ActionParameters{}, result);
  TEMOTO_PRINT("UMRF graph '" + graph_name + "' invoked successfully.");
}
catch(TemotoErrorStack e)
{
  throw FORWARD_TEMOTO_ERROR_STACK(e);
}

void ActionEngine::executeUmrfGraph(const std::string& graph_name, const ActionParameters& params, const std::string& result)
try
{
  LOCK_GUARD_TYPE_R guard_graph_map_(umrf_graph_map_rw_mutex_);

  auto insertIndexedGraph = [&](const std::string& graph_name)
  {
    const auto& indexed_graphs = ai_.getGraphs();
    const auto known_graph = std::find_if(indexed_graphs.begin(), indexed_graphs.end(),
    [&](const UmrfGraph& ug)
    {
      return ug.getName() == graph_name;
    });

    if (known_graph == indexed_graphs.end())
    {
      throw CREATE_TEMOTO_ERROR_STACK("Cannot execute UMRF graph '" + graph_name + "' because it doesn't exist.");
    }

    // Find a matching action for this UMRF
    UmrfGraph known_graph_cpy = *known_graph;
    if (!matchGraph(known_graph_cpy, {known_graph_cpy.getName()}))
    {
      throw CREATE_TEMOTO_ERROR_STACK("Could not resolve graph '" + known_graph_cpy.getName() + "'");
    }

    umrf_graph_exec_map_.emplace(graph_name, std::make_shared<UmrfGraphExec>(known_graph_cpy));
  };

  if (!graphExists(graph_name))
  {
    insertIndexedGraph(graph_name);
  }
  else if (umrf_graph_exec_map_.at(graph_name)->getState() == UmrfGraphExec::State::FINISHED)
  {
    finished_graphs_.erase(graph_name);
    umrf_graph_exec_map_.erase(graph_name);
    insertIndexedGraph(graph_name);
  }
  else if (umrf_graph_exec_map_.at(graph_name)->getState() != UmrfGraphExec::State::UNINITIALIZED)
  {
    throw CREATE_TEMOTO_ERROR_STACK("Cannot execute UMRF graph '" + graph_name + "' because it's already active.");
  }

  // Synchronize the execution of the graph with other actors
  auto actors = umrf_graph_exec_map_.at(graph_name)->getActors();
  actors.erase(actor_name_);

  if (!actors.empty())
  {
    if(!synchronizerAvailable())
    {
      throw CREATE_TEMOTO_ERROR_STACK("Cannot execute UMRF graph '" + graph_name + "': actor synchronizer not initialized.");
    }

    if (!as_->bidirHandshake(graph_name, actors, 5000))
    {
      throw CREATE_TEMOTO_ERROR_STACK("Cannot execute UMRF graph '" + graph_name + "': failed to reach other actors.");
    }

    TEMOTO_PRINT_OF("Handshake successful", actor_name_);
  }

  umrf_graph_exec_map_.at(graph_name)->startGraph(result, params);
}
catch(TemotoErrorStack e)
{
  TEMOTO_PRINT_OF(e.what(), actor_name_);
  stopUmrfGraph(graph_name);
  throw FORWARD_TEMOTO_ERROR_STACK(e);
}

// void ActionEngine::modifyGraph(const std::string& graph_name, const UmrfGraphDiffs& graph_diffs)
// {
//   LOCK_GUARD_TYPE_R guard_graph_map_(umrf_graph_map_rw_mutex_);

//   if (!graphExists(graph_name))
//   {
//     TEMOTO_PRINT("Cannot modify graph '" + graph_name + "' because it does not exist.");
//     return;
//   }

//   TEMOTO_PRINT("Received a request to modify UMRF graph '" + graph_name + "' ...");
//   std::shared_ptr<UmrfGraphExec> ugh = umrf_graph_exec_map_.at(graph_name);

//   /*
//    * Before modyfing the graph, check if the graph contains the UMRFs which are menitoned in the diffs
//    */
//   for (const auto& graph_diff : graph_diffs)
//   {
//     if (graph_diff.operation == UmrfGraphDiff::Operation::add_umrf)
//     {
//       if (ugh->partOfGraph(graph_diff.umrf_node.getFullName()))
//       {
//         throw CREATE_TEMOTO_ERROR_STACK("Cannot add UMRF '" + graph_diff.umrf_node.getFullName()
//           + "', as it is already part of graph '" + graph_name + "'");
//       }
//     }
//     else
//     {
//       if (!ugh->partOfGraph(graph_diff.umrf_node.getFullName()))
//       {
//         throw CREATE_TEMOTO_ERROR_STACK("Cannot perform operation '" + graph_diff.operation
//         + "' because UMRF graph '" + graph_name + "' does not contain node named '"
//         + graph_diff.umrf_node.getFullName() + "'");
//       }
//     }
//   }

//   /*
//    * Apply the diffs
//    */ 
//   for (const auto& graph_diff : graph_diffs)
//   {
//     if (graph_diff.operation == UmrfGraphDiff::Operation::add_umrf)
//     {
//       TEMOTO_PRINT("Applying an '" + graph_diff.operation + "' operation to UMRF '" + graph_diff.umrf_node.getFullName() + "' ...");
//       ugh->addUmrf(graph_diff.umrf_node);
//     }
//     else if (graph_diff.operation == UmrfGraphDiff::Operation::remove_umrf)
//     {
//       TEMOTO_PRINT("Applying an '" + graph_diff.operation + "' operation to UMRF '" + graph_diff.umrf_node.getFullName() + "' ...");
//       ugh->stopNode(graph_diff.umrf_node.getFullName());
//       ugh->removeUmrf(graph_diff.umrf_node);
//     }
//     else if (graph_diff.operation == UmrfGraphDiff::Operation::add_child)
//     {
//       TEMOTO_PRINT("Applying an '" + graph_diff.operation + "' operation to UMRF '" + graph_diff.umrf_node.getFullName() + "' ...");
//       ugh->addChild(graph_diff.umrf_node);
//     }
//     else if (graph_diff.operation == UmrfGraphDiff::Operation::remove_child)
//     {
//       TEMOTO_PRINT("Applying an '" + graph_diff.operation + "' operation to UMRF '" + graph_diff.umrf_node.getFullName() + "' ...");
//       ugh->removeChild(graph_diff.umrf_node);
//     }
//     else
//     {
//       throw CREATE_TEMOTO_ERROR_STACK("No such operation as " + graph_diff.operation);
//     }
//     TEMOTO_PRINT("Finished with the '" + graph_diff.operation + "' operation.");
//   }
// }

void ActionEngine::stopUmrfGraph(const std::string& umrf_graph_name)
{
  std::unique_lock<std::recursive_mutex> guard_graph_map(umrf_graph_map_rw_mutex_);

  // Check if the requested graph exists
  if (umrf_graph_exec_map_.find(umrf_graph_name) == umrf_graph_exec_map_.end())
  {
    throw CREATE_TEMOTO_ERROR_STACK("Cannot stop UMRF graph '" + umrf_graph_name + "' because it doesn't exist.");
  }

  try
  {
    umrf_graph_exec_map_.at(umrf_graph_name)->stopGraph();
  }
  catch (TemotoErrorStack e)
  {
    throw FORWARD_TEMOTO_ERROR_STACK(e);
  }
  catch (std::exception& e)
  {
    throw CREATE_TEMOTO_ERROR_STACK(e.what());
  }

  guard_graph_map.unlock();
  onStateChange("", umrf_graph_name);

  guard_graph_map.lock();
  umrf_graph_exec_map_.erase(umrf_graph_name);
}

bool ActionEngine::addActionsPath(const std::string& action_packages_path)
try
{
  if (ai_.containsActions(action_packages_path))
  {
    ai_.addActionPath(action_packages_path);
    return true;
  }
  return false;
}
catch(TemotoErrorStack e)
{
  throw FORWARD_TEMOTO_ERROR_STACK(e);
}

bool ActionEngine::stop()
{
  LOCK_GUARD_TYPE_R guard_graph_map_(umrf_graph_map_rw_mutex_);

  // Stop all actions
  for (auto& umrf_graph : umrf_graph_exec_map_)
  {
    TEMOTO_PRINT_OF("Stopping umrf graph " + umrf_graph.second->getName(), actor_name_);
    umrf_graph.second->stopGraph();
  }

  // Clear all actions
  for (auto& umrf_graph : umrf_graph_exec_map_)
  {
    TEMOTO_PRINT_OF("Clearing umrf graph " + umrf_graph.second->getName(), actor_name_);
    umrf_graph.second->clearGraph();
  }

  TEMOTO_PRINT_OF("Removing all umrf graphs", actor_name_);
  umrf_graph_exec_map_.clear();

  TEMOTO_PRINT_OF("Action Engine is stopped", actor_name_);
  return true;
}

std::vector<std::string> ActionEngine::getGraphJsonsRunning() const
try
{
  LOCK_GUARD_TYPE_R guard_graph_map_(umrf_graph_map_rw_mutex_);
  std::vector<std::string> umrf_graph_jsons;

  for (const auto& umrf_graph_exec : umrf_graph_exec_map_)
  {
    std::string umrf_graph_json = umrf_json::toUmrfGraphJsonStr(umrf_graph_exec.second->toUmrfGraphCommon());
    umrf_graph_jsons.push_back(umrf_graph_json);
  }

  return umrf_graph_jsons;
}
catch(TemotoErrorStack e)
{
  throw FORWARD_TEMOTO_ERROR_STACK(e);
}

std::vector<std::string> ActionEngine::getGraphJsonsIndexed() const
try
{
  std::vector<std::string> umrf_graph_jsons;

  for (const auto& indexed_graph : ai_.getGraphs())
  {
    std::string umrf_graph_json = umrf_json::toUmrfGraphJsonStr(indexed_graph);
    umrf_graph_jsons.push_back(umrf_graph_json);
  }

  return umrf_graph_jsons;
}
catch(TemotoErrorStack e)
{
  throw FORWARD_TEMOTO_ERROR_STACK(e);
}

std::vector<std::string> ActionEngine::getUmrfJsons() const
try
{
  std::vector<std::string> umrf_jsons;

  for (const auto& u : ai_.getUmrfs())
  {
    umrf_jsons.push_back(umrf_json::toUmrfJsonStr(u));
  }

  return umrf_jsons;
}
catch(TemotoErrorStack e)
{
  throw FORWARD_TEMOTO_ERROR_STACK(e);
}

void ActionEngine::acknowledge(const std::string& token)
{
  if (synchronizerAvailable()) /* TODO: prolly an error should be thrown, TBD */
    as_->unidirHandshake(token);
}

void ActionEngine::notifyFinished(const Waitable& waitable
, const std::string& result
, const ActionParameters& params
, const std::string& token)
{
  LOCK_GUARD_TYPE l_sync(sync_map_rw_mutex_);

  if (waitable.action_name == GRAPH_EXIT.getFullName())
  {
    finished_graphs_.insert({waitable.graph_name, result});
    notify_cv_.notify_all();
  }

  if (sync_map_.find(waitable) != sync_map_.end())
  {
    LOCK_GUARD_TYPE_R l_graph(umrf_graph_map_rw_mutex_);
    // const auto& waitable_action = umrf_graph_exec_map_.at(waitable.graph_name)->graph_nodes_map_.at(waitable.action_name);

    // Notify local actions
    for (const auto& waiter : sync_map_.at(waitable))
    {
      auto& waiting_action = umrf_graph_exec_map_.at(waiter.graph_name)->graph_nodes_map_.at(waiter.action_name);
      waiting_action->setRemoteResult(result);
      waiting_action->setOutputParameters(params);
      waiting_action->notifyFinished(token);
    }
  }

  // Notify remote acions
  if(waitable.actor_name == actor_name_)
  {
    auto actors{[&]
    {
      LOCK_GUARD_TYPE_R l_graph(umrf_graph_map_rw_mutex_);
      return umrf_graph_exec_map_.at(waitable.graph_name)->getActors();
    }()};

    actors.erase(actor_name_);

    if (synchronizerAvailable()) /* TODO: prolly an error should be thrown, TBD */
      if (!as_->sendNotify(waitable, result, params, actors, 5000))
        throw CREATE_TEMOTO_ERROR_STACK("The notification from '" + actor_name_ + "' did not reach all actors");
  }

  sync_map_.erase(waitable);
}

ActionEngine::~ActionEngine()
try
{
  stop();
}
catch(const std::exception& e)
{
  std::cerr << e.what() << '\n';
}

void ActionEngine::addWaiter(const Waitable& waitable, const Waiter& waiter)
{
  LOCK_GUARD_TYPE l(sync_map_rw_mutex_);

  if (sync_map_.find(waitable) == sync_map_.end())
  {
    sync_map_.insert({waitable, std::vector<Waiter>{waiter}});
    return;
  }

  // Make sure that it's not a duplicate entry
  for (const auto& other_waiter : sync_map_.at(waitable))
  {
    if (waiter == other_waiter)
    {
      throw CREATE_TEMOTO_ERROR_STACK("Attempted to add a duplicate entry to the wait list: waitable=['" 
      + waitable.action_name + "' in '" + waitable.graph_name + "'], waiter=['" 
      + waiter.action_name + "' in '" + waiter.graph_name + "']");
    }
  }

  sync_map_.at(waitable).push_back(waiter);
}

bool ActionEngine::matchGraph(UmrfGraph& g, std::set<std::string> g_blacklist)
{
  auto graph_nodes = g.getUmrfNodes();
  return [&]{
  for (auto& a : graph_nodes)
  {
    // ignore the entry and exit actions
    if(a.getName() == "graph_entry" || a.getName() == "graph_exit")
      continue;

    // ignore remote actions
    if (!a.getActor().empty() && a.getActor() != actor_name_)
    {
      a.setActorExecTraits(UmrfNode::ActorExecTraits::REMOTE);
      continue;
    }

    // look from atomic actions
    if (amf_.findMatchingAction(a, ai_.getUmrfs(), true))
    {
      continue;
    }

    bool is_subgraph = [&]
    {
      for (auto sub_g : ai_.getGraphs())
      {
        // Avoid self reference / self dependency
        if (g_blacklist.count(sub_g.getName()) != 0)
          continue;

        // Name of the subgraph must match the name of the desired action
        if (sub_g.getName() != a.getName())
          continue;

        // Get the signature of the subgraph (the configuration of input and output parameters)
        UmrfNode node_signature;
        node_signature.setInputParameters(sub_g.getUmrfNode(GRAPH_ENTRY.getFullName())->getInputParameters());
        node_signature.setOutputParameters(sub_g.getUmrfNode(GRAPH_EXIT.getFullName())->getOutputParameters());

        if (!amf_.findMatchingAction(a, node_signature))
          continue;

        // Find matching actions/graphs for the sub-graph
        g_blacklist.insert(sub_g.getName());
        if (matchGraph(sub_g, g_blacklist))
          return true;
      }
      return false;
    }();

    if (!is_subgraph)
    {
      return false;
    }
    a.setActorExecTraits(UmrfNode::ActorExecTraits::GRAPH);
  }

  g = UmrfGraph(g.getName(), graph_nodes);
  return true;
  }();
}

std::string ActionEngine::waitForGraph(const std::string& graph_name)
{
  std::unique_lock<std::mutex> notify_cv_lock(notify_cv_mutex_);
  notify_cv_.wait(notify_cv_lock, [&]
  {
    LOCK_GUARD_TYPE_R l(umrf_graph_map_rw_mutex_);
    return umrf_graph_exec_map_.at(graph_name)->getState() == UmrfGraph::State::FINISHED;
  });

  return finished_graphs_.at(graph_name);
}

bool ActionEngine::synchronizerAvailable() const
{
  return as_ ? true : false;
}

void ActionEngine::pauseUmrfGraph(const std::string& umrf_graph_name)
{
  std::unique_lock<std::recursive_mutex> guard_graph_map(umrf_graph_map_rw_mutex_);

  // Check if the requested graph exists
  if (umrf_graph_exec_map_.find(umrf_graph_name) == umrf_graph_exec_map_.end())
  {
    throw CREATE_TEMOTO_ERROR_STACK("Cannot pause UMRF graph '" + umrf_graph_name + "' because it doesn't exist.");
  }

  try
  {
    umrf_graph_exec_map_.at(umrf_graph_name)->pauseGraph();
  }
  catch (TemotoErrorStack e)
  {
    throw FORWARD_TEMOTO_ERROR_STACK(e);
  }
  catch (std::exception& e)
  {
    throw CREATE_TEMOTO_ERROR_STACK(e.what());
  }
}

void ActionEngine::resumeUmrfGraph(const std::string& umrf_graph_name)
{
  std::unique_lock<std::recursive_mutex> guard_graph_map(umrf_graph_map_rw_mutex_);

  // Check if the requested graph exists
  if (umrf_graph_exec_map_.find(umrf_graph_name) == umrf_graph_exec_map_.end())
  {
    throw CREATE_TEMOTO_ERROR_STACK("Cannot resume UMRF graph '" + umrf_graph_name + "' because it doesn't exist.");
  }

  try
  {
    umrf_graph_exec_map_.at(umrf_graph_name)->resumeGraph();
  }
  catch (TemotoErrorStack e)
  {
    throw FORWARD_TEMOTO_ERROR_STACK(e);
  }
  catch (std::exception& e)
  {
    throw CREATE_TEMOTO_ERROR_STACK(e.what());
  }
}
