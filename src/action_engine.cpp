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

#include "temoto_action_engine/action_engine.h"
#include "temoto_action_engine/messaging.h"
#include "temoto_action_engine/umrf_json.h"

ActionEngine::ActionEngine(const std::string& actor_name, std::vector<std::string> sync_plugin_names)
: actor_name_(actor_name)
, stop_monitoring_thread_(false)
{
  ENGINE_HANDLE.execute_graph_fptr_ = std::bind(&ActionEngine::executeUmrfGraph, this
  , std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

  ENGINE_HANDLE.notify_finished_fptr_ = std::bind(&ActionEngine::notifyFinished, this
  , std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

  ENGINE_HANDLE.add_waiter_fptr_ = std::bind(&ActionEngine::addWaiter, this
  , std::placeholders::_1, std::placeholders::_2);

  if (!sync_plugin_names.empty())
  {
    as_ = std::make_unique<ActionSynchronizer>(sync_plugin_names);
  }

  //start();
}

void ActionEngine::start()
{
  monitoring_thread_ = std::thread(&ActionEngine::monitoringLoop, this);
}

void ActionEngine::monitoringLoop()
{
  while (!stop_monitoring_thread_)
  {
    // Wait until a thread is finished
    std::unique_lock<std::mutex> notify_cv_lock(notify_cv_mutex_);
    notify_cv_.wait(notify_cv_lock, [&]{return !finished_graphs_.empty() || stop_monitoring_thread_;});

    if (stop_monitoring_thread_)
    {
      break;
    }

    LOCK_GUARD_TYPE_R guard_graph_nodes_map_(umrf_graph_map_rw_mutex_);
    for (const auto& finished_graph : finished_graphs_)
    try
    {
      /* TODO: notify*/

      TEMOTO_PRINT("Clearing umrf graph '" + finished_graph.first + "'");
      umrf_graph_exec_map_.erase(finished_graph.first);
    }
    catch(TemotoErrorStack e)
    {
      std::cout << e.what() << '\n';
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }
    catch(...)
    {
      std::cerr << "[" << __func__ << "] caught an unhandled exception" << '\n';
    }
    finished_graphs_.clear();
  }
}

bool ActionEngine::graphExists(const std::string& graph_name) const
{
  LOCK_GUARD_TYPE_R guard_graph_nodes_map_(umrf_graph_map_rw_mutex_);
  return (umrf_graph_exec_map_.find(graph_name) != umrf_graph_exec_map_.end());
}

void ActionEngine::addUmrfGraph(const std::string& graph_name, const std::vector<UmrfNode>& umrf_nodes)
{
  LOCK_GUARD_TYPE_R guard_graph_nodes_map_(umrf_graph_map_rw_mutex_);
  umrf_graph_exec_map_.emplace(graph_name, std::make_shared<UmrfGraphExec>(graph_name, umrf_nodes));
}

void ActionEngine::executeUmrfGraphA(UmrfGraph umrf_graph, const std::string& result, bool name_match_required)
try
{
  if (graphExists(umrf_graph.getName()))
  {
    TEMOTO_PRINT("UMRF graph '" + umrf_graph.getName() + "' is already running.");
    return;
  }

  if (!matchGraph(umrf_graph, {umrf_graph.getName()}))
  {
    throw CREATE_TEMOTO_ERROR_STACK("Could not resolve graph '" + umrf_graph.getName() + "'");
  }
  TEMOTO_PRINT("All actions in graph '" + umrf_graph.getName() + "' found.");

  umrf_graph_exec_map_.emplace(umrf_graph.getName(), std::make_shared<UmrfGraphExec>(umrf_graph));
  TEMOTO_PRINT("UMRF graph '" + umrf_graph.getName() + "' initialized.");

  executeUmrfGraph(umrf_graph.getName(), ActionParameters{}, result);
  TEMOTO_PRINT("UMRF graph '" + umrf_graph.getName() + "' invoked successfully.");
}
catch(TemotoErrorStack e)
{
  throw FORWARD_TEMOTO_ERROR_STACK(e);
}

void ActionEngine::executeUmrfGraph(const std::string& graph_name, const ActionParameters& params, const std::string& result)
try
{
  LOCK_GUARD_TYPE_R guard_graph_nodes_map_(umrf_graph_map_rw_mutex_);

  // Check if the requested graph exists
  if (!graphExists(graph_name))
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
  }

  // Check if the graph is in initialized state
  if (umrf_graph_exec_map_.at(graph_name)->getState() != UmrfGraphExec::State::INITIALIZED)
  {
    throw CREATE_TEMOTO_ERROR_STACK("Cannot execute UMRF graph '" + graph_name + "' because it's not in initialized state.");
  }
  else
  {
    umrf_graph_exec_map_.at(graph_name)->startGraph(result, params);
  }
}
catch(TemotoErrorStack e)
{
  TEMOTO_PRINT(e.what());
  stopUmrfGraph(graph_name);
  throw FORWARD_TEMOTO_ERROR_STACK(e);
}


void ActionEngine::modifyGraph(const std::string& graph_name, const UmrfGraphDiffs& graph_diffs)
{
  LOCK_GUARD_TYPE_R guard_graph_nodes_map_(umrf_graph_map_rw_mutex_);

  if (!graphExists(graph_name))
  {
    TEMOTO_PRINT("Cannot modify graph '" + graph_name + "' because it does not exist.");
    return;
  }

  TEMOTO_PRINT("Received a request to modify UMRF graph '" + graph_name + "' ...");
  std::shared_ptr<UmrfGraphExec> ugh = umrf_graph_exec_map_.at(graph_name);

  /*
   * Before modyfing the graph, check if the graph contains the UMRFs which are menitoned in the diffs
   */ 
  for (const auto& graph_diff : graph_diffs)
  {
    if (graph_diff.operation == UmrfGraphDiff::Operation::add_umrf)
    {
      if (ugh->partOfGraph(graph_diff.umrf_node.getFullName()))
      {
        throw CREATE_TEMOTO_ERROR_STACK("Cannot add UMRF '" + graph_diff.umrf_node.getFullName()
          + "', as it is already part of graph '" + graph_name + "'");
      }
    }
    else
    {
      if (!ugh->partOfGraph(graph_diff.umrf_node.getFullName()))
      {
        throw CREATE_TEMOTO_ERROR_STACK("Cannot perform operation '" + graph_diff.operation
        + "' because UMRF graph '" + graph_name + "' does not contain node named '"
        + graph_diff.umrf_node.getFullName() + "'");
      }
    }
  }

  /*
   * Apply the diffs
   */ 
  for (const auto& graph_diff : graph_diffs)
  {
    if (graph_diff.operation == UmrfGraphDiff::Operation::add_umrf)
    {
      TEMOTO_PRINT("Applying an '" + graph_diff.operation + "' operation to UMRF '" + graph_diff.umrf_node.getFullName() + "' ...");
      ugh->addUmrf(graph_diff.umrf_node);
    }
    else if (graph_diff.operation == UmrfGraphDiff::Operation::remove_umrf)
    {
      TEMOTO_PRINT("Applying an '" + graph_diff.operation + "' operation to UMRF '" + graph_diff.umrf_node.getFullName() + "' ...");
      ugh->stopNode(graph_diff.umrf_node.getFullName());
      ugh->removeUmrf(graph_diff.umrf_node);
    }
    else if (graph_diff.operation == UmrfGraphDiff::Operation::add_child)
    {
      TEMOTO_PRINT("Applying an '" + graph_diff.operation + "' operation to UMRF '" + graph_diff.umrf_node.getFullName() + "' ...");
      ugh->addChild(graph_diff.umrf_node);
    }
    else if (graph_diff.operation == UmrfGraphDiff::Operation::remove_child)
    {
      TEMOTO_PRINT("Applying an '" + graph_diff.operation + "' operation to UMRF '" + graph_diff.umrf_node.getFullName() + "' ...");
      ugh->removeChild(graph_diff.umrf_node);
    }
    else
    {
      throw CREATE_TEMOTO_ERROR_STACK("No such operation as " + graph_diff.operation);
    }
    TEMOTO_PRINT("Finished with the '" + graph_diff.operation + "' operation.");
  }
}

void ActionEngine::stopUmrfGraph(const std::string& umrf_graph_name)
{
  LOCK_GUARD_TYPE_R guard_graph_nodes_map_(umrf_graph_map_rw_mutex_);

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
  umrf_graph_exec_map_.erase(umrf_graph_name);
}

bool ActionEngine::addActionsPath(const std::string& action_packages_path)
try
{
  if (ai_.containsActions(action_packages_path))
  {
    ai_.addActionPath(action_packages_path); 
    auto summary = ai_.indexActions();
    TEMOTO_PRINT("Found '" + std::to_string(summary.actions) + "' actions and '" + std::to_string(summary.graphs) + "' graphs");
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
  LOCK_GUARD_TYPE_R guard_graph_nodes_map_(umrf_graph_map_rw_mutex_);

  // Stop all actions
  for (auto& umrf_graph : umrf_graph_exec_map_)
  {
    TEMOTO_PRINT("Stopping umrf graph " + umrf_graph.second->getName());
    umrf_graph.second->stopGraph();
  }

  // Clear all actions
  for (auto& umrf_graph : umrf_graph_exec_map_)
  {
    TEMOTO_PRINT("Clearing umrf graph " + umrf_graph.second->getName());
    umrf_graph.second->clearGraph();
  }

  TEMOTO_PRINT("Removing all umrf graphs");
  umrf_graph_exec_map_.clear();

  // // Stop the monitoring thread
  // stop_monitoring_thread_ = true;
  // notify_cv_.notify_all();
  // monitoring_thread_.join();

  TEMOTO_PRINT("Action Engine is stopped");
  return true;
}

std::vector<std::string> ActionEngine::getGraphJsons() const
{
  LOCK_GUARD_TYPE_R guard_graph_nodes_map_(umrf_graph_map_rw_mutex_);

  std::vector<std::string> umrf_graph_jsons;
  try
  {
    for (const auto& umrf_graph_exec : umrf_graph_exec_map_)
    {
      std::string umrf_graph_json = umrf_json::toUmrfGraphJsonStr(umrf_graph_exec.second->toUmrgGraphCommon()); 
      umrf_graph_jsons.push_back(umrf_graph_json);     
    }
  }
  catch(TemotoErrorStack e)
  {
    throw FORWARD_TEMOTO_ERROR_STACK(e);
  }
  return umrf_graph_jsons;
}

void ActionEngine::notifyFinished(const Waitable& waitable, const std::string& result, const ActionParameters& params)
{
  LOCK_GUARD_TYPE l_sync(sync_map_rw_mutex_);

  if (waitable.action_name == GRAPH_EXIT.getFullName())
  {
    finished_graphs_.insert({waitable.graph_name, result});
    notify_cv_.notify_all();
  }

  if (sync_map_.find(waitable) == sync_map_.end())
  {
    return;
  }

  LOCK_GUARD_TYPE_R l_graph(umrf_graph_map_rw_mutex_);
  const auto& waitable_action = umrf_graph_exec_map_.at(waitable.graph_name)->graph_nodes_map_.at(waitable.action_name);

  // Notify local actions
  for (const auto& waiter : sync_map_.at(waitable))
  {
    auto& waiting_action = umrf_graph_exec_map_.at(waiter.graph_name)->graph_nodes_map_.at(waiter.action_name);
    waiting_action->setRemoteResult(result);
    waiting_action->setOutputParameters(params);
    waiting_action->notifyFinished();
  }

  // Notify remote acions
  if(waitable.actor_name == actor_name_)
  {
    as_->sendNotify(waitable, result, params);
  }

  // TODO: for shared actions, erase when all acks are received
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
  as_ ? true : false;
}