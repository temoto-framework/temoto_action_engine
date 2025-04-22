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

  ENGINE_HANDLE.execute_graph_fptr_ = std::bind(&ActionEngine::startGraph, this
  , std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);

  ENGINE_HANDLE.stop_graph_fptr_ = std::bind(&ActionEngine::stopGraph, this
  , std::placeholders::_1);

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
    std::string jsonstr;
    {
      LOCK_GUARD_TYPE_R guard_graphs_map_(umrf_graph_map_rw_mutex_);
      jsonstr = umrf_json::toUmrfGraphJsonStr(umrf_graph_exec_map_.at(graph_name)->toUmrfGraphCommon());
    }
    return jsonstr;
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

void ActionEngine::addGraph(const std::string& graph_name, const std::vector<UmrfNode>& umrf_nodes)
{
  LOCK_GUARD_TYPE_R guard_graph_map_(umrf_graph_map_rw_mutex_);
  umrf_graph_exec_map_.emplace(graph_name, std::make_shared<UmrfGraphExec>(graph_name, umrf_nodes));
}

void ActionEngine::startGraphA(UmrfGraph umrf_graph, const std::string& result, [[maybe_unused]] bool name_match_required)
try
{
  LOCK_GUARD_TYPE_R guard_graph_map_(umrf_graph_map_rw_mutex_);

  const auto& graph_name = umrf_graph.getName();

  if (!graphExists(graph_name))
  {
    /*
     * Make sure that the name of the new graph is unique, i.e., is not contained in the indexed graphs
     */
    for (const auto& ig : ai_.getGraphs())
    {
      if (ig.getName() == graph_name)
      {
        throw CREATE_TEMOTO_ERROR_STACK("Cannot execute UMRF graph '" + graph_name + "' because it conflicts with an indexed graph (graph name not unique).");
      }
    }

    /*
     * Check if all the actions within the graph exist
     */
    if (!matchGraph(umrf_graph, {graph_name}))
    {
      throw CREATE_TEMOTO_ERROR_STACK("Could not resolve graph '" + graph_name + "'");
    }

    TEMOTO_PRINT("All actions in graph '" + graph_name + "' found.");
  }
  else if (
    [&]{
      const auto& s{umrf_graph_exec_map_.at(graph_name)->getState()};
      return s != UmrfGraphExec::State::RUNNING  &&
             s != UmrfGraphExec::State::PAUSED   &&
             s != UmrfGraphExec::State::STOPPING &&
             s != UmrfGraphExec::State::CLEARING;
    }())
  {
    TEMOTO_PRINT("Restarting graph '" + graph_name + "'.");

    if (finished_graphs_.find(graph_name) != finished_graphs_.end())
    {
      finished_graphs_.erase(graph_name);
    }

    umrf_graph_exec_map_.erase(graph_name);
  }

  umrf_graph_exec_map_.emplace(graph_name, std::make_shared<UmrfGraphExec>(umrf_graph));

  startGraph(graph_name, ActionParameters{}, result, {});
  TEMOTO_PRINT("UMRF graph '" + graph_name + "' invoked successfully.");
}
catch(TemotoErrorStack e)
{
  throw FORWARD_TEMOTO_ERROR_STACK(e);
}

void ActionEngine::startGraph(const std::string& graph_name
, const ActionParameters& params
, const std::string& result
, const std::string& graph_name_renamed)
try
{
  // std::string child_graph_name = parent_graph_name_ + "__" + getName() + "__" + std::to_str(getInstanceId());
  LOCK_GUARD_TYPE_R guard_graph_map_(umrf_graph_map_rw_mutex_);

  // Utilized for hierarchical graphs
  auto graph_name_final{graph_name_renamed.empty() ? graph_name : graph_name_renamed};

  auto insertIndexedGraph = [&]
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

    known_graph_cpy.setName(graph_name_final);

    umrf_graph_exec_map_.emplace(graph_name_final, std::make_shared<UmrfGraphExec>(known_graph_cpy));
  };

  if (!graphExists(graph_name_final))
  {
    insertIndexedGraph();
  }
  else if (umrf_graph_exec_map_.at(graph_name_final)->getState() != UmrfGraphExec::State::UNINITIALIZED)
  {
    const auto& s{umrf_graph_exec_map_.at(graph_name_final)->getState()};

    if (s != UmrfGraphExec::State::RUNNING  &&
        s != UmrfGraphExec::State::PAUSED   &&
        s != UmrfGraphExec::State::STOPPING &&
        s != UmrfGraphExec::State::CLEARING)
    {
      TEMOTO_PRINT("Restarting graph '" + graph_name_final + "'.");

      finished_graphs_.erase(graph_name_final);
      umrf_graph_exec_map_.erase(graph_name_final);
      insertIndexedGraph();
    }
    else
    {
      throw CREATE_TEMOTO_ERROR_STACK("Cannot execute UMRF graph '" + graph_name_final
        + "' because it's already active (" + UmrfGraphExec::state_to_str_map_.at(s) + ")");
    }
  }

  // Synchronize the execution of the graph with other actors
  auto actors = umrf_graph_exec_map_.at(graph_name_final)->getActors();
  actors.erase(actor_name_);

  if (!actors.empty())
  {
    if(!synchronizerAvailable())
    {
      throw CREATE_TEMOTO_ERROR_STACK("Cannot execute UMRF graph '" + graph_name_final + "': actor synchronizer not initialized.");
    }

    if (!as_->bidirHandshake(graph_name_final, actors, 5000))
    {
      throw CREATE_TEMOTO_ERROR_STACK("Cannot execute UMRF graph '" + graph_name_final + "': failed to reach other actors.");
    }

    TEMOTO_PRINT_OF("Handshake successful", actor_name_);
  }

  umrf_graph_exec_map_.at(graph_name_final)->startGraph(result, params);
}
catch(TemotoErrorStack e)
{
  TEMOTO_PRINT_OF(e.what(), actor_name_);
  stopGraph(graph_name);
  throw FORWARD_TEMOTO_ERROR_STACK(e);
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
  std::unique_lock<std::recursive_mutex> guard_graph_map(umrf_graph_map_rw_mutex_);

  // Stop all graphs
  std::vector<std::future<std::string>> graph_futures;
  for (auto& umrf_graph : umrf_graph_exec_map_)
  {
    TEMOTO_PRINT_OF("Stopping umrf graph " + umrf_graph.second->getName(), actor_name_);
    graph_futures.push_back(umrf_graph.second->stopGraph());
  }

  // Release the graph map mutex and wait until all graphs are stopped
  guard_graph_map.unlock();
  for (auto& future : graph_futures)
  try
  {
    future.get();
  }
  catch (const std::exception& ex)
  {
    std::cout << "Caught exception: " << ex.what() << std::endl;
  }

  // Clear all graphs
  guard_graph_map.lock();

  std::vector<std::future<bool>> graph_futures_clear;
  for (auto& umrf_graph : umrf_graph_exec_map_)
  {
    TEMOTO_PRINT_OF("Clearing umrf graph " + umrf_graph.second->getName(), actor_name_);
    graph_futures_clear.push_back(umrf_graph.second->clearGraph());
  }

  guard_graph_map.unlock();

  for (auto& future : graph_futures_clear)
  try
  {
    future.get();
  }
  catch (const std::exception& ex)
  {
    std::cout << "Caught exception: " << ex.what() << std::endl;
  }

  TEMOTO_PRINT_OF("Removing all umrf graphs", actor_name_);
  guard_graph_map.lock();
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

  if (waitable.action_name == GRAPH_EXIT.getFullName() || result == "on_halted")
  {
    finished_graphs_.insert({waitable.graph_name, result});
    notify_cv_.notify_all();
  }

  if (sync_map_.find(waitable) != sync_map_.end())
  {
    LOCK_GUARD_TYPE_R l_graph(umrf_graph_map_rw_mutex_);

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

    const auto& graph_state = umrf_graph_exec_map_.at(graph_name)->getState();
    return graph_state == UmrfGraph::State::FINISHED ||
           graph_state == UmrfGraph::State::STOPPED  ||
           graph_state == UmrfGraph::State::HALTED   ||
           graph_state == UmrfGraph::State::ERROR;
  });

  return finished_graphs_.at(graph_name);
}

bool ActionEngine::synchronizerAvailable() const
{
  return as_ ? true : false;
}

void ActionEngine::pauseGraph(const std::string& umrf_graph_name)
{
  if (!graphExists(umrf_graph_name))
  {
    throw CREATE_TEMOTO_ERROR_STACK("Cannot pause UMRF graph '" + umrf_graph_name + "' because it doesn't exist.");
  }

  try
  {
    std::unique_lock<std::recursive_mutex> guard_graph_map(umrf_graph_map_rw_mutex_);
    auto future{umrf_graph_exec_map_.at(umrf_graph_name)->pauseGraph()};

    // Unlock the mutex and wait until the graph is paused
    guard_graph_map.unlock();
    future.get();
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

void ActionEngine::resumeGraph(const std::string& umrf_graph_name, const std::string& continue_from)
{
  if (!graphExists(umrf_graph_name))
  {
    throw CREATE_TEMOTO_ERROR_STACK("Cannot resume UMRF graph '" + umrf_graph_name + "' because it doesn't exist.");
  }

  try
  {
    std::unique_lock<std::recursive_mutex> guard_graph_map(umrf_graph_map_rw_mutex_);
    umrf_graph_exec_map_.at(umrf_graph_name)->resumeGraph(continue_from);
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

std::string ActionEngine::stopGraph(const std::string& umrf_graph_name)
{
  std::unique_lock<std::recursive_mutex> guard_graph_map(umrf_graph_map_rw_mutex_);

  if (!graphExists(umrf_graph_name))
  {
    throw CREATE_TEMOTO_ERROR_STACK("Cannot stop UMRF graph '" + umrf_graph_name + "' because it doesn't exist.");
  }

  try
  {
    auto future{umrf_graph_exec_map_.at(umrf_graph_name)->stopGraph()};

    // Unlock the mutex and wait until the graph is stopped
    guard_graph_map.unlock();
    auto result{future.get()};
    onStateChange("", umrf_graph_name);

    return result;
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

void ActionEngine::modifyGraph(const UmrfGraph& graph_new, const std::string& continue_from)
try
{
  const std::string& graph_new_name = graph_new.getName();

  if (!graphExists(graph_new_name))
  {
    TEMOTO_PRINT("Cannot modify graph '" + graph_new_name + "' because it does not exist.");
    return;
  }

  // If the graph has already finished or halted, remove it from the list of finished graphs
  if (finished_graphs_.find(graph_new_name) != finished_graphs_.end())
  {
    finished_graphs_.erase(graph_new_name);
  }

  pauseGraph(graph_new_name);

  // Modify the graph
  {
    std::lock_guard<std::recursive_mutex> guard_graph_map(umrf_graph_map_rw_mutex_);
    umrf_graph_exec_map_.at(graph_new_name)->modifyGraph(graph_new);
  }

  resumeGraph(graph_new_name, continue_from);
}
catch (TemotoErrorStack e)
{
  throw FORWARD_TEMOTO_ERROR_STACK(e);
}
catch (std::exception& e)
{
  throw CREATE_TEMOTO_ERROR_STACK(e.what());
}
