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

/* Author: Robert Valner */

#ifndef TEMOTO_ACTION_ENGINE__ACTION_EXECUTOR_TPP
#define TEMOTO_ACTION_ENGINE__ACTION_EXECUTOR_TPP

#include "temoto_action_engine/action_executor.h"
#include "temoto_action_engine/temoto_error.h"
#include <set>
 
ActionExecutor::ActionExecutor()
{
  startCleanupLoopThread();
}

void ActionExecutor::notifyFinished(const unsigned int& parent_action_id, const ActionParameters& parent_action_parameters)
{
  LOCK_GUARD_TYPE_R guard_handles(named_action_handles_rw_mutex_);
  LOCK_GUARD_TYPE_R guard_graphs(named_umrf_graphs_rw_mutex_);
  /*
   * Check the UMRF graphs and execute sequential actions if necessary
   */
  try
  {
    for (auto& umrf_graph_pair : named_umrf_graphs_)
    {
      if ((umrf_graph_pair.second.checkState() != UmrfGraphHelper::State::ACTIVE) ||
          (umrf_graph_pair.second.getChildrenOf(parent_action_id).empty()))
      {
        continue;
      }

      /*
       * Transfer the parameters from parent to child action
       */ 
      for (const auto& child_id : umrf_graph_pair.second.getChildrenOf(parent_action_id))
      {
        Umrf& child_umrf = umrf_graph_pair.second.getUmrfOfNonconst(child_id);
        child_umrf.copyInputParameters(parent_action_parameters);
      }
      executeById(umrf_graph_pair.second.getChildrenOf(parent_action_id), umrf_graph_pair.second);
    }
  }
  catch(TemotoErrorStack e)
  {
    throw FORWARD_TEMOTO_ERROR_STACK(e);
  }
  catch(const std::exception& e)
  {
    throw CREATE_TEMOTO_ERROR_STACK(std::string(e.what()));
  }
}

template<typename T>
bool ActionExecutor::futureIsReady(const std::future<T>& t)
{
  return t.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}

bool ActionExecutor::isActive() const
{
  LOCK_GUARD_TYPE_R guard_handles(named_action_handles_rw_mutex_);
  for (const auto& named_action_handle : named_action_handles_)
  {
    if (named_action_handle.second.getState() == ActionHandle::State::RUNNING)
    {
      return true;
    }
  }
  return false;
}

unsigned int ActionExecutor::getActionCount() const
{
  LOCK_GUARD_TYPE_R guard_handles(named_action_handles_rw_mutex_);
  return named_action_handles_.size();
}

bool ActionExecutor::stopAndCleanUp()
{
  named_action_handles_rw_mutex_.lock();
  // Stop all actions
  for (auto& named_action_handle : named_action_handles_)
  {
    TEMOTO_PRINT("Stopping action " + named_action_handle.second.getActionName());
    named_action_handle.second.stopAction(4);
  }
  named_action_handles_rw_mutex_.unlock();

  // Wait until all actions are stopped
  TEMOTO_PRINT("Waiting for all actions to stop ...");
  while (isActive())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // Stop the cleanup loop
  TEMOTO_PRINT("Stopping the cleanup loop ...");
  cleanup_loop_spinning_ = false;
  while (cleanup_loop_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  TEMOTO_PRINT("Action Executor is stopped.");

  return true;
}

bool ActionExecutor::startCleanupLoopThread()
{
  cleanup_loop_spinning_ = true;
  cleanup_loop_future_ = std::async( std::launch::async
                                   , &ActionExecutor::cleanupLoop
                                   , this);
  return true;
}

void ActionExecutor::cleanupLoop()
{
  while(cleanup_loop_spinning_)
  {
    {
      LOCK_GUARD_TYPE_R guard_handles(named_action_handles_rw_mutex_);
      if (!named_action_handles_.empty())
      {
        for ( auto nah_it=named_action_handles_.begin()
            ; nah_it!=named_action_handles_.end()
            ; /* empty */)
        {
          /*
           * TODO: Handle actions that have reached to error state
           */
          if (/*(nah_it->second.getState() == ActionHandle::State::FINISHED) &&*/
              nah_it->second.futureIsReady() &&
              nah_it->second.getEffect() == "synchronous")
          {
            try
            {
              std::string error_message = nah_it->second.getFutureValue().getMessage();
              if (!error_message.empty())
              {
                std::cout << error_message << std::endl;
              }
              nah_it->second.clearAction();
              named_action_handles_.erase(nah_it++);
            }
            catch(TemotoErrorStack e)
            {
              std::cout << e.what() << '\n';
              ++nah_it;
            }
          }
          else
          {
            ++nah_it;
          }
        }
      }
      else
      {
        TEMOTO_PRINT("no actions in executing state");
      }
    } // Lock guard scope
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }
}

void ActionExecutor::addUmrfGraph(const std::string& graph_name, std::vector<Umrf> umrf_jsons_vec)
{
  try
  {
    // Give each UMRF a unique ID
    for (auto& umrf_json : umrf_jsons_vec)
    {
      umrf_json.setId(createId());
    }

    // Create an UMRF graph
    UmrfGraphHelper ugh = UmrfGraphHelper(graph_name, umrf_jsons_vec);
    if (ugh.checkState() == UmrfGraphHelper::State::UNINITIALIZED)
    {
      throw CREATE_TEMOTO_ERROR_STACK("Cannot add UMRF graph because it's uninitialized.");
    }

    LOCK_GUARD_TYPE_R guard_graphs(named_umrf_graphs_rw_mutex_);
    named_umrf_graphs_.insert(std::pair<std::string, UmrfGraphHelper>(graph_name, ugh));
  }
  catch(TemotoErrorStack e)
  {
    throw FORWARD_TEMOTO_ERROR_STACK(e);
  }
}

void ActionExecutor::executeUmrfGraph(const std::string& graph_name)
{
  LOCK_GUARD_TYPE_R guard_graphs(named_umrf_graphs_rw_mutex_);
  LOCK_GUARD_TYPE_R guard_handles(named_action_handles_rw_mutex_);

  try
  {
    // Check if the requested graph exists
    if (named_umrf_graphs_.find(graph_name) == named_umrf_graphs_.end())
    {
      throw CREATE_TEMOTO_ERROR_STACK("Cannot execute UMRF graph '" + graph_name + "' because it doesn't exist.");
    }

    // Check if the graph is in initialized state
    if (named_umrf_graphs_.at(graph_name).checkState() != UmrfGraphHelper::State::INITIALIZED)
    {
      throw CREATE_TEMOTO_ERROR_STACK("Cannot execute UMRF graph '" + graph_name + "' because it's not in initialized state.");
    }

    UmrfGraphHelper& ugh = named_umrf_graphs_.at(graph_name);
    std::vector<unsigned int> action_ids = ugh.getRootNodes();

    executeById(action_ids, ugh, true);
  }
  catch(TemotoErrorStack e)
  {
    throw FORWARD_TEMOTO_ERROR_STACK(e);
  }
}

void ActionExecutor::stopUmrfGraph(const std::string& graph_name)
{
  LOCK_GUARD_TYPE_R guard_graphs(named_umrf_graphs_rw_mutex_);
  LOCK_GUARD_TYPE_R guard_handles(named_action_handles_rw_mutex_);

  // Check if the requested graph exists
  if (named_umrf_graphs_.find(graph_name) == named_umrf_graphs_.end())
  {
    throw CREATE_TEMOTO_ERROR_STACK("Cannot stop UMRF graph '" + graph_name + "' because it doesn't exist.");
  }

  for (const auto& umrf : named_umrf_graphs_.at(graph_name).getUmrfs())
  {
    auto action_handle_it = named_action_handles_.find(umrf.getId());
    if (action_handle_it == named_action_handles_.end())
    {
      continue;
    }
    try
    {
      action_handle_it->second.clearAction();
      named_action_handles_.erase(action_handle_it);
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
}

void ActionExecutor::executeById(const std::vector<unsigned int> ids, UmrfGraphHelper& ugh, bool initialized_requrired)
{
  LOCK_GUARD_TYPE_R guard_graphs(named_umrf_graphs_rw_mutex_);
  LOCK_GUARD_TYPE_R guard_handles(named_action_handles_rw_mutex_);

  std::set<unsigned int> action_rollback_list;
  std::set<unsigned int> init_action_ids;
  try
  {
    /*
     * Load the action handles
     */ 
    HandleMap named_action_handles_tmp;
    for (const auto& action_id : ids)
    {
      ActionHandle ah = ActionHandle(ugh.getUmrfOf(action_id), this);
      if (ah.getState() != ActionHandle::State::INITIALIZED)
      {
        if (initialized_requrired)
        {
          throw CREATE_TEMOTO_ERROR_STACK("Cannot execute the actions because all actions were not fully initialized.");
        }
      }
      else
      {
        named_action_handles_tmp.insert(std::pair<unsigned int, ActionHandle>(ah.getHandleId(), ah));
        init_action_ids.insert(ah.getHandleId());
        action_rollback_list.insert(ah.getHandleId());
      }
    }

    // put all loaded action handles to the global named_action_handles map
    named_action_handles_.insert(named_action_handles_tmp.begin(), named_action_handles_tmp.end());

    /*
     * Load/Instantiate the actions. If there are any problems with instantiating
     * the graph, then the whole graph is rolled back (uninitialized)
     */
    for (const auto& action_id : init_action_ids)
    {
      // Instantiate the action
      try
      {
        named_action_handles_.at(action_id).instantiateAction();
        action_rollback_list.insert(action_id);
      }
      catch(TemotoErrorStack e)
      {
        ugh.setNodeError(action_id);
        throw FORWARD_TEMOTO_ERROR_STACK(e);
      } 
      catch(const std::exception& e)
      {
        ugh.setNodeError(action_id);
        throw CREATE_TEMOTO_ERROR_STACK("Cannot initialize the actions because: " 
          + std::string(e.what()));
      }
    }

    /*
     * Execute the actions. If there are any problems with executing
     * the graph, then the whole graph is rolled back
     */
    for (const auto& action_id : init_action_ids)
    {
      // Execute the action
      try
      {
        named_action_handles_.at(action_id).executeActionThread();
        action_rollback_list.insert(action_id);
        ugh.setNodeActive(action_id);
      }
      catch(TemotoErrorStack e)
      {
        ugh.setNodeError(action_id);
        throw FORWARD_TEMOTO_ERROR_STACK(e);
      } 
      catch(const std::exception& e)
      {
        ugh.setNodeError(action_id);
        throw CREATE_TEMOTO_ERROR_STACK("Cannot execute the actions because: " 
          + std::string(e.what()));
      }
    }
  }
  catch(TemotoErrorStack e)
  {
    std::cout << "Rollbacking actions" << std::endl;
    for (const auto& action_id : action_rollback_list)
    {
      named_action_handles_.at(action_id).clearAction();
      named_action_handles_.erase(action_id);
      ugh.setNodeFinished(action_id);
    }
    throw FORWARD_TEMOTO_ERROR_STACK(e);
  }
}

unsigned int ActionExecutor::createId()
{
  return action_handle_id_count_++;
}

#endif