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

#include "temoto_action_engine/action_indexer.h"
#include "temoto_action_engine/messaging.h"
#include "temoto_action_engine/temoto_error.h"
#include "temoto_action_engine/umrf_json.h"
#include <algorithm>
#include <sstream>
#include <fstream>

ActionIndexer::ActionIndexer()
{}

void ActionIndexer::addActionPath(const std::string& path)
{
  // Lock the mutex
  LOCK_GUARD_TYPE l(action_paths_mutex_);

  // Add the path if it does not exist
  if (std::find(action_paths_.begin(), action_paths_.end(), path) == action_paths_.end())
  {
    action_paths_.push_back(path);
  }
}

void ActionIndexer::addActionPath(const std::vector<std::string>& paths)
{
  for (const std::string& path : paths)
  {
    addActionPath(path);
  }
}

unsigned int ActionIndexer::indexActions()
try
{
  // Lock the mutex
  LOCK_GUARD_TYPE l_actions(action_sfs_mutex_);
  LOCK_GUARD_TYPE l_graphs(graphs_mutex_);
  LOCK_GUARD_TYPE l_paths(action_paths_mutex_);

  // Clear the old indexed umrfs frames
  indexed_umrfs_.clear();
  for (const std::string action_path : action_paths_)
  {
    boost::filesystem::directory_entry full_path_b = boost::filesystem::directory_entry(action_path);

    std::vector<UmrfNode> indexed_umrfs_local = findActionFilesys("", full_path_b, 2);
    indexed_umrfs_.insert(indexed_umrfs_.end(), indexed_umrfs_local.begin(), indexed_umrfs_local.end());

    std::vector<UmrfGraph> indexed_graphs_local = findGraphs(full_path_b, 2);
    indexed_graphs_.insert(indexed_graphs_.end(), indexed_graphs_local.begin(), indexed_graphs_local.end());
  }
  return indexed_umrfs_.size();
}
catch(TemotoErrorStack e)
{
  throw FORWARD_TEMOTO_ERROR_STACK(e);
}

unsigned int ActionIndexer::containsActions(const std::string& actions_path) const
{
  boost::filesystem::directory_entry full_path_b = boost::filesystem::directory_entry(actions_path);
  return findActionFilesys("", full_path_b, 2).size();
}

std::vector<UmrfNode> ActionIndexer::findActionFilesys( std::string action_to_find
, boost::filesystem::directory_entry base_path
, int search_depth) const
try
{
  boost::filesystem::path current_dir (base_path);
  boost::filesystem::directory_iterator end_itr;
  std::vector<UmrfNode> indexed_umrfs;

  // Start looking the files inside current directory
  for (boost::filesystem::directory_iterator itr( current_dir ); itr != end_itr; ++itr)
  {
    // if its a directory and depth limit is not there yet, go inside it
    if (boost::filesystem::is_directory(*itr) && (search_depth > 0))
    {
      std::vector<UmrfNode> indexed_umrfs_recursive{findActionFilesys( action_to_find, *itr, (search_depth - 1))};
      indexed_umrfs.insert(indexed_umrfs.end(), indexed_umrfs_recursive.begin(), indexed_umrfs_recursive.end());
    }

    // if its a file and matches the desc file name, process the file
    else if (boost::filesystem::is_regular_file(*itr) &&
              (itr->path().filename().string().find(umrf_file_extension_) != std::string::npos))
    try
    {
      std::string umrf_full_path = itr->path().string();
      std::ifstream ifs(umrf_full_path);
      std::string umrf_json_str;
      umrf_json_str.assign(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());
      UmrfNode umrf = umrf_json::fromUmrfJsonStr(umrf_json_str);
      indexed_umrfs.push_back(umrf);
    }
    catch(TemotoErrorStack e)
    {
      TEMOTO_PRINT(e.what());
      //throw FORWARD_TEMOTO_ERROR_STACK(e);
    }
  }
  return indexed_umrfs;
}
catch (std::exception& e)
{
  throw CREATE_TEMOTO_ERROR_STACK(e.what());
}
catch(...)
{
  // Rethrow the exception
  throw CREATE_TEMOTO_ERROR_STACK("Received an unhandled exception");
  // return std::vector<UmrfNode>{}; // TODO: suppressing the -Wreturn-type warning. Temoto error should be thrown via [[noreturn]] function 
}

std::vector<UmrfGraph> ActionIndexer::findGraphs(boost::filesystem::directory_entry base_path, int search_depth) const
try
{
  boost::filesystem::path current_dir (base_path);
  boost::filesystem::directory_iterator end_itr;
  std::vector<UmrfGraph> graphs;

  // Start looking the files inside current directory
  for (boost::filesystem::directory_iterator itr(current_dir); itr != end_itr; ++itr )
  {
    if (boost::filesystem::is_directory(*itr) && (search_depth > 0))
    {
      std::vector<UmrfGraph> graphs_recursive{findGraphs(*itr, search_depth - 1)};
      graphs.insert(graphs.end(), graphs_recursive.begin(), graphs_recursive.end());
    }

    else if (boost::filesystem::is_regular_file(*itr) &&
      (itr->path().filename().string().find("graph.json") != std::string::npos))
    try
    {
      std::ifstream ifs(itr->path().string());
      std::string graph_json_str;
      graph_json_str.assign(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());
      graphs.push_back(umrf_json::fromUmrfGraphJsonStr(graph_json_str));
    }
    catch(TemotoErrorStack e)
    {
      TEMOTO_PRINT(e.what());
      //throw FORWARD_TEMOTO_ERROR_STACK(e);
    }
  }
  return graphs;
}
catch (std::exception& e)
{
  throw CREATE_TEMOTO_ERROR_STACK(e.what());
}
catch(...)
{
  // Rethrow the exception
  throw CREATE_TEMOTO_ERROR_STACK("Received an unhandled exception");
}

const std::vector<UmrfNode>& ActionIndexer::getUmrfs() const
{
  // Lock the mutex
  LOCK_GUARD_TYPE l(action_sfs_mutex_);
  return indexed_umrfs_;
}

const std::vector<UmrfGraph>& ActionIndexer::getGraphs() const
{
  LOCK_GUARD_TYPE l(graphs_mutex_);
  return indexed_graphs_;
}