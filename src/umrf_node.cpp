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

#include "temoto_action_engine/umrf_node.h"

UmrfNode::UmrfNode()
: state_(State::UNINITIALIZED)
, instance_id_(0)
, execute_first_(false)
, is_remote_actor_(false)
{}

UmrfNode::UmrfNode(const UmrfNode& un)
: Umrf(un)
, state_(un.state_)
, package_name_(un.package_name_)
, instance_id_(un.instance_id_)
, library_path_(un.library_path_)
, parents_(un.parents_)
, children_(un.children_)
, full_name_(un.full_name_)
, execute_first_(un.execute_first_)
, is_remote_actor_(un.is_remote_actor_)
{}

UmrfNode UmrfNode::asUmrfNode() const
{
  return *this;
}

UmrfNode::~UmrfNode()
{}

const std::string& UmrfNode::getPackageName() const
{
  LOCK_GUARD_TYPE guard_package_name(package_name_rw_mutex_);
  return package_name_;
}

bool UmrfNode::setPackageName(const std::string& package_name)
{
  LOCK_GUARD_TYPE guard_package_name(package_name_rw_mutex_);
  if (!package_name.empty())
  {
    package_name_ = package_name;
    return true;  
  }
  else
  {
    return false;
  }
}

const std::string& UmrfNode::getFullName() const
{
  LOCK_GUARD_TYPE guard_full_name(full_name_rw_mutex_);
  full_name_ = name_ + "_" + std::to_string(instance_id_);
  return full_name_;
}

UmrfNode::State UmrfNode::getState() const
{
  LOCK_GUARD_TYPE guard_state(state_rw_mutex_);
  return state_;
}

void UmrfNode::setState(UmrfNode::State state)
{
  LOCK_GUARD_TYPE guard_state(state_rw_mutex_);
  state_ = state;
}

bool UmrfNode::getExecuteFirst() const
{
  LOCK_GUARD_TYPE guard_execute_first(execute_first_rw_mutex_);
  return execute_first_;
}

void UmrfNode::setExecuteFirst(bool execute_first)
{
  LOCK_GUARD_TYPE guard_execute_first(execute_first_rw_mutex_);
  execute_first_ = execute_first;
}

const std::string& UmrfNode::getLibraryPath() const
{
  LOCK_GUARD_TYPE guard_library_path(library_path_rw_mutex_);
  return library_path_;
}

bool UmrfNode::setLibraryPath(const std::string& library_path)
{
  LOCK_GUARD_TYPE guard_library_path(library_path_rw_mutex_);
  if (!library_path.empty())
  {
    library_path_ = library_path;
    return true;  
  }
  else
  {
    return false;
  }
}

const unsigned int& UmrfNode::getInstanceId() const
{
  LOCK_GUARD_TYPE guard_instance_id(instance_id_rw_mutex_);
  return instance_id_;
}

bool UmrfNode::setInstanceId(const unsigned int& instance_id)
{
  LOCK_GUARD_TYPE guard_instance_id(instance_id_rw_mutex_);
  instance_id_ = instance_id;
  full_name_ = name_ + "_" + std::to_string(instance_id_);
  return true;  
}

const std::vector<UmrfNode::Relation>& UmrfNode::getParents() const
{
  LOCK_GUARD_TYPE guard_parents(parents_rw_mutex_);
  return parents_;
}

bool UmrfNode::setParents(const std::vector<UmrfNode::Relation>& parents)
{
  LOCK_GUARD_TYPE guard_parents(parents_rw_mutex_);
  if (!parents.empty())
  {
    parents_ = parents;
    return true;  
  }
  else
  {
    return false;
  }
}

void UmrfNode::clearParents()
{
  LOCK_GUARD_TYPE guard_parents(parents_rw_mutex_);
  parents_.clear();
}

bool UmrfNode::addParent(const UmrfNode::Relation& parent)
{
  LOCK_GUARD_TYPE guard_parents(parents_rw_mutex_);
  if (!parent.empty())
  {
    parents_.push_back(parent);
    return true;
  }
  else
  {
    return false;
  }
}

bool UmrfNode::removeParent(const UmrfNode::Relation& parent)
{
  LOCK_GUARD_TYPE guard_parents(parents_rw_mutex_);
  auto parent_it = std::find(parents_.begin(), parents_.end(), parent);

  if (parent_it != parents_.end())
  {
    parents_.erase(parent_it);
    return true;
  }
  else
  {
    return false;
  }
}

const std::vector<UmrfNode::Relation>& UmrfNode::getChildren() const
{
  LOCK_GUARD_TYPE guard_children(children_rw_mutex_);
  return children_;
}

bool UmrfNode::setChildren(const std::vector<UmrfNode::Relation>& children)
{
  LOCK_GUARD_TYPE guard_children(children_rw_mutex_);
  if (!children.empty())
  {
    children_ = children;
    return true;  
  }
  else
  {
    return false;
  }
}

void UmrfNode::clearChildren()
{
  LOCK_GUARD_TYPE guard_children(children_rw_mutex_);
  children_.clear();
}

bool UmrfNode::addChild(const UmrfNode::Relation& child)
{
  LOCK_GUARD_TYPE guard_children(children_rw_mutex_);
  if (!child.empty())
  {
    children_.push_back(child);
    return true;
  }
  else
  {
    return false;
  }
}

bool UmrfNode::removeChild(const UmrfNode::Relation& child)
{
  LOCK_GUARD_TYPE guard_children(children_rw_mutex_);
  auto child_it = std::find(children_.begin(), children_.end(), child);

  if (child_it != children_.end())
  {
    children_.erase(child_it);
    return true;
  }
  else
  {
    return false;
  }
}

UmrfNode::Relation UmrfNode::asRelation() const
{
  return UmrfNode::Relation(getName(), getInstanceId());
}

bool UmrfNode::requiredParentsFinished() const
{
  LOCK_GUARD_TYPE guard_parents(parents_rw_mutex_);
  for (const auto& parent : parents_)
  {
    if (parent.getRequired() && !parent.getReceived())
    {
      return false;
    }
  }
  return true;
}

void UmrfNode::setParentReceived(const UmrfNode::Relation& parent)
{
  LOCK_GUARD_TYPE guard_parents(parents_rw_mutex_);
  auto parent_it = std::find(parents_.begin(), parents_.end(), parent);

  if (parent_it != parents_.end())
  {
    parent_it->received_ = true;
  }
  else
  {
    throw CREATE_TEMOTO_ERROR_STACK("The parent does not exist");
  }
}

void UmrfNode::setIsRemoteActor(bool is_remote_actor)
{
  is_remote_actor_ = is_remote_actor;
}

bool UmrfNode::getIsRemoteActor() const
{
  return is_remote_actor_;
}