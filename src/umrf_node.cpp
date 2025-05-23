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

// const UmrfNode::Relation GRAPH_ENTRY = UmrfNode::Relation("graph_entry", 0);
// const UmrfNode::Relation GRAPH_EXIT = UmrfNode::Relation("graph_exit", 0);

UmrfNode::UmrfNode()
: instance_id_(0)
, state_(State::UNINITIALIZED)
, actor_exec_traits_(UmrfNode::ActorExecTraits::LOCAL)
, log_{UmrfNode::Log(10)}
{}

UmrfNode::UmrfNode(const UmrfNode& un)
: Umrf(un)
, parents_(un.parents_)
, children_(un.children_)
, instance_id_(un.instance_id_)
, full_name_(un.full_name_)
, state_(un.state_)
, actor_exec_traits_(un.actor_exec_traits_)
, gui_attributes_(un.gui_attributes_)
, log_(un.getLog())
{}

UmrfNode UmrfNode::asUmrfNode() const
{
  return *this;
}

UmrfNode::~UmrfNode()
{}

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

std::optional<UmrfNode::Relation> UmrfNode::getParentRelation(const UmrfNode::Relation& parent) const
{
  const auto it = std::find(getParents().begin(), getParents().end(), parent);

  if (it == getParents().end())
  {
    return std::optional<UmrfNode::Relation>();
  }

  return *it;
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
    throw CREATE_TEMOTO_ERROR_STACK("Action '" + getFullName() + "' does not have a parent '" + parent.getFullName() + "'");
  }
}

void UmrfNode::setActorExecTraits(UmrfNode::ActorExecTraits traits)
{
  actor_exec_traits_ = traits;
}

UmrfNode::ActorExecTraits UmrfNode::getActorExecTraits() const
{
  return actor_exec_traits_;
}

void UmrfNode::setGuiAttributes(const std::string& gui_attributes)
{
  gui_attributes_ = gui_attributes;
}

std::string UmrfNode::getGuiAttributes() const
{
  return gui_attributes_;
}

const UmrfNode::Log& UmrfNode::getLog() const
{
  LOCK_GUARD_TYPE guard_parents(log_rw_mutex_);
  return log_;
}

void UmrfNode::writeLog(const std::string& message)
{
  LOCK_GUARD_TYPE l(log_rw_mutex_);
  log_.push_front({
    .message = message,
    .timestamp = std::chrono::system_clock::now()});
}

void UmrfNode::writeLog(const UmrfNode::LogEntry& log_entry)
{
  LOCK_GUARD_TYPE l(log_rw_mutex_);
  log_.push_front(log_entry);
}
