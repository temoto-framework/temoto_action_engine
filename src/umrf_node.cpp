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
{}

UmrfNode::UmrfNode(const UmrfNode& un)
: Umrf(un)
, state_(un.state_)
, package_name_(un.package_name_)
, suffix_(un.suffix_)
, library_path_(un.library_path_)
, parents_(un.parents_)
, children_(un.children_)
, id_(un.id_)
, full_name_(un.full_name_)
{}

UmrfNode UmrfNode::asUmrfNode() const
{
  return *this;
}

UmrfNode::~UmrfNode()
{}

const std::string& UmrfNode::getPackageName() const
{
  return package_name_;
}

bool UmrfNode::setPackageName(const std::string& package_name)
{
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
  full_name_ = name_ + "_" + std::to_string(suffix_);
  return full_name_;
}

const std::string& UmrfNode::getLibraryPath() const
{
  return library_path_;
}

bool UmrfNode::setLibraryPath(const std::string& library_path)
{
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

const unsigned int& UmrfNode::getSuffix() const
{
  return suffix_;
}

bool UmrfNode::setSuffix(const unsigned int& suffix)
{

  suffix_ = suffix;
  return true;  
}

const unsigned int& UmrfNode::getId() const
{
  return id_;
}
bool UmrfNode::setId(const unsigned int& id)
{
  id_ = id;
  return true;
}

const std::vector<UmrfNode::Relation>& UmrfNode::getParents() const
{
  return parents_;
}

bool UmrfNode::setParents(const std::vector<UmrfNode::Relation>& parents)
{
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
  parents_.clear();
}

bool UmrfNode::addParent(const UmrfNode::Relation& parent)
{
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
  return children_;
}

bool UmrfNode::setChildren(const std::vector<UmrfNode::Relation>& children)
{
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
  children_.clear();
}

bool UmrfNode::addChild(const UmrfNode::Relation& child)
{
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
  return UmrfNode::Relation(getName(), getSuffix());
}

bool UmrfNode::requiredParentsFinished() const
{
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