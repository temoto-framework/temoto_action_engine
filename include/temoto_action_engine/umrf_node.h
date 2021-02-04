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

#ifndef TEMOTO_ACTION_ENGINE__UMRF_NODE_H
#define TEMOTO_ACTION_ENGINE__UMRF_NODE_H

#include <memory>
#include "temoto_action_engine/umrf.h"

class UmrfNode : public Umrf
{
public:

  /// Defines all possible states for the action handle
  enum class State
  {
    UNINITIALIZED,      // Action library is not loaded or input parameters not received
    INITIALIZED,        // Action library is loaded and input parameters received
    READY,              // Action instance is loaded
    RUNNING,            // Action instance is running
    STOP_REQUESTED,     // A request to stop has been registered
    FINISHED,           // Action instance has finished execution
    ERROR,              // Problems with any critical component of the action
  };

  /// A convenience datastructure which helps to convert state names to std::string
  std::map<State, std::string> state_to_str_map_ = 
  {
    {State::UNINITIALIZED, "UNINITIALIZED"},
    {State::INITIALIZED, "INITIALIZED"},
    {State::READY, "READY"},
    {State::RUNNING, "RUNNING"},
    {State::STOP_REQUESTED, "STOP_REQUESTED"},
    {State::FINISHED, "FINISHED"},
    {State::ERROR, "ERROR"},
  };

  /**
   * @brief Embeds infromation about a parent/child connection
   * 
   */
  struct Relation
  {
    Relation()
    {}

    Relation(const std::string& name, const unsigned int& suffix, bool required = true)
    : name_(name)
    , suffix_(suffix)
    , required_(required)
    , received_(false)
    {}

    Relation(const Relation& r_in)
    : name_(r_in.getName())
    , suffix_(r_in.getSuffix())
    , required_(r_in.getRequired())
    , received_(r_in.getReceived())
    {}

    void operator=(const Relation& r_in)
    {
      name_ = r_in.getName();
      suffix_ = r_in.getSuffix();
      required_ = r_in.getRequired();
      received_ = r_in.getReceived();
    }

    bool operator==(const Relation& r_in) const
    {
      return (name_ == r_in.getName()) && (suffix_ == r_in.getSuffix());
    }

    const std::string& getName() const
    {
      return name_;
    }

    const unsigned int& getSuffix() const
    {
      return suffix_;
    }

    bool getRequired() const
    {
      return required_;
    }

    bool getReceived() const
    {
      return received_;
    }

    std::string getFullName() const
    {
      return name_ + "_" + std::to_string(suffix_);
    }

    bool empty() const
    {
      return name_.empty();
    }

    std::string name_;
    unsigned int suffix_;
    bool required_; // Indicates whether the child can only execute once the parent has finished the execution  
    bool received_; // Indicates whether the parent has finished execution. Applies only to parent-type relations
  };

  UmrfNode();

  UmrfNode(const UmrfNode& un);

  virtual ~UmrfNode();

  virtual UmrfNode asUmrfNode() const;

  void operator=(const UmrfNode& un)
  {
    package_name_ = un.package_name_;
    suffix_ = un.suffix_;
    library_path_ = un.library_path_;
    parents_ = un.parents_;
    children_ = un.children_;
    full_name_ = un.full_name_;
  }

  const std::string& getPackageName() const;
  bool setPackageName(const std::string& name);

  const unsigned int& getSuffix() const;
  bool setSuffix(const unsigned int& suffix);

  const std::string& getFullName() const;

  const std::string& getLibraryPath() const;
  bool setLibraryPath(const std::string& library_path);

  State getState() const;
  void setState(UmrfNode::State state);

  const std::vector<Relation>& getParents() const;
  bool setParents(const std::vector<Relation>& parents);
  void clearParents();
  bool addParent(const Relation& parent);
  bool removeParent(const Relation& parent);

  const std::vector<Relation>& getChildren() const;
  bool setChildren(const std::vector<Relation>& children);
  void clearChildren();
  bool addChild(const Relation& child);
  bool removeChild(const Relation& child);

  Relation asRelation() const;

  bool requiredParentsFinished() const;

  void setParentReceived(const UmrfNode::Relation& parent);

protected:

  mutable MUTEX_TYPE parents_rw_mutex_;
  GUARDED_VARIABLE(std::vector<Relation> parents_, parents_rw_mutex_);

  mutable MUTEX_TYPE children_rw_mutex_;
  GUARDED_VARIABLE(std::vector<Relation> children_, children_rw_mutex_);

  mutable MUTEX_TYPE package_name_rw_mutex_;
  GUARDED_VARIABLE(std::string package_name_, package_name_rw_mutex_);

  mutable MUTEX_TYPE suffix_rw_mutex_;
  GUARDED_VARIABLE(unsigned int suffix_, suffix_rw_mutex_);

  mutable MUTEX_TYPE full_name_rw_mutex_;
  GUARDED_VARIABLE(mutable std::string full_name_, full_name_rw_mutex_);

  mutable MUTEX_TYPE library_path_rw_mutex_;
  GUARDED_VARIABLE(std::string library_path_, library_path_rw_mutex_);

  mutable MUTEX_TYPE state_rw_mutex_;
  GUARDED_VARIABLE(State state_, state_rw_mutex_);
};

#endif