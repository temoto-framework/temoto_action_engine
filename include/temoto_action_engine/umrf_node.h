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

  /// Defines what the underlying implementation for this umrf is
  enum class ActorExecTraits
  {
    LOCAL,  // A locally executable action 
    REMOTE, // An action that is executed on a remote system
    GRAPH   // A sub-graph
  };

  /// Defines all possible states for the action handle
  // enum class State
  // {
  //   NOT_SET,
  //   UNINITIALIZED,      // Action library is not loaded or input parameters not received
  //   INSTANTIATED,
  //   READY,              // Action instance is loaded and input parameters received
  //   RUNNING,            // Action instance is running
  //   PAUSED,
  //   STOP_REQUESTED,     // A request to stop has been registered
  //   FINISHED,           // Action instance has finished execution
  //   ERROR,              // Problems with any critical component of the action
  // };

  enum class State
  {
    NOT_SET,      // TODO: Rename to something more meaningful 
    INITIALIZED,
    RUNNING,
    PAUSED,
    STOPPING,
    FINISHED,
    ERROR
  };


  /// A convenience datastructure which helps to convert state names to std::string
  const static inline std::map<State, std::string> state_to_str_map_{
    {State::NOT_SET, "NOT_SET"},
    {State::UNINITIALIZED, "UNINITIALIZED"},
    {State::INSTANTIATED, "INSTANTIATED"},
    {State::READY, "READY"},
    {State::RUNNING, "RUNNING"},
    {State::PAUSED, "PAUSED"},
    {State::STOP_REQUESTED, "STOP_REQUESTED"},
    {State::FINISHED, "FINISHED"},
    {State::ERROR, "ERROR"},};

  /**
   * @brief Embeds infromation about a parent/child connection
   * 
   */
  struct Relation
  {
    // struct Condition
    // {
    //   std::string precondition;
    //   std::string response;
    // };

    typedef std::map<std::string, std::string> Conditions;

    Relation()
    : conditions_({
      {"on_true", "run"},
      {"on_false", "run"},
      {"on_error", "bypass"}})
    {}

    Relation(const std::string& name, const unsigned int& instance_id, bool required = true)
    : name_(name)
    , instance_id_(instance_id)
    , required_(required)
    , received_(false)
    , conditions_({
      {"on_true", "run"},
      {"on_false", "run"},
      {"on_error", "bypass"}})
    {}

    Relation(const Relation& r_in)
    : name_(r_in.getName())
    , instance_id_(r_in.getInstanceId())
    , required_(r_in.getRequired())
    , received_(r_in.getReceived())
    , conditions_(r_in.conditions_)
    {}

    void operator=(const Relation& r_in)
    {
      name_ = r_in.getName();
      instance_id_ = r_in.getInstanceId();
      required_ = r_in.getRequired();
      received_ = r_in.getReceived();
      conditions_ = r_in.getConditions();
    }

    bool operator==(const Relation& r_in) const
    {
      return (name_ == r_in.getName()) && (instance_id_ == r_in.getInstanceId());
    }

    const std::string& getName() const
    {
      return name_;
    }

    const unsigned int& getInstanceId() const
    {
      return instance_id_;
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
      return name_ + "_" + std::to_string(instance_id_);
    }

    const Conditions& getConditions() const
    {
      return conditions_;
    }

    std::string getResponse(const std::string& precondition) const
    {
      return conditions_.at(precondition);
    }

    void setCondition(const std::string& precondition, const std::string& response)
    {
      if (std::find(valid_preconditions_.begin(), valid_preconditions_.end(), precondition) == valid_preconditions_.end())
      {
        throw CREATE_TEMOTO_ERROR_STACK("Invalid 'precondition': " + precondition); 
      }

      if (std::find(valid_conditions_responses_.begin(), valid_conditions_responses_.end(), response) == valid_conditions_responses_.end())
      {
        throw CREATE_TEMOTO_ERROR_STACK("Invalid 'response': " + response); 
      }

      conditions_.at(precondition) = response;
    }

    bool empty() const
    {
      return name_.empty();
    }

    const static inline std::vector<std::string> valid_preconditions_{
      "on_true", 
      "on_false",
      "on_error"};

    /**
     * @brief List of responses an action can have for given preconditions
     * 
     * Run    - Run the action
     * Pause  - Pause if running, ignore otherwise
     * Stop   - Stop if running, ignore otherwise
     * Reset  - Reset if running, ignore otherwise
     * Bypass - Skip the action and run its children instead by passing the same return value
     * Ignore - Do nothing
     * 
     */
    const static inline std::vector<std::string> valid_conditions_responses_{
      "run",
      "pause",
      "restart",
      "stop",
      "bypass",
      "ignore"};

    std::string name_;
    unsigned int instance_id_;
    bool required_; // Indicates whether the child can only execute once the parent has finished the execution  
    bool received_; // Indicates whether the parent has finished execution. Applies only to parent-type relations
    std::map<std::string, std::string> conditions_; // Outlines the conditions of execution
  };

  UmrfNode();

  UmrfNode(const UmrfNode& un);

  virtual ~UmrfNode();

  virtual UmrfNode asUmrfNode() const;

  void operator=(const UmrfNode& un)
  {
    this->Umrf::operator=(un);
    instance_id_ = un.instance_id_;
    library_path_ = un.library_path_;
    parents_ = un.parents_;
    children_ = un.children_;
    full_name_ = un.full_name_;
    execute_first_ = un.execute_first_;
    state_ = un.state_;
    actor_exec_traits_ = un.actor_exec_traits_;
  }

  const unsigned int& getInstanceId() const;
  bool setInstanceId(const unsigned int& instance_id);

  const std::string& getFullName() const;

  const std::string& getLibraryPath() const;
  bool setLibraryPath(const std::string& library_path);

  State getState() const;
  void setState(UmrfNode::State state);

  bool getExecuteFirst() const;
  void setExecuteFirst(bool execute_first);

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

  void setActorExecTraits(ActorExecTraits traits);
  bool getActorExecTraits() const;

protected:

  mutable MUTEX_TYPE parents_rw_mutex_;
  GUARDED_VARIABLE(std::vector<Relation> parents_, parents_rw_mutex_);

  mutable MUTEX_TYPE children_rw_mutex_;
  GUARDED_VARIABLE(std::vector<Relation> children_, children_rw_mutex_);

  mutable MUTEX_TYPE instance_id_rw_mutex_;
  GUARDED_VARIABLE(unsigned int instance_id_, instance_id_rw_mutex_);

  mutable MUTEX_TYPE full_name_rw_mutex_;
  GUARDED_VARIABLE(mutable std::string full_name_, full_name_rw_mutex_);

  mutable MUTEX_TYPE library_path_rw_mutex_;
  GUARDED_VARIABLE(std::string library_path_, library_path_rw_mutex_);

  mutable MUTEX_TYPE state_rw_mutex_;
  GUARDED_VARIABLE(State state_, state_rw_mutex_);

  mutable MUTEX_TYPE execute_first_rw_mutex_;
  GUARDED_VARIABLE(bool execute_first_, execute_first_rw_mutex_);

  ActorExecTraits actor_exec_traits_;
};

#endif