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

#include "temoto_action_engine/umrf.h"

#include <boost/circular_buffer.hpp>
#include <memory>
#include <optional>

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
  enum class State
  {
    UNINITIALIZED,
    INITIALIZED,
    RUNNING,
    PAUSE_REQUESTED,
    PAUSED,
    STOPPING,
    FINISHED,
    BYPASSED,
    ERROR
  };

  /// A convenience datastructure which helps to convert state names to std::string
  const static inline std::map<State, std::string> state_to_str_map_{
    {State::UNINITIALIZED  , "UNINITIALIZED"},
    {State::INITIALIZED    , "INITIALIZED"},
    {State::RUNNING        , "RUNNING"},
    {State::PAUSE_REQUESTED, "PAUSE_REQUESTED"},
    {State::PAUSED         , "PAUSED"},
    {State::STOPPING       , "STOPPING"},
    {State::FINISHED       , "FINISHED"},
    {State::BYPASSED       , "BYPASSED"},
    {State::ERROR          , "ERROR"}};

  const static inline std::map<std::string, State> str_to_state_map_{
    {"UNINITIALIZED"  , State::UNINITIALIZED},
    {"INITIALIZED"    , State::INITIALIZED},
    {"RUNNING"        , State::RUNNING},
    {"PAUSE_REQUESTED", State::PAUSE_REQUESTED},
    {"PAUSED"         , State::PAUSED},
    {"STOPPING"       , State::STOPPING},
    {"FINISHED"       , State::FINISHED},
    {"BYPASSED"       , State::BYPASSED},
    {"ERROR"          , State::ERROR}};

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
    : name_("")
    , instance_id_(0)
    , required_(true)
    , received_(false)
    , conditions_({
      {"on_true", "run"},
      {"on_false", "run"},
      {"on_stopped", "ignore"},
      {"on_error", "bypass"}})
    , parameter_remap_(std::map<std::string, std::string>())
    {}

    Relation(const std::string& name, const unsigned int& instance_id, bool required = true)
    : name_(name)
    , instance_id_(instance_id)
    , required_(required)
    , received_(false)
    , conditions_({
      {"on_true", "run"},
      {"on_false", "run"},
      {"on_stopped", "ignore"},
      {"on_error", "bypass"}})
    , parameter_remap_(std::map<std::string, std::string>())
    {}

    Relation(const Relation& r_in)
    : name_(r_in.getName())
    , instance_id_(r_in.getInstanceId())
    , required_(r_in.getRequired())
    , received_(r_in.getReceived())
    , conditions_(r_in.conditions_)
    , parameter_remap_(r_in.parameter_remap_)
    {}

    void operator=(const Relation& r_in)
    {
      name_ = r_in.getName();
      instance_id_ = r_in.getInstanceId();
      required_ = r_in.getRequired();
      received_ = r_in.getReceived();
      conditions_ = r_in.getConditions();
      parameter_remap_ = r_in.getParameterRemappings();
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

    Conditions getConditions() const
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

    std::string getParameterRemap(const std::string from) const
    {
      const auto& to = parameter_remap_.find(from);
      if (to == parameter_remap_.end())
      {
        throw CREATE_TEMOTO_ERROR_STACK("No remapping specified for parameter: " + from);
      }
      return to->second;
    }

    const std::map<std::string, std::string> getParameterRemappings() const
    {
      return parameter_remap_;
    }

    void setParameterRemap(const std::string& from, const std::string& to)
    {
      if (parameter_remap_.find(from) != parameter_remap_.end())
      {
        throw CREATE_TEMOTO_ERROR_STACK("Attempted to add duplicate remapping for parameter: " + from);
      }
      parameter_remap_.insert({from, to});
      return;
    }

    bool empty() const
    {
      return name_.empty();
    }

    const static inline std::vector<std::string> valid_preconditions_{
      "on_true",
      "on_false",
      "on_stopped",
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
    std::map<std::string, std::string> parameter_remap_;
  };

  struct LogEntry
  {
    std::string message;
    std::chrono::time_point<std::chrono::system_clock> timestamp;
  };

  using Log = boost::circular_buffer<LogEntry>;

  UmrfNode();

  UmrfNode(const UmrfNode& un);

  virtual ~UmrfNode();

  virtual UmrfNode asUmrfNode() const;

  void operator=(const UmrfNode& un)
  {
    Umrf::operator=(un);
    instance_id_ = un.instance_id_;
    parents_ = un.parents_;
    children_ = un.children_;
    full_name_ = un.full_name_;
    state_ = un.state_;
    actor_exec_traits_ = un.actor_exec_traits_;
    log_ = un.getLog();
  }

  const unsigned int& getInstanceId() const;
  bool setInstanceId(const unsigned int& instance_id);

  const std::string& getFullName() const;

  State getState() const;
  void setState(UmrfNode::State state);

  const std::vector<Relation>& getParents() const;
  bool setParents(const std::vector<Relation>& parents);
  void clearParents();
  bool addParent(const Relation& parent);
  bool removeParent(const Relation& parent);
  std::optional<Relation> getParentRelation(const UmrfNode::Relation& parent) const;

  const std::vector<Relation>& getChildren() const;
  bool setChildren(const std::vector<Relation>& children);
  void clearChildren();
  bool addChild(const Relation& child);
  bool removeChild(const Relation& child);

  Relation asRelation() const;

  bool requiredParentsFinished() const;

  void setParentReceived(const UmrfNode::Relation& parent);

  void setActorExecTraits(ActorExecTraits traits);
  ActorExecTraits getActorExecTraits() const;

  void setGuiAttributes(const std::string& gui_attributes);
  std::string getGuiAttributes() const;

  const Log& getLog() const;
  void writeLog(const std::string& message);
  void writeLog(const LogEntry& log_entry);

protected:

  mutable MUTEX_TYPE parents_rw_mutex_;
  GUARDED_VARIABLE(std::vector<Relation> parents_, parents_rw_mutex_);

  mutable MUTEX_TYPE children_rw_mutex_;
  GUARDED_VARIABLE(std::vector<Relation> children_, children_rw_mutex_);

  mutable MUTEX_TYPE instance_id_rw_mutex_;
  GUARDED_VARIABLE(unsigned int instance_id_, instance_id_rw_mutex_);

  mutable MUTEX_TYPE full_name_rw_mutex_;
  GUARDED_VARIABLE(mutable std::string full_name_, full_name_rw_mutex_);

  mutable MUTEX_TYPE state_rw_mutex_;
  GUARDED_VARIABLE(State state_, state_rw_mutex_);

  mutable MUTEX_TYPE log_rw_mutex_;
  GUARDED_VARIABLE(Log log_, log_rw_mutex_);

  ActorExecTraits actor_exec_traits_;

  std::string gui_attributes_; // JSON object stored as string
};

inline const UmrfNode::Relation GRAPH_ENTRY = UmrfNode::Relation("graph_entry", 0);
inline const UmrfNode::Relation GRAPH_EXIT = UmrfNode::Relation("graph_exit", 0);

#endif
