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

#ifndef TEMOTO_ACTION_ENGINE__UMRF_H
#define TEMOTO_ACTION_ENGINE__UMRF_H

#include <string>
#include <vector>
#include <iostream>
#include "temoto_action_engine/util/compiler_macros.hpp"
#include "temoto_action_engine/action_parameters.h"

namespace action_type
{
  const std::vector<std::string> TYPE_LIST {
    "sync",
    "async",
    "spontaneous"
  };
}

class Umrf
{
public:

  Umrf();

  Umrf(const Umrf& uj);

  void operator=(const Umrf& umrf)
  {
    name_ = umrf.name_;
    actor_ = umrf.actor_;
    type_ = umrf.type_;
    description_ = umrf.description_;
    input_parameters_ = umrf.input_parameters_;
    output_parameters_ = umrf.output_parameters_;
  }

  bool isEqual(const Umrf& umrf_in, bool check_updatable = true) const;

  const std::string& getName() const;
  std::string& getNameNc();
  bool setName(const std::string& name);

  const std::string& getActor() const;
  bool setActor(const std::string& actor);

  const std::string& getType() const;
  std::string& getTypeNc();
  bool setType(const std::string& type);

  const std::string& getDescription() const;
  bool setDescription(const std::string& description);

  const ActionParameters& getInputParameters() const;
  ActionParameters& getInputParametersNc();
  bool setInputParameters(const ActionParameters& params);
  bool setInputParameter(const ActionParameters::ParameterContainer& param_in);

  //bool copyInputParameters(const ActionParameters& action_parameters);

  bool inputParametersReceived() const;

  const ActionParameters& getOutputParameters() const;
  ActionParameters& getOutputParametersNc();
  bool setOutputParameters(const ActionParameters& params);

  bool updateInputParams(const ActionParameters& params_other);

  ~Umrf()
  {
  }

  friend std::ostream& operator<<( std::ostream& stream, const Umrf& umrf);

protected:

  mutable MUTEX_TYPE_R name_rw_mutex_;
  GUARDED_VARIABLE(std::string name_, name_rw_mutex_);

  mutable MUTEX_TYPE_R actor_rw_mutex_;
  GUARDED_VARIABLE(std::string actor_, actor_rw_mutex_);

  mutable MUTEX_TYPE_R type_rw_mutex_;
  GUARDED_VARIABLE(std::string type_, type_rw_mutex_);

  mutable MUTEX_TYPE_R description_rw_mutex_;
  GUARDED_VARIABLE(std::string description_, description_rw_mutex_);

  mutable MUTEX_TYPE_R input_params_rw_mutex_;
  GUARDED_VARIABLE(ActionParameters input_parameters_, input_params_rw_mutex_);

  mutable MUTEX_TYPE_R output_params_rw_mutex_;
  GUARDED_VARIABLE(ActionParameters output_parameters_, output_params_rw_mutex_);
};
#endif
