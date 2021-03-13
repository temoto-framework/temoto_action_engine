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

#ifndef TEMOTO_ACTION_ENGINE__UMRF_H
#define TEMOTO_ACTION_ENGINE__UMRF_H

#include <string>
#include <vector>
#include <iostream>
#include "temoto_action_engine/compiler_macros.h"
#include "temoto_action_engine/action_parameters.h"

namespace action_effect
{
  const std::vector<std::string> EFFECT_LIST {
    "synchronous",
    "asynchronous"
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
    notation_ = umrf.notation_;
    effect_ = umrf.effect_;
    description_ = umrf.description_;
    input_parameters_ = umrf.input_parameters_;
    output_parameters_ = umrf.output_parameters_;
  }

  bool isEqual(const Umrf& umrf_in, bool check_updatable = true) const;

  const std::string& getName() const;
  std::string& getNameNc();
  bool setName(const std::string& name);

  const std::string& getNotation() const;
  bool setNotation(const std::string& notation);

  const std::string& getDescription() const;
  bool setDescription(const std::string& description);

  const std::string& getEffect() const;
  std::string& getEffectNc();
  bool setEffect(const std::string& effect);

  const ActionParameters& getInputParameters() const;
  ActionParameters& getInputParametersNc();
  bool setInputParameters(const ActionParameters& params);
  bool setInputParameter(const ActionParameters::ParameterContainer& param_in);

  //bool copyInputParameters(const ActionParameters& action_parameters);
  
  bool inputParametersReceived() const;

  const ActionParameters& getOutputParameters() const;
  ActionParameters& getOutputParametersNc();
  bool setOutputParameters(const ActionParameters& params);

  bool updateInputParams(const Umrf& umrf_in);
  
  ~Umrf()
  {
  }

  friend std::ostream& operator<<( std::ostream& stream, const Umrf& umrf);
  
protected:

  mutable MUTEX_TYPE_R name_rw_mutex_;
  GUARDED_VARIABLE(std::string name_, name_rw_mutex_);

  mutable MUTEX_TYPE_R notation_rw_mutex_;
  GUARDED_VARIABLE(std::string notation_, notation_rw_mutex_);

  mutable MUTEX_TYPE_R effect_rw_mutex_;
  GUARDED_VARIABLE(std::string effect_, effect_rw_mutex_);

  mutable MUTEX_TYPE_R description_rw_mutex_;
  GUARDED_VARIABLE(std::string description_, description_rw_mutex_);
  
  mutable MUTEX_TYPE_R input_params_rw_mutex_;
  GUARDED_VARIABLE(ActionParameters input_parameters_, input_params_rw_mutex_);

  mutable MUTEX_TYPE_R output_params_rw_mutex_;
  GUARDED_VARIABLE(ActionParameters output_parameters_, output_params_rw_mutex_);
};
#endif