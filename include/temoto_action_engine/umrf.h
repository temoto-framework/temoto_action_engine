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
    package_name_ = umrf.package_name_;
    suffix_ = umrf.suffix_;
    notation_ = umrf.notation_;
    effect_ = umrf.effect_;
    library_path_ = umrf.library_path_;
    parents_ = umrf.parents_;
    children_ = umrf.children_;
    id_ = umrf.id_;
    full_name_ = umrf.full_name_;
    input_parameters_ = umrf.input_parameters_;
    output_parameters_ = umrf.output_parameters_;
  }

  const std::string& getName() const;
  std::string& getNameNc();
  bool setName(const std::string& name);

  const std::string& getPackageName() const;
  bool setPackageName(const std::string& name);

  const std::string& getSuffix() const;
  bool setSuffix(const std::string& suffix);

  const std::string& getNotation() const;
  bool setNotation(const std::string& notation);

  const std::string& getFullName() const;

  const std::string& getLibraryPath() const;
  bool setLibraryPath(const std::string& library_path);

  const std::vector<std::string>& getParents() const;
  bool setParents(const std::vector<std::string>& parents);

  const std::vector<std::string>& getChildren() const;
  bool setChildren(const std::vector<std::string>& children);

  const std::string& getEffect() const;
  std::string& getEffectNc();
  bool setEffect(const std::string& effect);

  const unsigned int& getId() const;
  bool setId(const unsigned int& id);

  const ActionParameters& getInputParameters() const;
  ActionParameters& getInputParametersNc();
  bool setInputParameters(const ActionParameters& params);

  bool copyInputParameters(const ActionParameters& action_parameters);
  
  bool inputParametersReceived() const;

  const ActionParameters& getOutputParameters() const;
  ActionParameters& getOutputParametersNc();
  bool setOutputParameters(const ActionParameters& params);

  bool isUmrfCorrect() const;
  
  ~Umrf()
  {
  }

  friend std::ostream& operator<<( std::ostream& stream, const Umrf& umrf);
  
private:
  std::string name_;
  std::string package_name_;
  std::string suffix_;
  std::string notation_;
  std::string effect_;
  std::vector<std::string> parents_;
  std::vector<std::string> children_;

  mutable MUTEX_TYPE_R input_params_rw_mutex_;
  GUARDED_VARIABLE(ActionParameters input_parameters_, input_params_rw_mutex_);

  mutable MUTEX_TYPE_R output_params_rw_mutex_;
  GUARDED_VARIABLE(ActionParameters output_parameters_, output_params_rw_mutex_);

  /*
   * Internal management variables
   */ 
  unsigned int id_;
  std::string full_name_;
  std::string library_path_;
};
#endif