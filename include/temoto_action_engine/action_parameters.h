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

#ifndef TEMOTO_ACTION_ENGINE__ACTION_PARAMETERS_H
#define TEMOTO_ACTION_ENGINE__ACTION_PARAMETERS_H

#include <vector>
#include <set>
#include <string>
#include <algorithm>
#include "temoto_action_engine/action_parameter.h"
#include "temoto_action_engine/temoto_error.h"
#include "boost/any.hpp"

/*
 * Action Parameters
 */
class ActionParameters
{
public:
  typedef boost::any Payload;
  typedef ActionParameter<Payload> ParameterContainer;
  typedef std::set<ParameterContainer> Parameters;

  ActionParameters()
  {}

  ActionParameters(const Parameters& parameters)
  : parameters_(parameters)
  {}

  ActionParameters(const ActionParameters& a_parameters)
  : parameters_(a_parameters.parameters_)
  {}

  ActionParameters& operator = (const ActionParameters& ap)
  {
    for (const auto& p : ap)
    {
      parameters_.insert(p);
    }
  }

  /**
   * @brief Set the Parameter object
   * 
   * @param parameter_in 
   * @param merge_new Indicates whether old param data should be prioritized or new
   * @return true 
   * @return false 
   */
  bool setParameter(const ParameterContainer& parameter_in, bool merge_new = false)
  {
    try
    {
      if (parameters_.find(parameter_in) == parameters_.end())
      {
        return parameters_.insert(parameter_in).second;
      }
      else
      {
        if (merge_new)
        {
          // Insert the new param without keeping any info about the old param
          parameters_.erase(parameters_.find(parameter_in));
          parameters_.insert(parameter_in);
        }
        else
        {
          // Create copy of the old parameter and assign it new params data
          ParameterContainer param = *parameters_.find(parameter_in);
          param.setData(parameter_in.getData());
          parameters_.erase(parameters_.find(parameter_in));          
          parameters_.insert(param);
        }
      }
      return true;
    }
    catch(TemotoErrorStack e)
    {
      throw FORWARD_TEMOTO_ERROR_STACK(e);
    }
  }

  bool setParameter(const std::string& param_name, const std::string& param_type, const Payload& pl)
  {
    ParameterContainer pc(param_name, param_type);
    pc.setData(pl);
    setParameter(pc);
  }

  bool copyParameters(const ActionParameters& params_in)
  {
    try
    {
      // std::cout << "incoming params:\n" << params_in.toString();
      // std::cout << "this params:\n" << toString();
      std::set<std::string> param_names = getParamNames();
      // std::cout << " so a total of: " << param_names.size() << std::endl;
      while(!param_names.empty())
      {
        std::set<std::string> params_in_group = checkParamSourceGroup(*parameters_.find(*param_names.begin()));
        // std::cout << " got a grup of: " << params_in_group.size() << std::endl;
        for (const auto& param_in : params_in.getParameterGroup(params_in_group))
        {
          // std::cout << "  D1_4\n";
          setParameter(param_in);
        }
        // std::cout << " erasing params\n";
        for (const auto& p : params_in_group)
        {
          // std::cout << " - " << p << std::endl;
        }
        for (const auto param_in_group : params_in_group)
        {
          param_names.erase(param_in_group);
        }
      }
      return true;
    }
    catch(TemotoErrorStack e)
    {
      throw FORWARD_TEMOTO_ERROR_STACK(e);
    }
  }

  const ParameterContainer& getParameter(const std::string& name) const
  {
    if (!hasParameter(name))
    {
      throw CREATE_TEMOTO_ERROR("Could not find parameter '" + name + "'.");
    }
    return *parameters_.find(name);
  }

  // ParameterContainer& getParameterNc(const std::string& name)
  // {
  //   if (!hasParameter(name))
  //   {
  //     throw CREATE_TEMOTO_ERROR("Could not find parameter '" + name + "'.");
  //   }
  //   return *parameters_.find(name);
  // }

  const Parameters& getParameters() const
  {
    return parameters_;
  }

  bool removeParameter(const std::string& name)
  {
    if (!hasParameter(name))
    {
      return false;
    }
    parameters_.erase(parameters_.find(name));
    return true;
  }

  std::set<ParameterContainer> getParameterGroup(const std::set<std::string>& param_names) const
  {
    std::set<ParameterContainer> parameters_out;
    if (!containsAll(param_names))
    {
      return parameters_out;
    }

    // If all parameters are there then
    for (const auto& param_name : param_names)
    {
      parameters_out.insert(*parameters_.find(param_name));
    }
    return std::move(parameters_out);
  }

  bool hasParameter(const std::string& parameter_name) const
  {
    return (parameters_.find(parameter_name) != parameters_.end());
  }

  unsigned int getParameterCount() const
  {
    return parameters_.size();
  }

  bool empty() const
  {
    return parameters_.empty();
  }

  Parameters::const_iterator begin() const
  {
    return parameters_.begin();
  }

  Parameters::const_iterator end() const
  {
    return parameters_.end();
  }

  void clear()
  {
    parameters_.clear();
  }

  std::string toString() const
  {
    std::string out;
    for (const auto& parameter : parameters_)
    {
      out += " * " + parameter.getName() + "; type:" + parameter.getType() + "; source:" + std::to_string(parameter.getSourceId()) + "\n";
    }
    return out;
  }

  ~ActionParameters()
  {
    parameters_.clear();
  }

  std::set<std::string> getParamNames() const
  {
    std::set<std::string> keys;
    for (const auto& parameter : parameters_)
    {
      keys.insert(parameter.getName());
    }
    return keys;
  }
  
private:
  bool containsAll(const std::set<std::string>& param_names) const
  {
    bool contains_all = true;
    for (const auto& param_name : param_names)
    {
      if (parameters_.find(param_name) == parameters_.end())
      {
        contains_all = false;
        break;
      }
    }
    return contains_all;
  }

  std::set<std::string> checkParamSourceGroup(const ParameterContainer& param_in) const
  {
    // std::cout << " checking group of param: " << param_in.getName() << std::endl;
    std::set<std::string> params_in_same_group;
    params_in_same_group.insert(param_in.getName());

    // TODO: define a namespace for action parameters and put the "UNDEFINED SOURCE" constant under that
    if (param_in.getSourceId() == ActionParameter<int>::UNDEFINED_SOURCE)
    {
      return params_in_same_group;
    }

    for (const auto& parameter : parameters_)
    {
      if (parameter.getSourceId() == param_in.getSourceId())
      {
        // std::cout << " belogs to same group: " << parameter.getName() << std::endl;
        params_in_same_group.insert(parameter.getName());
      }
    }
    return params_in_same_group;
  }

  Parameters parameters_;
};

#endif
