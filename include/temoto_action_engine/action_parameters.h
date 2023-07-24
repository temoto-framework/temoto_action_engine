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
    parameters_ = ap.parameters_;
    return *this;
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
  try
  {
    auto local_parameter_it = parameters_.find(parameter_in);
    if (local_parameter_it == parameters_.end())
    {
      return parameters_.insert(parameter_in).second;
    }

    /*
     * Modify the existing parameter. First check if the types match
     */
    if (local_parameter_it->getType() != parameter_in.getType())
    {
      return false;
    }

    if (merge_new)
    {
      // Insert the new param without keeping any info about the old param
      parameters_.erase(local_parameter_it);
      parameters_.insert(parameter_in);
    }
    else
    {
      // Check if the "parameter-to-be-set" is restricted to certain data values
      if (!checkParamAllowedData(*local_parameter_it, parameter_in))
      {
        return false;
      }

      // Create copy of the old parameter and assign it new params data
      ParameterContainer param = *parameters_.find(parameter_in);
      if (parameter_in.getDataSize() != 0)
      {
        param.setData(parameter_in.getData());
      }
      parameters_.erase(parameters_.find(parameter_in));          
      parameters_.insert(param);
    }
    return true;
  }
  catch(TemotoErrorStack& e)
  {
    throw FORWARD_TEMOTO_ERROR_STACK(e);
  }


  bool setParameter(const std::string& param_name, const std::string& param_type, const Payload& pl)
  {
    ParameterContainer pc(param_name, param_type);
    pc.setData(pl);
    return setParameter(pc);
  }

  template <class T> void setParameterData(const std::string& param_name, const T& data)
  {
    ParameterContainer new_param = *parameters_.find(param_name);
    new_param.setData(boost::any(data));
    parameters_.erase(parameters_.find(param_name));
    parameters_.insert(new_param);
  }

  /**
   * @brief Checks if the destination parameter is constrained to any particular allowed values or not.
   * TODO: This method should be embedded into the ActionParameter<T> class via a generic comparison method.
   * 
   * @param param_dest
   * @param param_source 
   * @return true if there are no restrictions or these are satisfied
   * @return false 
   */
  bool checkParamAllowedData(const ParameterContainer& param_dest, const ParameterContainer& param_source) const
  {
    if (param_dest.getAllowedData().empty())
    {
      return true;
    }

    Payload param_source_data = param_source.getData();

    // Check each allowed data instance
    for (const auto& allowed_dest_data : param_dest.getAllowedData())
    {
      // TODO: due to the lack of knowledge, only generic types such as strings and numbers
      // are checked. I guess one can serialize the data and compare it byte-by-byte ...
      if (param_dest.getType() == "string")
      {
        if (boost::any_cast<std::string>(allowed_dest_data) == boost::any_cast<std::string>(param_source_data))
        {
          return true;
        }
      }
      else if (param_dest.getType() == "number")
      {
        if (boost::any_cast<double>(allowed_dest_data) == boost::any_cast<double>(param_source_data))
        {
          return true;
        }
      }
      else if (param_dest.getType() == "bool")
      {
        if (boost::any_cast<bool>(allowed_dest_data) == boost::any_cast<bool>(param_source_data))
        {
          return true;
        }
      }
    }
    return false;
  }

  std::set<std::string> getTransferableParams(const ActionParameters& params_in) const
  try
  {
    std::set<std::string> local_param_names = getParamNames();
    std::set<std::string> transferable_params;

    while(!local_param_names.empty())
    {
      std::set<std::string> local_params_in_group = checkParamSourceGroup(*parameters_.find(*local_param_names.begin()));
      
      if (!params_in.getParameterGroup(local_params_in_group).empty())
      {
        bool all_params_in_group_ok = true;
        for (const auto& param_in : params_in.getParameterGroup(local_params_in_group))
        {
          if (!checkParamAllowedData(getParameter(param_in.getName()), param_in))
          {
            all_params_in_group_ok = false;
          }
        }

        if(all_params_in_group_ok)
        {
          transferable_params.insert(local_params_in_group.begin(), local_params_in_group.end());
        }
      }

      for (const auto& p : local_params_in_group)
      {
        local_param_names.erase(p);
      }
    }
    return transferable_params;
  }
  catch(TemotoErrorStack& e)
  {
    throw FORWARD_TEMOTO_ERROR_STACK(e);
  }

  const ParameterContainer& getParameter(const std::string& name) const
  {
    if (!hasParameter(name))
    {
      throw CREATE_TEMOTO_ERROR_STACK("Could not find parameter '" + name + "'.");
    }
    return *parameters_.find(name);
  }

  template <class T> T getParameterData(const std::string& name) const
  try
  {
    const ParameterContainer& pc = getParameter(name);
    if (pc.getDataSize() > 0)
    {
      return boost::any_cast<T>(pc.getData());
    }
    else
    {
      return T();
    }
  }
  catch(TemotoErrorStack e)
  {
    throw FORWARD_TEMOTO_ERROR_STACK(e);
  }
  catch(std::exception e)
  {
    throw CREATE_TEMOTO_ERROR_STACK(e.what());
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

  Parameters& getParametersNc()
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

  Parameters getParameterGroup(const std::set<std::string>& param_names) const
  {
    Parameters parameters_out;
    if (!containsAll(param_names))
    {
      return parameters_out;
    }

    // If all parameters are there then
    for (const auto& param_name : param_names)
    {
      parameters_out.insert(*parameters_.find(param_name));
    }
    return parameters_out;
  }

  bool hasParameter(const std::string& parameter_name) const
  {
    bool todo_remove = parameters_.find(parameter_name) != parameters_.end();
    return todo_remove;
    //return (parameters_.find(parameter_name) != parameters_.end());
  }

  bool hasParameter(const ParameterContainer& param_in) const
  {
    const auto& local_param_it = parameters_.find(param_in.getName());
    if (local_param_it == parameters_.end() || local_param_it->getType() != param_in.getType())
    {
      return false;
    }
    else
    {
      return true;
    }
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
        params_in_same_group.insert(parameter.getName());
      }
    }
    return params_in_same_group;
  }

  Parameters parameters_;
};

#endif
