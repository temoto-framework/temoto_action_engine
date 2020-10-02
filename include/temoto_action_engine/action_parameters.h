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
#include <iostream>

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
    // for (const auto& p : ap)
    // {
    //   parameters_.insert(p);
    // }
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
      auto local_parameter_it = parameters_.find(parameter_in);
      std::cout << "local_param " << local_parameter_it->getName() << " found. allowed_data=" << local_parameter_it->getAllowedData().size() << std::endl;
      if (local_parameter_it == parameters_.end())
      {
        return parameters_.insert(parameter_in).second;
      }
      else
      {
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
      std::cout << "no restrictions on this param" << std::endl;
      return true;
    }
    std::cout << "WE GOTTA LIVE ONE" << std::endl;

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
    }
    return false;
  }

  bool copyParameters(const ActionParameters& params_in)
  {
    try
    {
      std::cout << "Alright, the finished action outputs " << params_in.getParameterCount() << " params" << std::endl;
      for (const auto& prm : params_in)
      {
        std::cout << " * name=" << prm.getName() << ". type=" << prm.getType() << ". alwd=" << prm.getAllowedData().size() << ". data=" << prm.getDataSize() << std::endl;
      }
      std::set<std::string> param_names = getParamNames();
      std::cout << "And, the next action accepts " << param_names.size() << " params" << std::endl;
      for (const auto& prm : parameters_)
      {
        std::cout << " * name=" << prm.getName() << ". type=" << prm.getType() << ". alwd=" << prm.getAllowedData().size() << std::endl;
      }
      while(!param_names.empty())
      {
        std::set<std::string> params_in_group = checkParamSourceGroup(*parameters_.find(*param_names.begin()));

        // First make sure that the parameters are of same type
        bool all_params_correct = true;
        for (const auto& param_in : params_in.getParameterGroup(params_in_group))
        {
          if (!hasParameter(param_in))
          {
            all_params_correct = false;
            break;
          }
        }
        if (all_params_correct)
        {
          for (const auto& param_in : params_in.getParameterGroup(params_in_group))
          {
            setParameter(param_in);
          }
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
    std::cout << __func__ << " getting param " << name << std::endl;
    if (!hasParameter(name))
    {
      std::cout << __func__ << " couldnt find the param " << std::endl;
      throw CREATE_TEMOTO_ERROR("Could not find parameter '" + name + "'.");
    }
    std::cout << __func__ << " fot it " << std::endl;
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
    return std::move(parameters_out);
  }

  bool hasParameter(const std::string& parameter_name) const
  {
    return (parameters_.find(parameter_name) != parameters_.end());
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
