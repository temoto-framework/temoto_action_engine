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

#include "temoto_action_engine/umrf.h"
#include "temoto_action_engine/temoto_error.h"
#include <iostream>

Umrf::Umrf()
{}

Umrf::Umrf(const Umrf& uj)
: name_(uj.name_)
, actor_(uj.actor_)
, description_(uj.description_)
, type_(uj.type_)
, input_parameters_(uj.input_parameters_)
, output_parameters_(uj.output_parameters_)
{}

const std::string& Umrf::getName() const
{
  return name_;
}

std::string& Umrf::getNameNc()
{
  return name_;
}

bool Umrf::setName(const std::string& name)
{
  LOCK_GUARD_TYPE_R guard_name(name_rw_mutex_);
  if (!name.empty())
  {
    name_ = name;
    return true;  
  }
  else
  {
    return false;
  }
}

const std::string& Umrf::getActor() const
{
  return actor_;
}

bool Umrf::setActor(const std::string& actor)
{
  LOCK_GUARD_TYPE_R guard_description(actor_rw_mutex_);
  actor_ = actor;
  return true;
}

const std::string& Umrf::getDescription() const
{
  return description_;
}

bool Umrf::setDescription(const std::string& description)
{
  LOCK_GUARD_TYPE_R guard_description(description_rw_mutex_);
  description_ = description;
  return true;
}

const std::string& Umrf::getType() const
{
  return type_;
}

std::string& Umrf::getTypeNc()
{
  return type_;
}

bool Umrf::setType(const std::string& type)
{
  LOCK_GUARD_TYPE_R guard_type(type_rw_mutex_);
  if (!type.empty())
  {
    type_ = type;
    return true;  
  }
  else
  {
    return false;
  }
}

const ActionParameters& Umrf::getInputParameters() const
{
  LOCK_GUARD_TYPE_R guard_input_params(input_params_rw_mutex_);
  return input_parameters_;
}

ActionParameters& Umrf::getInputParametersNc()
{
  LOCK_GUARD_TYPE_R guard_input_params(input_params_rw_mutex_);
  return input_parameters_;
}

bool Umrf::setInputParameters(const ActionParameters& params)
{
  LOCK_GUARD_TYPE_R guard_input_params(input_params_rw_mutex_);
  if (!params.empty())
  {
    input_parameters_ = params;
    return true;
  }
  else
  {
    return false;
  }
}

bool Umrf::setInputParameter(const ActionParameters::ParameterContainer& param_in)
{
  LOCK_GUARD_TYPE_R guard_input_params(input_params_rw_mutex_);
  return input_parameters_.setParameter(param_in);
}

const ActionParameters& Umrf::getOutputParameters() const
{
  LOCK_GUARD_TYPE_R guard_output_params(output_params_rw_mutex_);
  return output_parameters_;
}

ActionParameters& Umrf::getOutputParametersNc()
{
  LOCK_GUARD_TYPE_R guard_output_params(output_params_rw_mutex_);
  return output_parameters_;
}

bool Umrf::setOutputParameters(const ActionParameters& params)
{
  LOCK_GUARD_TYPE_R guard_output_params(output_params_rw_mutex_);
  if (!params.empty())
  {
    output_parameters_ = params;
    return true;
  }
  else
  {
    return false;
  }
}

// bool Umrf::copyInputParameters(const ActionParameters& action_parameters)
// {
//   LOCK_GUARD_TYPE_R guard_input_params(input_params_rw_mutex_);
//   input_parameters_.copyParameters(action_parameters);
//   return inputParametersReceived();
// }

bool Umrf::inputParametersReceived() const
{
  LOCK_GUARD_TYPE_R guard_input_params(input_params_rw_mutex_);
  bool params_received = true;
  for (const auto& input_parameter : input_parameters_)
  {
    if (!input_parameter.isRequired())
    {
      continue;
    }
    if (input_parameter.getDataSize() == 0)
    {
      params_received = false;
      break;
    }
  }
  return params_received;
}

std::ostream& operator<<( std::ostream& stream, const Umrf& umrf)
{
  stream << "  name: " << umrf.getName() << std::endl;
  stream << "  type: " << umrf.getType() << std::endl;

  if (!umrf.getInputParameters().empty())
  {
    stream << "  input_parameters:" << std::endl;
    for (const auto& ip : umrf.getInputParameters())
    {
      stream << "   - " 
      << "name=" << ip.getName() << "; "
      << "type=" << ip.getType() << "; "
      << "required=" << ip.isRequired() << "; "
      << "data_size=" << ip.getDataSize() << "; "
      << "allowed_data_size=" << ip.getAllowedData().size() << std::endl;
    }
  }

  if (!umrf.getOutputParameters().empty())
  {
    stream << "  output_parameters:" << std::endl;
    for (const auto& op : umrf.getOutputParameters())
    {
      stream << "   - " 
      << "name=" << op.getName() << "; "
      << "type=" << op.getType() << "; "
      << "data_size=" << op.getDataSize() << std::endl;
    }
  }

  return stream;
}

bool Umrf::updateInputParams(const ActionParameters& params_other)
try
{
  LOCK_GUARD_TYPE_R guard_input_params(input_params_rw_mutex_);
  bool parameters_updated = false;
  for (const auto& input_param_in : params_other)
  {
    // Get the parameter
    const ActionParameters::ParameterContainer& input_param_loc = *input_parameters_.getParameters().find(input_param_in);

    // Skip that parameter if it's not updatable
    if (!input_param_loc.isUpdatable())
    {
      continue;
    }

    // Update the parameter
    if (!setInputParameter(input_param_in))
    {
      continue;
    }
    parameters_updated = true;
  }
  return parameters_updated;
}
catch(TemotoErrorStack e)
{
  throw FORWARD_TEMOTO_ERROR_STACK(e);
}


bool Umrf::isEqual(const Umrf& umrf_in, bool check_updatable) const
{
  LOCK_GUARD_TYPE_R guard_input_params(input_params_rw_mutex_);
  LOCK_GUARD_TYPE_R guard_output_params(output_params_rw_mutex_);

  /*
   * Compare the general parameters
   */
  if ((name_ != umrf_in.name_) ||
      (type_ != umrf_in.type_))
  {
    return false;
  }

  /*
   * Compare the parameters
   */
  const ActionParameters& input_parameters_in = umrf_in.getInputParameters();
  const ActionParameters& output_parameters_in = umrf_in.getOutputParameters();

  if ((input_parameters_.getParameterCount() != input_parameters_in.getParameterCount()) ||
      (output_parameters_.getParameterCount() != output_parameters_in.getParameterCount()))
  {
    return false;
  }

  // Check each input parameter individually
  for (const auto& input_param : input_parameters_)
  {
    if (input_parameters_in.hasParameter(input_param.getName()))
    {
      // Check if updatability has to be controlled
      if (check_updatable)
      {
        if (!input_param.isEqualNoData(input_parameters_in.getParameter(input_param.getName())))
        {
          // Params not equal
          return false;
        }
      }
      else
      {
        if (!input_param.isEqualNoDataNoUpdate(input_parameters_in.getParameter(input_param.getName())))
        {
          // Params not equal
          return false;
        }
      }
    }
    else
    {
      // Does not have the parameter
      return false;
    }
  }

  // Check each output parameter individually
  for (const auto& output_param : output_parameters_)
  {
    if (output_parameters_in.hasParameter(output_param.getName()))
    {
      if (!output_param.isEqualNoData(output_parameters_in.getParameter(output_param.getName())))
      {
        // Params not equal
        return false;
      }
    }
    else
    {
      // Does not have the parameter
      return false;
    }
  }
  return true;
}