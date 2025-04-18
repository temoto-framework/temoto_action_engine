/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright 2023 TeMoto Framework
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License atUNDEFINED_SOURCE
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "temoto_action_engine/action_match_finder.h"

bool ActionMatchFinder::findMatchingAction(UmrfNode& umrf_node_in
, const std::vector<UmrfNode>& known_umrfs
, bool name_match) const
try
{
  for (const auto& known_umrf : known_umrfs)
  {
    if (findMatchingAction(umrf_node_in, known_umrf, name_match))
    {
      return true;
    }
  }
  return false;
}
catch(TemotoErrorStack e)
{
  throw FORWARD_TEMOTO_ERROR_STACK(e);
}

bool ActionMatchFinder::findMatchingAction(UmrfNode& umrf_node_in, const UmrfNode& known_umrf, bool name_match) const
{

  /*
   * Compare name
   */
  if ((umrf_node_in.getName() != known_umrf.getName()) && name_match)
  {
    return false;
  }

  /*
   * Compare input parameters
   *    TODO: Add other PVF field comparisons: 
   * https://temoto-telerobotics.github.io/temoto-telerobotics.github.io/site/concepts/actions#parameter-value-format
   */
  if (umrf_node_in.getInputParameters().getParameterCount() != known_umrf.getInputParameters().getParameterCount())
  {
    return false;
  }

  bool input_params_match = true;
  for (const auto& umrf_in_input_param : umrf_node_in.getInputParameters())
  {
    if (!known_umrf.getInputParameters().hasParameter(umrf_in_input_param.getName()))
    {
      input_params_match = false;
      break;
    }
    /*
     * TODO: Check for other PVF values
     */
  }
  if (!input_params_match)
  {
    return false;
  }

  /*
   * Compare output parameters
   *    TODO: Add other PVF field comparisons: 
   * https://temoto-telerobotics.github.io/temoto-telerobotics.github.io/site/concepts/actions#parameter-value-format
   */
  if (umrf_node_in.getOutputParameters().getParameterCount() != known_umrf.getOutputParameters().getParameterCount())
  {
    return false;
  }

  bool output_params_match = true;
  for (const auto& umrf_in_output_param : umrf_node_in.getOutputParameters())
  {
    if (!known_umrf.getOutputParameters().hasParameter(umrf_in_output_param.getName()))
    {
      output_params_match = false;
      break;
    }
  }
  if (!output_params_match)
  {
    return false;
  }

  umrf_node_in.setName(known_umrf.getName());

  /*
   * Update parameter PVF fields
   */
  for (const auto& known_umrf_param : known_umrf.getInputParameters())
  {
    ActionParameters::ParameterContainer new_param = known_umrf_param;
    const ActionParameters::ParameterContainer& param_in = umrf_node_in.getInputParameters().getParameter(known_umrf_param.getName());

    new_param.setAllowedData(param_in.getAllowedData());
    if (param_in.getDataSize() != 0)
    {
      new_param.setData(param_in.getData());
      new_param.setNativeData(param_in.getNativeData());
    }
    umrf_node_in.getInputParametersNc().setParameter(new_param, true);
  }

  return true;

  /*
   * TODO: Currently if a match is found then the function returns right away.
   * Yet again there could be multiple matching actions and hence, there should 
   * be some kind of metric to evaluate the best one to return.
   */ 
}
