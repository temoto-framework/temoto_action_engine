/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright 2019 TeMoto Telerobotics
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

/* Author: Robert Valner */

#include "temoto_action_engine/action_match_finder.h"

bool ActionMatchFinder::findMatchingAction(Umrf& umrf_in, const std::vector<Umrf>& known_umrfs) const
{
  for (const auto& known_umrf : known_umrfs)
  {
    /*
     * Compare name
     */
    if (umrf_in.getName() != known_umrf.getName())
    {
      continue;
    }

    /*
     * Compare input parameters
     */
    bool input_params_match = true;
    for (const auto& umrf_in_input_param : umrf_in.getInputParameters())
    {
      if (!known_umrf.getInputParameters().hasParameter(umrf_in_input_param.getName()))
      {
        input_params_match = false;
        break;
      }
    }
    if (!input_params_match)
    {
      continue;
    }

    /*
     * Compare output parameters
     */
    bool output_params_match = true;
    for (const auto& umrf_in_output_param : umrf_in.getOutputParameters())
    {
      if (!known_umrf.getOutputParameters().hasParameter(umrf_in_output_param.getName()))
      {
        output_params_match = false;
        break;
      }
    }
    if (!output_params_match)
    {
      continue;
    }

    /*
     * Get the library path of the matching action
     */
    umrf_in.setLibraryPath(known_umrf.getLibraryPath());
    return true;

    /*
     * TODO: Currently if a match is found then the function returns right away.
     * Yet again there could be multiple matching actions and hence, there should 
     * be some kind of metric to evaluate the best one to return.
     */ 
  }
  return false;
}