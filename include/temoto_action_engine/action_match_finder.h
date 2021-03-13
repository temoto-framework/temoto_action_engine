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

#ifndef TEMOTO_ACTION_ENGINE__ACTION_MATCH_FINDER_H
#define TEMOTO_ACTION_ENGINE__ACTION_MATCH_FINDER_H

#include "temoto_action_engine/umrf_node.h"

/**
 * @brief A helper class that tries to find a suitable action from a list of known actions based on given UMRF.
 * 
 */
class ActionMatchFinder
{
public:
  bool findMatchingAction(UmrfNode& umrf_node_in, const std::vector<UmrfNode>& known_umrfs, bool name_match = false) const;
};

#endif