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

#ifndef TEMOTO_ACTION_ENGINE__UMRF_VALIDATOR_H
#define TEMOTO_ACTION_ENGINE__UMRF_VALIDATOR_H

#include "umrf_node.h"
#include "umrf_graph.h"

/**
 * @brief Checks for conflicting configurations, such as
 * different parents of a child action having conficting execution conditions
 * TODO
 * 
 */

namespace umrf_validator
{

void validateRelation(const UmrfNode::Relation& rel);

void validateAction(const UmrfNode& u);

void validateGraph(const UmrfGraph& ug);

}

#endif