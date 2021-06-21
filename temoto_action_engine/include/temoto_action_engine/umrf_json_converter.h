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

#ifndef TEMOTO_ACTION_ENGINE__UMRF_JSON_CONVERTER_H
#define TEMOTO_ACTION_ENGINE__UMRF_JSON_CONVERTER_H

#include <string>
#include <vector>
#include "temoto_action_engine/umrf_node.h"
#include "temoto_action_engine/umrf_graph.h"

namespace umrf_json_converter
{

static const struct UmrfGraphFields
{
  const char* name = "graph_name";
  const char* description = "graph_description";
  const char* state = "graph_state";
  const char* umrf_actions = "umrf_actions";
}UMRF_GRAPH_FIELDS;

static const struct UmrfFields
{
  const char* parents = "parents";
  const char* children = "children";
  const char* library_path = "library_path";
  const char* state = "state";
  const char* name = "name";
  const char* package_name = "package_name";
  const char* description = "description";
  const char* suffix = "id";
  const char* effect = "effect";
  const char* notation = "notation";
  const char* input_parameters = "input_parameters";
  const char* output_parameters = "output_parameters";
  const char* execute_first = "execute_first";
}UMRF_FIELDS;

static const struct PvfFields
{
  const char* type = "pvf_type";
  const char* value = "pvf_value";
  const char* example = "pvf_example";
  const char* required = "pvf_required";
  const char* updatable = "pvf_updatable";
  const char* allowed_values = "pvf_allowed_values";
}PVF_FIELDS;

static const struct RelationFields
{
  const char* name = "name";
  const char* suffix = "id";
  const char* required = "required";
}RELATION_FIELDS;

UmrfNode fromUmrfJsonStr(const std::string& umrf_json_str, bool as_descriptor = false);

UmrfGraph fromUmrfGraphJsonStr(const std::string& umrf_graph_json_str);

ActionParameters::Parameters fromUmrfParametersJsonStr(const std::string umrf_param_json_str);

std::string toUmrfJsonStr(const UmrfNode& umrf_node, bool as_descriptor = false);

std::string toUmrfGraphJsonStr(const UmrfGraph& umrf_graph);

}// umrf_json_converter namespace
#endif