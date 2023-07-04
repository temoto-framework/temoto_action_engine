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

#ifndef TEMOTO_ACTION_ENGINE__umrf_json
#define TEMOTO_ACTION_ENGINE__umrf_json

#include "temoto_action_engine/umrf_node.h"
#include "temoto_action_engine/umrf_graph.h"

namespace umrf_json
{

static const struct GraphFields
{
  const char* actions = "actions";
  const char* description = "graph_description";
  const char* entry = "graph_entry";
  const char* exit = "graph_exit";
  const char* name = "graph_name";
  const char* state = "graph_state";
}GRAPH_FIELDS;

static const struct UmrfFields
{
  const char* parents = "parents";
  const char* children = "children";
  const char* state = "state";
  const char* name = "name";
  const char* actor = "actor";
  const char* description = "description";
  const char* instance_id = "instance_id";
  const char* type = "type";
  const char* input_parameters = "input_parameters";
  const char* output_parameters = "output_parameters";
}UMRF_FIELDS;

static const struct PvfFields
{
  const char* type = "pvf_type";
  const char* value = "pvf_value";
  const char* required = "pvf_required";
  const char* updatable = "pvf_updatable";
  const char* allowed_values = "pvf_allowed_values";
}PVF_FIELDS;

static const struct RelationFields
{
  const char* name = "name";
  const char* instance_id = "instance_id";
  const char* required = "required";
  const char* conditions = "conditions";
  const char* precondition = "precondition";
  const char* response = "response";
}RELATION_FIELDS;

const std::vector<std::string> NATIVE_JSON_TYPES{
  "string",
  "strings",
  "number",
  "numbers",
  "bool",
  "bools"
};

UmrfNode fromUmrfJsonStr(const std::string& umrf_json_str);

UmrfGraph fromUmrfGraphJsonStr(const std::string& ug_json_str);

ActionParameters::Parameters fromUmrfParametersJsonStr(const std::string umrf_param_json_str);

std::string toUmrfJsonStr(const UmrfNode& u);

std::string toUmrfGraphJsonStr(const UmrfGraph& ug);

}

#endif