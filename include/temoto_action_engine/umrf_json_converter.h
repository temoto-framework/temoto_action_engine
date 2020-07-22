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

#ifndef TEMOTO_ACTION_ENGINE__UMRF_JSON_CONVERTER_H
#define TEMOTO_ACTION_ENGINE__UMRF_JSON_CONVERTER_H

#include <string>
#include <vector>
#include "temoto_action_engine/umrf.h"
#include "rapidjson/document.h"

namespace umrf_json_converter
{
static const struct UmrfFields
{
  const char* parents = "parents";
  const char* children = "children";
  const char* library_path = "library_path";
  const char* name = "name";
  const char* package_name = "package_name";
  const char* description = "description";
  const char* suffix = "id";
  const char* effect = "effect";
  const char* notation = "notation";
  const char* input_parameters = "input_parameters";
  const char* output_parameters = "output_parameters";
}UMRF_FIELDS;

static const struct PvfFields
{
  const char* type = "pvf_type";
  const char* value = "pvf_value";
  const char* example = "pvf_example";
  const char* required = "pvf_required";
  const char* updatable = "pvf_updatable";
}PVF_FIELDS;

static const struct RelationFields
{
  const char* name = "name";
  const char* suffix = "id";
}RELATION_FIELDS;

Umrf fromUmrfJsonStr(const std::string& umrf_json_str, bool as_descriptor = false);

std::string toUmrfJsonStr(const Umrf& umrf, bool as_descriptor = false);

const rapidjson::Value& getRootJsonElement(const char* element_name, const rapidjson::Document& json_doc);

const rapidjson::Value& getJsonElement(const char* element_name, const rapidjson::Value& value_in);

std::string getStringFromValue(const rapidjson::Value& value);

float getNumberFromValue(const rapidjson::Value& value);

std::vector<Umrf::Relation> parseRelations(const rapidjson::Value& value_in);

ActionParameters::Parameters parseParameters(const rapidjson::Value& value_in, std::string parent_member_name);

void parseParameter(
  rapidjson::Value& json_value,
  rapidjson::Document::AllocatorType& allocator,
  const ActionParameters::ParameterContainer& pc
);

void parsePvfFields(
  rapidjson::Value& json_value,
  rapidjson::Document::AllocatorType& allocator,
  const ActionParameters::ParameterContainer& parameter
);

}// umrf_json_converter namespace
#endif