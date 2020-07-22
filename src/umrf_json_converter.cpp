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

#include "temoto_action_engine/umrf_json_converter.h"
#include "temoto_action_engine/temoto_error.h"
#include <iostream>
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include <boost/algorithm/string.hpp>

namespace umrf_json_converter
{

Umrf fromUmrfJsonStr(const std::string& umrf_json_str, bool as_descriptor)
{
  rapidjson::Document json_doc;
  json_doc.Parse(umrf_json_str.c_str());

  if (json_doc.HasParseError())
  {
    throw CREATE_TEMOTO_ERROR_STACK("The provided JSON string contains syntax errors.");
  }

  // Parse the umrf json string to umrf datastructure
  Umrf umrf;

  /*
   * Parse the required fields
   */ 
  try
  {
    std::string name = getStringFromValue(getRootJsonElement(UMRF_FIELDS.name, json_doc));
    if (!umrf.setName(name))
    {
      throw CREATE_TEMOTO_ERROR_STACK("Illegal value in name field.");
    }

    std::string effect = getStringFromValue(getRootJsonElement(UMRF_FIELDS.effect, json_doc));
    if (!umrf.setEffect(effect))
    {
      throw CREATE_TEMOTO_ERROR_STACK("Illegal value in effect field.");
    }

    // If the UMRF is not used just as a descriptor of an action (as a part of an action package)
    // then the following fields are required
    if (as_descriptor)
    {
      std::string package_name = getStringFromValue(getRootJsonElement("package_name", json_doc));
      if (!umrf.setPackageName(package_name))
      {
        throw CREATE_TEMOTO_ERROR_STACK("Illegal value in package_name field.");
      }
    }
    else
    {
      unsigned int suffix = getNumberFromValue(getRootJsonElement(UMRF_FIELDS.suffix, json_doc));
      if (!umrf.setSuffix(suffix))
      {
        throw CREATE_TEMOTO_ERROR_STACK("Illegal value in suffix field.");
      }
    }
  }
  catch(TemotoErrorStack e)
  {
    throw FORWARD_TEMOTO_ERROR_STACK(e);
  }

  /*
   * Parse the not required fields
   * TODO: If the non-required field exists but is ill formatted, then raise an error
   */ 
  // Library path
  try
  {
    std::string lib_path = getStringFromValue(getRootJsonElement(UMRF_FIELDS.library_path, json_doc));
    umrf.setLibraryPath(lib_path);
  }
  catch(const TemotoErrorStack& e)
  {
    // ... do nothing
  }

  // Description
  try
  {
    std::string description = getStringFromValue(getRootJsonElement(UMRF_FIELDS.description, json_doc));
    if (!umrf.setDescription(description))
    {
      throw CREATE_TEMOTO_ERROR_STACK("Illegal value in description field.");
    }
  }
  catch(const TemotoErrorStack& e)
  {
    // ... do nothing
  }

  // Parents
  try
  {
    bool has_parents = false;
    try
    {
      getRootJsonElement(UMRF_FIELDS.parents, json_doc);
      has_parents = true;
    }
    catch(const std::exception& e)
    {
      // ... do nothing
    }
    
    if (has_parents)
    {
      std::vector<Umrf::Relation> parents = parseRelations(getRootJsonElement(UMRF_FIELDS.parents, json_doc));
      umrf.setParents(parents);
    }
  }
  catch(TemotoErrorStack e)
  {
    // If parents field is defined but its ill formated, then throw an error
    throw FORWARD_TEMOTO_ERROR_STACK(e);
  }

  // Children
  try
  {
    bool has_children = false;
    try
    {
      getRootJsonElement(UMRF_FIELDS.children, json_doc);
      has_children = true;
    }
    catch(const std::exception& e)
    {
      // ... do nothing
    }
    
    if (has_children)
    {
      std::vector<Umrf::Relation> children = parseRelations(getRootJsonElement(UMRF_FIELDS.children, json_doc));
      umrf.setChildren(children);
    }
  }
  catch(TemotoErrorStack e)
  {
    // If children field is defined but its ill formated, then throw an error
    throw FORWARD_TEMOTO_ERROR_STACK(e);
  }

  // Input parameters
  try
  {
    ActionParameters::Parameters input_parameters = parseParameters(getRootJsonElement(UMRF_FIELDS.input_parameters, json_doc), "");
    umrf.setInputParameters(input_parameters);
  }
  catch(const TemotoErrorStack& e)
  {
    // Just print the error
    //std::cerr << e.what() << '\n';
  }
  
  // Output parameters
  try
  {
    ActionParameters::Parameters output_parameters = parseParameters(getRootJsonElement(UMRF_FIELDS.output_parameters, json_doc), "");
    umrf.setOutputParameters(output_parameters);
  }
  catch(const TemotoErrorStack& e)
  {
    // Just print the error
    //std::cerr << e.what() << '\n';
  }

  return umrf;
}

std::string toUmrfJsonStr(const Umrf& umrf, bool as_descriptor)
{
  /*
   * Create UMRF JSON string from scratch.
   * reference: http://www.thomaswhitton.com/blog/2013/06/28/json-c-plus-plus-examples/
   */ 
  rapidjson::Document fromScratch; // document is the root of a json message
  fromScratch.SetObject(); // define the document as an object rather than an array

  // must pass an allocator when the object may need to allocate memory
  rapidjson::Document::AllocatorType& allocator = fromScratch.GetAllocator();

  /*
   * Fill the JSON datastructure according to UMRF format
   */ 

  // Set the name of the UMRF frame
  rapidjson::Value name_value(rapidjson::kStringType);
  name_value.SetString(umrf.getName().c_str(), umrf.getName().size(), allocator);
  fromScratch.AddMember(rapidjson::StringRef(UMRF_FIELDS.name), name_value, allocator);

  // Set the package name
  if (!umrf.getPackageName().empty())
  {
    rapidjson::Value package_name_value(rapidjson::kStringType);
    package_name_value.SetString(umrf.getPackageName().c_str(), umrf.getPackageName().size(), allocator);
    fromScratch.AddMember(rapidjson::StringRef(UMRF_FIELDS.package_name), package_name_value, allocator);
  }

  // Set the description name
  if (!umrf.getDescription().empty())
  {
    rapidjson::Value description_value(rapidjson::kStringType);
    description_value.SetString(umrf.getDescription().c_str(), umrf.getDescription().size(), allocator);
    fromScratch.AddMember(rapidjson::StringRef(UMRF_FIELDS.description), description_value, allocator);
  }

  // Set the suffix
  if (as_descriptor)
  {
    /* do not add the suffix field */
  }
  else
  {
    rapidjson::Value suffix_value(rapidjson::kNumberType);
    suffix_value.SetInt(umrf.getSuffix());
    fromScratch.AddMember(rapidjson::StringRef(UMRF_FIELDS.suffix), suffix_value, allocator);
  }

  // Set the notation
  if (!umrf.getNotation().empty())
  {
    rapidjson::Value notation_value(rapidjson::kStringType);
    notation_value.SetString(umrf.getNotation().c_str(), umrf.getNotation().size(), allocator);
    fromScratch.AddMember(rapidjson::StringRef(UMRF_FIELDS.notation), notation_value, allocator);
  }

  // Set the effect
  if (!umrf.getEffect().empty())
  {
    rapidjson::Value effect_value(rapidjson::kStringType);
    effect_value.SetString(umrf.getEffect().c_str(), umrf.getEffect().size(), allocator);
    fromScratch.AddMember(rapidjson::StringRef(UMRF_FIELDS.effect), effect_value, allocator);
  }

  // Set the input parameters via a rapidjson object type
  if (!umrf.getInputParameters().empty())
  {
    rapidjson::Value input_object(rapidjson::kObjectType);
    for (const auto& parameter : umrf.getInputParameters())
    {
      parseParameter(input_object, allocator, parameter);
    }
    fromScratch.AddMember(rapidjson::StringRef(UMRF_FIELDS.input_parameters), input_object, allocator);
  }

  // Set the output parameters via a rapidjson object type
  if (!umrf.getOutputParameters().empty())
  {
    rapidjson::Value output_object(rapidjson::kObjectType);
    for (const auto& parameter : umrf.getOutputParameters())
    {
      parseParameter(output_object, allocator, parameter);
    }
    fromScratch.AddMember(rapidjson::StringRef(UMRF_FIELDS.output_parameters), output_object, allocator);
  }

  // Set children
  if (!umrf.getChildren().empty())
  {
    rapidjson::Value children_object(rapidjson::kArrayType);
    for (const auto& child : umrf.getChildren())
    {
      rapidjson::Value child_object(rapidjson::kObjectType);
      //rapidjson::Value child_name_value = rapidjson::StringRef(child.getName().c_str());
      child_object.AddMember(rapidjson::StringRef(RELATION_FIELDS.name), rapidjson::StringRef(child.getName().c_str()), allocator);

      rapidjson::Value child_suffix_value(rapidjson::kNumberType);
      child_suffix_value.SetInt(child.getSuffix());
      child_object.AddMember(rapidjson::StringRef(RELATION_FIELDS.suffix), child_suffix_value, allocator);

      children_object.PushBack(child_object, allocator);
    }
    fromScratch.AddMember(rapidjson::StringRef(UMRF_FIELDS.children), children_object, allocator);
  }

  // Set parents
  if (!umrf.getParents().empty())
  {
    rapidjson::Value parents_object(rapidjson::kArrayType);
    for (const auto& parent : umrf.getParents())
    {
      rapidjson::Value parent_object(rapidjson::kObjectType);
      //rapidjson::Value parent_name_value = rapidjson::StringRef(parent.getName().c_str());
      parent_object.AddMember(rapidjson::StringRef(RELATION_FIELDS.name), rapidjson::StringRef(parent.getName().c_str()), allocator);

      rapidjson::Value parent_suffix_value(rapidjson::kNumberType);
      parent_suffix_value.SetInt(parent.getSuffix());
      parent_object.AddMember(rapidjson::StringRef(RELATION_FIELDS.suffix), parent_suffix_value, allocator);

      parents_object.PushBack(parent_object, allocator);
    }
    fromScratch.AddMember(rapidjson::StringRef(UMRF_FIELDS.parents), parents_object, allocator);
  }

  /*
   * Convert the JSON datastructure to a JSON string
   */
  rapidjson::StringBuffer strbuf;
  rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(strbuf);
  fromScratch.Accept(writer);
  return strbuf.GetString();
}

const rapidjson::Value& getRootJsonElement(const char* element_name, const rapidjson::Document& json_doc)
{
  if(!json_doc.HasMember(element_name))
  {
    throw CREATE_TEMOTO_ERROR_STACK("This JSON does not contain element '" + std::string(element_name) + "'");
  }
  return json_doc[element_name];
}

const rapidjson::Value& getJsonElement(const char* element_name, const rapidjson::Value& value_in)
{
  if(!value_in.HasMember(element_name))
  {
    throw CREATE_TEMOTO_ERROR_STACK("This field does not contain element '" + std::string(element_name) + "'");
  }
  return value_in[element_name];
}

std::string getStringFromValue(const rapidjson::Value& value)
{
  if (!value.IsString())
  {
    throw CREATE_TEMOTO_ERROR_STACK("This JSON value is not a string");
  }
  return value.GetString();
}

float getNumberFromValue(const rapidjson::Value& value)
{
  if (!value.IsNumber())
  {
    throw CREATE_TEMOTO_ERROR_STACK("This JSON value is not a number");
  }
  return value.GetFloat();
}

std::vector<Umrf::Relation> parseRelations(const rapidjson::Value& value_in)
{
  if (!value_in.IsArray())
  {
    throw CREATE_TEMOTO_ERROR_STACK("The relations field must be an array");
  }

  std::vector<Umrf::Relation> umrf_relations;
  for (rapidjson::SizeType i=0; i<value_in.Size(); i++)
  {
    try
    {
      std::string name = getStringFromValue(getJsonElement(RELATION_FIELDS.name, value_in[i]));
      unsigned int suffix = getNumberFromValue(getJsonElement(RELATION_FIELDS.suffix, value_in[i]));
      umrf_relations.emplace_back(name, suffix);
    }
    catch(TemotoErrorStack e)
    {
      throw FORWARD_TEMOTO_ERROR_STACK(e);
    }
  }

  return umrf_relations;
}

ActionParameters::Parameters parseParameters(const rapidjson::Value& value_in, std::string parent_member_name)
{
  if (!value_in.IsObject())
  {
    throw CREATE_TEMOTO_ERROR_STACK("The parameters were not formatted as JSON objects");
  }

  ActionParameters::Parameters action_parameters;
  if (value_in.HasMember(PVF_FIELDS.type))
  {
    /*
     * Get type
     */
    std::string type = value_in[PVF_FIELDS.type].GetString();
    ActionParameters::ParameterContainer pc(parent_member_name, type);

    /*
     * Get required
     */
    try
    {
      std::string required = getStringFromValue(getJsonElement(PVF_FIELDS.required, value_in));
      if (required == "true")
      {
        pc.setRequired(true);
      }
      else
      {
        pc.setRequired(false);
      }
    }
    catch(TemotoErrorStack e)
    {
      //std::cerr << e.what() << '\n';
    }

    /*
     * Get updatable
     */
    try
    {
      std::string updatable = getStringFromValue(getJsonElement(PVF_FIELDS.updatable, value_in));
      if (updatable == "true")
      {
        pc.setUpdatable(true);
      }
      else
      {
        pc.setUpdatable(false);
      }
    }
    catch(TemotoErrorStack e)
    {
      //std::cerr << e.what() << '\n';
    }

    /*
     * Get example
     */
    try
    {
      std::string example = getStringFromValue(getJsonElement(PVF_FIELDS.example, value_in));
      pc.setExample(example);
    }
    catch(TemotoErrorStack e)
    {
      //std::cerr << e.what() << '\n';
    }

    /*
     * Get value
     */ 
    try
    {
      if (type == "string")
      {
        std::string value = getStringFromValue(getJsonElement(PVF_FIELDS.value, value_in));
        pc.setData(boost::any(value));
      }
      else if (type == "number")
      {
        double value = getNumberFromValue(getJsonElement(PVF_FIELDS.value, value_in));
        pc.setData(boost::any(value));
      }
    }
    catch(TemotoErrorStack e)
    {
      //std::cerr << e.what() << '\n';
    }

    action_parameters.insert(pc);
  }
  else
  {
    for (const auto& member : value_in.GetObject())
    {
      std::string member_name;
      if (parent_member_name.empty())
      {
        member_name = member.name.GetString();
      }
      else
      {
        member_name = parent_member_name + "::" + member.name.GetString();
      }
      ActionParameters::Parameters ap_rec = parseParameters(member.value, member_name);
      action_parameters.insert(ap_rec.begin(), ap_rec.end());
    }
  }
  
  return action_parameters;
}

std::string removeFirstToken(const std::vector<std::string>& tokens_in)
{
  if (tokens_in.empty())
  {
    return std::string("");
  }
  else if (tokens_in.size() == 1)
  {
    return tokens_in.at(0);
  }
  else
  {
    std::string name;
    std::vector<std::string> name_tokens_renamed(tokens_in.begin() + 2, tokens_in.end());
  
    for (const auto& name_token : name_tokens_renamed)
    {
      if (name_token.empty())
      {
        continue;
      }
      else if (name.empty())
      {
        name += name_token;
      }
      else
      {
        name += "::" + name_token;
      }
    }
    return name;
  }
}

void parseParameter(
  rapidjson::Value& json_value,
  rapidjson::Document::AllocatorType& allocator,
  const ActionParameters::ParameterContainer& pc)
{
  // Split the name of the parameter into tokens
  std::vector<std::string> name_tokens;
  boost::split(name_tokens, pc.getName(), boost::is_any_of("::"));

  if (name_tokens.empty())
  {
    throw CREATE_TEMOTO_ERROR_STACK("No name tokens received");
  }

  // Extract the "first_token" and check if such "sub_json_object" exists in the "input_json_object"
  if (json_value.HasMember(name_tokens.at(0).c_str()))
  {
    // IF there are more tokens left then:
    if(name_tokens.size() > 1)
    {
      // Rename the "parameter" by removing the first token and leaving the rest as "renamed_parameter"
      ActionParameters::ParameterContainer renamed_param = pc;
      renamed_param.setName(removeFirstToken(name_tokens));

      // Recursively invoke parseParameter("sub_json_object", "renamed_parameter")
      parseParameter(json_value[name_tokens.at(0).c_str()], allocator, renamed_param);
    }
    else
    {
      // Throw an error, because it's a duplicate entry
      throw CREATE_TEMOTO_ERROR_STACK("Duplicate entry detected");
    }
  }
  else
  {
    if(name_tokens.size() > 1)
    {
      // Rename the "parameter" by removing the first token and leaving the rest as "renamed_parameter"
      ActionParameters::ParameterContainer renamed_param = pc;
      renamed_param.setName(removeFirstToken(name_tokens));

      // Create new json object and Recursively invoke parseParameter
      rapidjson::Value new_json_name(rapidjson::kStringType);
      new_json_name.SetString(name_tokens.at(0).c_str(), name_tokens.at(0).size(), allocator);

      rapidjson::Value new_json_value(rapidjson::kObjectType);
      parseParameter(new_json_value, allocator, renamed_param);

      json_value.AddMember(new_json_name, new_json_value, allocator);
      return;
    }
    else
    {
      // Parse the all the pvf_values of the parameter and insert them to the json object
      parsePvfFields(json_value, allocator, pc);
      return;
    }
  }
}

void parsePvfFields(
  rapidjson::Value& json_value,
  rapidjson::Document::AllocatorType& allocator,
  const ActionParameters::ParameterContainer& parameter)
{

  rapidjson::Value parameter_name(rapidjson::kStringType);
  parameter_name.SetString(parameter.getName().c_str(), parameter.getName().size(), allocator);

  rapidjson::Value pvf_value_json_value(rapidjson::kStringType);
  rapidjson::Value pvf_type_json_value(rapidjson::kStringType);
  rapidjson::Value pvf_example_json_value(rapidjson::kStringType);
  rapidjson::Value pvf_updatable_json_value(rapidjson::kStringType);
  pvf_value_json_value.SetString(PVF_FIELDS.value, allocator);
  pvf_type_json_value.SetString(PVF_FIELDS.type, allocator);
  pvf_example_json_value.SetString(PVF_FIELDS.example, allocator);
  pvf_updatable_json_value.SetString(PVF_FIELDS.updatable, allocator);

  rapidjson::Value parameter_value(rapidjson::kObjectType);
  if (parameter.getType() == "string")
  {
    parameter_value.AddMember(pvf_type_json_value, "string", allocator);

    if (parameter.getDataSize() != 0)
    {
      std::string pvf_value_str = boost::any_cast<std::string>(parameter.getData());
      rapidjson::Value pvf_value(rapidjson::kStringType);
      pvf_value.SetString(pvf_value_str.c_str(), pvf_value_str.size(), allocator);
      parameter_value.AddMember(pvf_value_json_value, pvf_value, allocator);
    }
  }
  else if (parameter.getType() == "number")
  {
    parameter_value.AddMember(pvf_type_json_value, "number", allocator);

    if (parameter.getDataSize() != 0)
    {
      double pvf_value_number = boost::any_cast<double>(parameter.getData());
      rapidjson::Value pvf_value(rapidjson::kNumberType);
      pvf_value.SetDouble(pvf_value_number);
      parameter_value.AddMember(pvf_value_json_value, pvf_value, allocator);
    }
  }
  else
  {
    rapidjson::Value pvf_type(rapidjson::kStringType);
    pvf_type.SetString(parameter.getType().c_str(), parameter.getType().size(), allocator);
    parameter_value.AddMember(pvf_type_json_value, pvf_type, allocator);
  }
  
  // Parse the updatablilty
  if (parameter.isUpdatable())
  {
    parameter_value.AddMember(pvf_updatable_json_value, "true", allocator);
  }

  // Parse the example
  if (!parameter.getExample().empty())
  {
    rapidjson::Value parameter_example(rapidjson::kStringType);
    parameter_example.SetString(parameter.getExample().c_str(), parameter.getExample().size(), allocator);
    parameter_value.AddMember(pvf_example_json_value, parameter_example, allocator);
  }

  json_value.AddMember(parameter_name, parameter_value, allocator);
}

}// umrf_json_converter namespace
