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
      std::string suffix = getStringFromValue(getRootJsonElement(UMRF_FIELDS.suffix, json_doc));
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
    // Just print the error
    //std::cout << e.what() << '\n';
  }

  // Parents
  try
  {
    std::vector<std::string> parents = getStringVectorFromValue(getRootJsonElement(UMRF_FIELDS.parents, json_doc));
    //umrf.setParents(parents);
  }
  catch(const TemotoErrorStack& e)
  {
    // Just print the error
    //std::cout << e.what() << '\n';
  }

  // Children
  try
  {
    std::vector<std::string> children = getStringVectorFromValue(getRootJsonElement(UMRF_FIELDS.children, json_doc));
    //umrf.setChildren(children);
  }
  catch(const TemotoErrorStack& e)
  {
    // Just print the error
    //std::cout << e.what() << '\n';
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

std::string toUmrfJsonStr(const Umrf& umrf)
{
  return "";
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

std::vector<std::string> getStringVectorFromValue(const rapidjson::Value& value)
{
  std::vector<std::string> string_vector;
  if (!value.IsArray())
  {
    throw CREATE_TEMOTO_ERROR_STACK("The parents/children field must be an array");
  }
  for (rapidjson::SizeType i=0; i<value.Size(); i++)
  {
    try
    {
      std::string name = getStringFromValue(getJsonElement(UMRF_FIELDS.name, value[i]));
      std::string suffix = getStringFromValue(getJsonElement(UMRF_FIELDS.suffix, value[i]));
      string_vector.push_back(name + "_" + suffix);
    }
    catch(TemotoErrorStack e)
    {
      throw FORWARD_TEMOTO_ERROR_STACK(e);
    }
  }
  return string_vector;
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
        float value = getNumberFromValue(getJsonElement(PVF_FIELDS.value, value_in));
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

}// umrf_json_converter namespace