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

#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

#include "temoto_action_engine/umrf_json_converter.h"
#include "temoto_action_engine/umrf_graph_fs.h"

using json = nlohmann::json;

namespace umrf_json
{

boost::any toParameterData(const std::string& type, const json& data_json)
{
  if (type == "string")
  {
    return boost::any(std::string{data_json});
  }
  else if (type == "strings")
  {
    return boost::any(data_json.get<std::vector<std::string>>());
  }
  else if (type == "number")
  {
    return boost::any(double{data_json});
  }
  else if (type == "numbers")
  {
    return boost::any(data_json.get<std::vector<double>>());
  }
  else if (type == "bool")
  {
    return boost::any(bool{data_json});
  }
  else if (type == "bools")
  {
    return boost::any(data_json.get<std::vector<bool>>());
  }
  else
  {
    throw CREATE_TEMOTO_ERROR_STACK("Unable to parse the 'value'");
  }
}

ActionParameters::Parameters parseParameters(std::string parent_member_name, const json& parameters_json)
{
  if(!parameters_json.is_object())
  {
    throw CREATE_TEMOTO_ERROR_STACK("The parameters field must be an object");
  }

  ActionParameters::Parameters action_parameters;
  if (parameters_json.contains(PVF_FIELDS.type))
  {
    /*
     * Get type
     */
    ActionParameters::ParameterContainer pc(parent_member_name, parameters_json.at(PVF_FIELDS.type));

    /*
     * Get required
     */
    if (parameters_json.contains(PVF_FIELDS.required))
    try
    {
      pc.setRequired(parameters_json.at(PVF_FIELDS.required));
    }
    catch(TemotoErrorStack e)
    {
      throw CREATE_TEMOTO_ERROR_STACK("Invalid 'pvf_required': " + std::string(e.what()));
    }

    /*
     * Get updatable
     */
    if (parameters_json.contains(PVF_FIELDS.updatable))
    try
    {
      pc.setUpdatable(parameters_json.at(PVF_FIELDS.updatable));
    }
    catch(TemotoErrorStack e)
    {
      throw CREATE_TEMOTO_ERROR_STACK("Invalid 'pvf_updatable': " + std::string(e.what()));
    }

    /*
     * Get allowed values
     */
    if (parameters_json.contains(PVF_FIELDS.allowed_values))
    try
    {
      for (const auto& allowed_value_json : parameters_json.at(PVF_FIELDS.allowed_values))
      {
        pc.addAllowedData(toParameterData(pc.getType(), allowed_value_json));
      }
    }
    catch(TemotoErrorStack e)
    {
      throw CREATE_TEMOTO_ERROR_STACK("Invalid 'pvf_allowed_values': " + std::string(e.what()));
    }

    /*
     * Get value
     */
    if (parameters_json.contains(PVF_FIELDS.value)) 
    try
    {
      pc.setData(toParameterData(pc.getType(), parameters_json.at(PVF_FIELDS.value)));
    }
    catch(TemotoErrorStack e)
    {
      throw CREATE_TEMOTO_ERROR_STACK("Invalid 'pvf_value' of parameter " + pc.getName() + ": " + std::string(e.what()));
    }

    action_parameters.insert(pc);
  }
  else
  {
    for (const auto& member_json : parameters_json.items())
    {
      std::string member_name{member_json.key()};
      if (!parent_member_name.empty())
      {
        member_name = parent_member_name + "::" + member_name;
      }
      
      ActionParameters::Parameters ap_rec = parseParameters(member_name, member_json.value());
      action_parameters.insert(ap_rec.begin(), ap_rec.end());
    }
  }
  
  return action_parameters;
}

std::vector<UmrfNode::Relation> parseParentRelations(const json& relations_json)
{
  if(!relations_json.is_array())
  {
    throw CREATE_TEMOTO_ERROR_STACK("The relations field must be an array");
  }

  std::vector<UmrfNode::Relation> relations;
  for (const auto& relation_json : relations_json)
  {
    UmrfNode::Relation relation;

    // GET NAME - required
    if (!relation_json.contains(RELATION_FIELDS.name))
    {
      throw CREATE_TEMOTO_ERROR_STACK("Relation must contain a name");
    }
    relation.name_ = relation_json.at(RELATION_FIELDS.name);

    // GET INSTANCE ID - not required if there are no duplicate instances
    try
    {
      // Default to '0'
      relation.instance_id_ = (relation_json.contains(RELATION_FIELDS.instance_id)) ? uint16_t (relation_json.at(RELATION_FIELDS.instance_id)) : 0;
    }
    catch(const std::exception& e)
    {
      throw CREATE_TEMOTO_ERROR_STACK("Invalid 'instance_id': " + std::string(e.what()));
    }

    // GET REQUIRED
    try
    {
      // Default to false
      relation.required_ = (relation_json.contains(RELATION_FIELDS.required)) ? bool(relation_json.at(RELATION_FIELDS.required)) : true;
    }
    catch(const std::exception& e)
    {
      throw CREATE_TEMOTO_ERROR_STACK("Invalid 'required':" + std::string(e.what()));
    }

    relations.push_back(relation);
  }

  return relations;
}

std::vector<UmrfNode::Relation> parseChildRelations(const json& relations_json)
{
  if(!relations_json.is_array())
  {
    throw CREATE_TEMOTO_ERROR_STACK("The relations field must be an array");
  }

  std::vector<UmrfNode::Relation> relations;
  for (const auto& relation_json : relations_json)
  {
    UmrfNode::Relation relation;

    // GET NAME
    if (!relation_json.contains(RELATION_FIELDS.name))
    {
      throw CREATE_TEMOTO_ERROR_STACK("Relation must contain a name");
    }
    relation.name_ = relation_json.at(RELATION_FIELDS.name);

    // GET INSTANCE ID - not required if there are no duplicate instances
    try
    {
      // Default to '0'
      relation.instance_id_ = (relation_json.contains(RELATION_FIELDS.instance_id)) ? uint16_t (relation_json.at(RELATION_FIELDS.instance_id)) : 0;
    }
    catch(const std::exception& e)
    {
      throw CREATE_TEMOTO_ERROR_STACK("Invalid 'instance_id': " + std::string(e.what()));
    }

    // GET CONDITION
    try
    {
      // Default to "always -> run"
      relation.condition_ = (relation_json.contains(RELATION_FIELDS.condition)) ? relation_json.at(RELATION_FIELDS.condition) : "always -> run";
    }
    catch(const std::exception& e)
    {
      throw CREATE_TEMOTO_ERROR_STACK("Invalid 'condition':" + std::string(e.what()));
    }

    relations.push_back(relation);
  }

  return relations;
}

UmrfNode fromUmrfJsonStr(const std::string& umrf_json_str)
try
{
  json uj = json::parse(umrf_json_str);
  UmrfNode un;

  /*
   * Parse the required fields
   */
  // GET NAME
  if (!un.setName(uj.at(UMRF_FIELDS.name)))
  {
    throw CREATE_TEMOTO_ERROR_STACK("Illegal value in 'name' field.");
  }

  // GET TYPE
  if (!un.setType(uj.at(UMRF_FIELDS.type)))
  {
    throw CREATE_TEMOTO_ERROR_STACK("Illegal value in 'type' field.");
  }

  /*
   * Parse the optional fields
   */

  // GET ACTOR
  if (uj.contains(UMRF_FIELDS.actor))
  try
  {
    un.setActor(uj.at(UMRF_FIELDS.actor));
  }
  catch(const std::exception& e)
  {
    throw CREATE_TEMOTO_ERROR_STACK("Invalid 'actor' in " + un.getName() + ": " + std::string(e.what()));
  }

  // GET DESCRIPTION
  if (uj.contains(UMRF_FIELDS.description))
  try
  {
    un.setDescription(uj.at(UMRF_FIELDS.description));
  }
  catch(const std::exception& e)
  {
    throw CREATE_TEMOTO_ERROR_STACK("Invalid 'description' in " + un.getName() + ": " + std::string(e.what()));
  }

  // GET ID
  if (uj.contains(UMRF_FIELDS.instance_id))
  try
  {
    un.setInstanceId(uj.at(UMRF_FIELDS.instance_id));
  }
  catch(const std::exception& e)
  {
    throw CREATE_TEMOTO_ERROR_STACK("Invalid 'instance_id' in " + un.getName() + ": " + std::string(e.what()));
  }

  // GET STATE
  if (uj.contains(UMRF_FIELDS.state))
  try
  {
    un.setState(uj.at(UMRF_FIELDS.state));
  }
  catch(const std::exception& e)
  {
    throw CREATE_TEMOTO_ERROR_STACK("Invalid 'state' in " + un.getName() + ": " + std::string(e.what()));
  }

  // GET PARENTS
  if (uj.contains(UMRF_FIELDS.parents))
  try
  {
    std::vector<UmrfNode::Relation> parents = parseParentRelations(uj[UMRF_FIELDS.parents]);
    un.setParents(parents);
  }
  catch(TemotoErrorStack e)
  {
    throw FORWARD_TEMOTO_ERROR_STACK_WMSG(e, "Invalid 'parents' in " + un.getName());
  }

  // GET CHILDREN
  if (uj.contains(UMRF_FIELDS.children))
  try
  {
    std::vector<UmrfNode::Relation> children = parseChildRelations(uj[UMRF_FIELDS.children]);
    un.setChildren(children);
  }
  catch(TemotoErrorStack e)
  {
    throw FORWARD_TEMOTO_ERROR_STACK_WMSG(e, "Invalid 'children' in " + un.getName());
  }

  // GET INPUT PARAMETERS
  if (uj.contains(UMRF_FIELDS.input_parameters))
  try
  {
    un.setInputParameters(parseParameters("", uj[UMRF_FIELDS.input_parameters]));
  }
  catch(TemotoErrorStack e)
  {
    throw FORWARD_TEMOTO_ERROR_STACK_WMSG(e, "Invalid 'input_parameters' in " + un.getName());
  }

  // GET OUTPUT PARAMETERS
  if (uj.contains(UMRF_FIELDS.output_parameters))
  try
  {
    un.setOutputParameters(parseParameters("", uj[UMRF_FIELDS.output_parameters]));
  }
  catch(TemotoErrorStack e)
  {
    throw FORWARD_TEMOTO_ERROR_STACK_WMSG(e, "Invalid 'output_parameters' in " + un.getName());
  }

  return un;
}
catch(TemotoErrorStack e)
{
  throw FORWARD_TEMOTO_ERROR_STACK(e);
}
catch(const std::exception& e)
{
  throw CREATE_TEMOTO_ERROR_STACK("UMRF JSON error: " + std::string(e.what()));
}

UmrfGraph fromUmrfGraphJsonStr(const std::string& ug_json_str)
try
{
  json ug_json = json::parse(ug_json_str);

  /*
   * Parse the required fields
   */
  std::string graph_name{ug_json[GRAPH_FIELDS.name]};
  std::vector<UmrfNode> umrf_actions;

  for (auto& umrf_json : ug_json[GRAPH_FIELDS.actions])
  {
    umrf_actions.push_back(fromUmrfJsonStr(umrf_json.dump()));
  }

  /*
   * Parse graph entry and exit actions
   */
  std::vector<UmrfNode::Relation> graph_entry, graph_exit;
  
  // Graph entry and exit doesn't have to be explicitly defined if graph contains only one action 
  if (umrf_actions.size() == 1)
  {
    UmrfNode::Relation r_entry, r_exit;

    r_entry.condition_ = "always -> run";
    r_entry.name_ = umrf_actions.front().getName();
    r_entry.instance_id_ = umrf_actions.front().getInstanceId();
    graph_entry.push_back(r_entry);

    r_exit.required_ = true;
    r_exit.name_ = umrf_actions.front().getName();
    r_exit.instance_id_ = umrf_actions.front().getInstanceId();
    graph_exit.push_back(r_exit);
  }
  else
  {
    graph_entry = parseChildRelations(ug_json[GRAPH_FIELDS.entry]);
    graph_exit = parseParentRelations(ug_json[GRAPH_FIELDS.exit]);
  }

  // Create a new action that represents the entry 
  UmrfNode graph_entry_action;
  graph_entry_action.setName(GRAPH_FIELDS.entry);
  graph_entry_action.setChildren(graph_entry);

  // Get the input parameters for the entry
  for (const auto& a : graph_entry)
  {
    const auto& it = std::find_if(
      umrf_actions.begin(),
      umrf_actions.end(),
      [&](const UmrfNode& u)
      {
        return u.getFullName() == a.getFullName();
      });

    if (it == umrf_actions.end())
    {
      throw CREATE_TEMOTO_ERROR_STACK("Graph entry '" + a.getFullName() + "' not found in the list of actions");
    }
    graph_entry_action.setInputParameters(it->getInputParameters());
  }

  umrf_actions.push_back(graph_entry_action);

  // Create a new action that represents the exit
  UmrfNode graph_exit_action;
  graph_exit_action.setName(GRAPH_FIELDS.exit);
  graph_exit_action.setParents(graph_exit);

  // Get the output parameters for the exit
  for (const auto& a : graph_exit)
  {
    const auto& it = std::find_if(
      umrf_actions.begin(),
      umrf_actions.end(),
      [&](const UmrfNode& u)
      {
        return u.getFullName() == a.getFullName();
      });

    if (it == umrf_actions.end())
    {
      throw CREATE_TEMOTO_ERROR_STACK("Graph exit '" + a.getFullName() + "' not found in the list of actions");
    }
    graph_exit_action.setOutputParameters(it->getOutputParameters());
  }

  umrf_actions.push_back(graph_exit_action);

  // Now we can greate the UmrfGraph datastructure
  UmrfGraph umrf_graph(graph_name, umrf_actions);

  /*
   * Parse the optional fields
   */

  // GET DESCRIPTION
  if (ug_json.contains(GRAPH_FIELDS.description))
  try
  {
    umrf_graph.setDescription(ug_json.at(GRAPH_FIELDS.description));
  }
  catch(const std::exception& e)
  {
    throw CREATE_TEMOTO_ERROR_STACK("Invalid 'description' in " + umrf_graph.getName() + ": " + std::string(e.what()));
  }

  return umrf_graph;
}
catch(TemotoErrorStack e)
{
  throw FORWARD_TEMOTO_ERROR_STACK(e);
}
catch(const std::exception& e)
{
  throw CREATE_TEMOTO_ERROR_STACK("The provided JSON string contains syntax errors: " + std::string(e.what()));
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

json parameterValueToJson(const std::string& type, const ActionParameters::Payload& data)
{
  json val_json;

  if (type == "string")
  {
    val_json = boost::any_cast<std::string>(data);
  }
  else if (type == "strings")
  {
    val_json = boost::any_cast<std::vector<std::string>>(data);
  }
  else if (type == "number")
  {
    val_json = boost::any_cast<double>(data);
  }
  else if (type == "numbers")
  {
    val_json = boost::any_cast<std::vector<double>>(data);
  }
  else if (type == "bool")
  {
    val_json = boost::any_cast<bool>(data);
  }
  else if (type == "bools")
  {
    val_json = boost::any_cast<std::vector<bool>>(data);
  }

  return val_json;
}

json parameterFieldsToJson(const ActionParameters::ParameterContainer& p)
{
  json p_json;
  
  auto isNative = [&](const std::string& type) 
  { 
    for (const auto& native_datatype : NATIVE_JSON_TYPES)
    {
      if (native_datatype == type) return true;
    }
    return false;
  };

  p_json[PVF_FIELDS.type] = p.getType();
  if (p.getDataSize() != 0 && isNative(p.getType()))
  {
    p_json[PVF_FIELDS.value] = parameterValueToJson(p.getType(), p.getData());
  }
  
  // Parse the updatablilty
  if (p.isUpdatable())
  {
    p_json[PVF_FIELDS.updatable] = true;
  }

  // Parse allowed values
  if (!p.getAllowedData().empty() && isNative(p.getType()))
  {
    p_json[PVF_FIELDS.allowed_values] = json::array();
    for (const auto& allowed_value : p.getAllowedData())
    {
      p_json[PVF_FIELDS.allowed_values].push_back(parameterValueToJson(p.getType(), allowed_value));
    }
  }

  return p_json;
} 

void parameterToJson(const ActionParameters::ParameterContainer& p, json& parent_json)
{
  // Split the name of the parameter into tokens
  std::vector<std::string> name_tokens;
  boost::split(name_tokens, p.getName(), boost::is_any_of("::"));

  if (name_tokens.empty())
  {
    throw CREATE_TEMOTO_ERROR_STACK("No name tokens received");
  }

  // Extract the "first_token" and check if such "sub_json_object" exists in the "input_json_object"
  if (parent_json.contains(name_tokens.at(0)))
  {
    // IF there are more tokens left then:
    if(name_tokens.size() > 1)
    {
      // Rename the "parameter" by removing the first token and leaving the rest as "renamed_parameter"
      ActionParameters::ParameterContainer renamed_param = p;
      renamed_param.setName(removeFirstToken(name_tokens));

      // Recursively invoke parseParameter("sub_json_object", "renamed_parameter")
      parameterToJson(renamed_param, parent_json[name_tokens.at(0)]);
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
      ActionParameters::ParameterContainer renamed_param = p;
      renamed_param.setName(removeFirstToken(name_tokens));

      // Create new json object and Recursively invoke parseParameter
      json nested_parent_json;
      parameterToJson(renamed_param, nested_parent_json);

      parent_json[name_tokens.at(0)] = nested_parent_json;
      return;
    }
    else
    {
      // Parse the all the pvf_values of the parameter and insert them to the json object
      parent_json[p.getName()] = parameterFieldsToJson(p);
      return;
    }
  }
}

json parametersToJson(const ActionParameters& parameters)
{
  json parameters_json;
  for (const auto& parameter : parameters)
  {
    parameterToJson(parameter, parameters_json);
  }
  return parameters_json;
}

json toUmrfJson(const UmrfNode& u)
{
  json action;
  action[UMRF_FIELDS.name] = u.getName();
  action[UMRF_FIELDS.actor] = u.getActor();
  action[UMRF_FIELDS.instance_id] = u.getInstanceId();
  action[UMRF_FIELDS.description] = u.getDescription();
  action[UMRF_FIELDS.type] = u.getType();

  // parse input parameters
  if (u.getInputParameters().getParameters().size() != 0)
  {
    action[UMRF_FIELDS.input_parameters] = parametersToJson(u.getInputParameters());
  }

  // parse output parameters
  if (u.getOutputParameters().getParameters().size() != 0)
  {
    action[UMRF_FIELDS.output_parameters] = parametersToJson(u.getOutputParameters());
  }

  // parse parents
  if (u.getParents().size() != 0)
  {
    action[UMRF_FIELDS.parents] = json::array();
    for (const auto& parent : u.getParents())
    {
      json parent_j;
      parent_j[RELATION_FIELDS.name] = parent.getName();
      parent_j[RELATION_FIELDS.instance_id] = parent.getInstanceId();
      parent_j[RELATION_FIELDS.required] = parent.getRequired();

      action[UMRF_FIELDS.parents].push_back(parent_j);
    }
  }

  // parse children
  if (u.getChildren().size() != 0)
  {
    action[UMRF_FIELDS.children] = json::array();
    for (const auto& child : u.getChildren())
    {
      json child_j;
      child_j[RELATION_FIELDS.name] = child.getName();
      child_j[RELATION_FIELDS.instance_id] = child.getInstanceId();
      child_j[RELATION_FIELDS.condition] = child.getCondition();

      action[UMRF_FIELDS.children].push_back(child_j);
    }
  }
  
  return action;
}

std::string toUmrfJsonStr(const UmrfNode& u)
{
  return toUmrfJson(u).dump(4);
}

std::string toUmrfGraphJsonStr(const UmrfGraph& ug)
{
  /*
   * TODO: use ordered_json: https://json.nlohmann.me/api/ordered_json/
   */
  json ug_json;
  ug_json[GRAPH_FIELDS.name] = ug.getName();
  ug_json[GRAPH_FIELDS.description] = ug.getDescription();
  
  // parse graph entry
  ug_json[GRAPH_FIELDS.entry] = json::array();
  for (const auto& child : ug.getUmrfNode(GRAPH_FIELDS.entry + std::string("_0"))->getChildren())
  {
    json child_json;
    child_json[RELATION_FIELDS.name] = child.getName();
    child_json[RELATION_FIELDS.instance_id] = std::to_string(child.getInstanceId());
    ug_json[GRAPH_FIELDS.entry].push_back(child_json);
  }

  // parse graph exit
  ug_json[GRAPH_FIELDS.exit] = json::array();
  for (const auto& parent : ug.getUmrfNode(GRAPH_FIELDS.exit + std::string("_0"))->getParents())
  {
    json parent_json;
    parent_json[RELATION_FIELDS.name] = parent.getName();
    parent_json[RELATION_FIELDS.required] = parent.getRequired();
    parent_json[RELATION_FIELDS.instance_id] = std::to_string(parent.getInstanceId());
    ug_json[GRAPH_FIELDS.exit].push_back(parent_json);
  }

  // parse the actions
  ug_json[GRAPH_FIELDS.actions] = json::array();
  for (const auto& u : ug.getUmrfNodes())
  {
    if (u.getName() != GRAPH_FIELDS.entry && u.getName() != GRAPH_FIELDS.exit)
    {
      ug_json[GRAPH_FIELDS.actions].push_back(toUmrfJson(u));
    }
  }

  return ug_json.dump(4);
}

ActionParameters::Parameters fromUmrfParametersJsonStr(const std::string umrf_param_json_str)
{
  return parseParameters("", json::parse(umrf_param_json_str));
}

} // namespace umrf_json

// void printParameters(const ActionParameters params)
// {
//   for (const auto& parameter : params)
//   {
//     std::cout << "   - name: " << parameter.getName() << std::endl;
//     std::cout << "     type: " << parameter.getType() << std::endl;

//     if (parameter.getDataSize() != 0)
//     {
//       const auto& data = parameter.getData();
//       if (parameter.getType() == "string")
//       {
//         std::cout << "     value: " << boost::any_cast<std::string>(data) << std::endl;
//       }
//       else if (parameter.getType() == "strings")
//       {
//         std::cout << "     value: ";
//         for (const auto& data_item : boost::any_cast<std::vector<std::string>>(data))
//         {
//           std::cout << data_item << ", ";
//         }
//         std::cout << std::endl;
//       }
//       else if (parameter.getType() == "number")
//       {
//         std::cout << "     value: " << boost::any_cast<double>(data) << std::endl;
//       }
//       else if (parameter.getType() == "numbers")
//       {
//         std::cout << "     value: ";
//         for (const auto& data_item : boost::any_cast<std::vector<double>>(data))
//         {
//           std::cout << data_item << ", ";
//         }
//         std::cout << std::endl;
//       }
//       else if (parameter.getType() == "bool")
//       {
//         std::cout << "     value: " << boost::any_cast<bool>(data) << std::endl;
//       }
//       else if (parameter.getType() == "bools")
//       {
//         std::cout << "     value: ";
//         for (const auto& data_item : boost::any_cast<std::vector<bool>>(data))
//         {
//           std::cout << data_item << ", ";
//         }
//         std::cout << std::endl;
//       }
//     }

//     if (parameter.getAllowedData().size() != 0)
//     {
//       std::cout << "     allowed_values: ";
//       for (const auto& allowed_data : parameter.getAllowedData())
//       {
//         if (parameter.getType() == "string")
//         {
//           std::cout << boost::any_cast<std::string>(allowed_data) << ", ";
//         }
//         else if (parameter.getType() == "number")
//         {
//           std::cout << boost::any_cast<double>(allowed_data) << ", ";
//         }
//         else if (parameter.getType() == "bool")
//         {
//           std::cout << boost::any_cast<bool>(allowed_data) << ", ";
//         }
//       }
//       std::cout << std::endl;
//     }
//   }
// }

// int main()
// {
//   std::string ug_json_str = temoto_action_engine::readFromFile("example_b.json");
//   UmrfGraph ug = umrf_json::fromUmrfGraphJsonStr(ug_json_str);
//   std::string ug_json_str_new = umrf_json::toUmrfGraphJsonStr(ug);

//   std::cout << "graph_name: " << ug.getName() << std::endl;
//   std::cout << "graph_description: " << ug.getDescription() << std::endl;
//   std::cout << "actions: " << std::endl;

//   for (const auto& action : ug.getUmrfNodes())
//   {
//     std::cout << " - name: " << action.getName() << std::endl;
//     std::cout << "   instance_id: " << action.getInstanceId() << std::endl;
//     std::cout << "   type: " << action.getType() << std::endl;

//     if (!action.getActor().empty())
//     {
//       std::cout << "   actor: " << action.getActor() << std::endl;
//     }

//     if (!action.getDescription().empty())
//     {
//       std::cout << "   description: " << action.getDescription() << std::endl;
//     }

//     if (!action.getParents().empty())
//     {
//       std::cout << "   parents: " << std::endl;

//       for (const auto& parent : action.getParents())
//       {
//         std::cout << "   - name: " << parent.getName() << std::endl;
//         std::cout << "     instance_id: " << parent.getInstanceId() << std::endl;
//         std::cout << "     required: " << parent.getRequired() << std::endl;
//       }
//     }

//     if (!action.getChildren().empty())
//     {
//       std::cout << "   children: " << std::endl;

//       for (const auto& child : action.getChildren())
//       {
//         std::cout << "   - name: " << child.getName() << std::endl;
//         std::cout << "     instance_id: " << child.getInstanceId() << std::endl;
//         std::cout << "     condition: " << child.getCondition() << std::endl;
//       }
//     }

//     if (!action.getInputParameters().empty())
//     {
//       std::cout << "   input_parameters: " << std::endl;
//       printParameters(action.getInputParameters());
//     }

//     if (!action.getOutputParameters().empty())
//     {
//       std::cout << "   output_parameters: " << std::endl;
//       printParameters(action.getOutputParameters());
//     }
//   }

//   std::cout << std::endl;
//   std::cout << ug_json_str_new << std::endl;
// }