#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

#include "temoto_action_engine/umrf_node.h"
#include "temoto_action_engine/umrf_graph.h"
#include "temoto_action_engine/umrf_graph_fs.h"

using json = nlohmann::json;

// TODO - make it generic and not depend on pvf_value. 
boost::any toParameterData(const std::string& type, const json& data_json)
{
  if (type == "string")
  {
    return boost::any(std::string{data_json.at("pvf_value")});
  }
  else if (type == "number")
  {
    return boost::any(double{data_json.at("pvf_value")});
  }
  else if (type == "bool")
  {
    return boost::any(bool{data_json.at("pvf_value")});
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
  if (parameters_json.contains("pvf_type"))
  {
    /*
     * Get type
     */
    ActionParameters::ParameterContainer pc(parent_member_name, parameters_json.at("pvf_type"));

    /*
     * Get required
     */
    if (parameters_json.contains("pvf_required"))
    try
    {
      pc.setRequired(parameters_json.at("pvf_required"));
    }
    catch(TemotoErrorStack e)
    {
      throw CREATE_TEMOTO_ERROR_STACK("Invalid 'pvf_required': " + std::string(e.what()));
    }

    /*
     * Get updatable
     */
    if (parameters_json.contains("pvf_updatable"))
    try
    {
      pc.setUpdatable(parameters_json.at("pvf_updatable"));
    }
    catch(TemotoErrorStack e)
    {
      throw CREATE_TEMOTO_ERROR_STACK("Invalid 'pvf_updatable': " + std::string(e.what()));
    }

    /*
     * Get allowed values
     */
    // if (parameters_json.contains("pvf_allowed_values"))
    // try
    // {
    //   for (const auto& allowed_value_json : parameters_json.at("pvf_allowed_values").)
    //   {
    //     pc.addAllowedData(toParameterData(pc.getType(), allowed_value_json));
    //   }
    // }
    // catch(TemotoErrorStack e)
    // {
    //   throw CREATE_TEMOTO_ERROR_STACK("Invalid 'pvf_allowed_values': " + std::string(e.what()));
    // }

    /*
     * Get value
     */
    if (parameters_json.contains("pvf_value")) 
    try
    {
      pc.setData(toParameterData(pc.getType(), parameters_json));
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

    // GET NAME
    if (!relation_json.contains("name"))
    {
      throw CREATE_TEMOTO_ERROR_STACK("Relation must contain a name");
    }
    relation.name_ = relation_json.at("name");

    // GET ID
    if (!relation_json.contains("id"))
    {
      throw CREATE_TEMOTO_ERROR_STACK("Relation must contain an ID");
    }
    relation.suffix_ = relation_json.at("id");

    // GET REQUIRED
    try
    {
      // Default to false
      relation.required_ = (relation_json.contains("required")) ? bool(relation_json.at("required")) : false;
    }
    catch(const std::exception& e)
    {
      throw CREATE_TEMOTO_ERROR_STACK("Invalid 'preempted_by':" + std::string(e.what()));
    }

    // GET PRE-EMPTED
    try
    {
      // Default to false
      relation.stop_when_received_ = (relation_json.contains("preempted_by")) ? bool(relation_json.at("preempted_by")) : false;
    }
    catch(const std::exception& e)
    {
      throw CREATE_TEMOTO_ERROR_STACK("Invalid 'preempted_by':" + std::string(e.what()));
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
    if (!relation_json.contains("name"))
    {
      throw CREATE_TEMOTO_ERROR_STACK("Relation must contain a name");
    }
    relation.name_ = relation_json.at("name");

    // GET ID
    if (!relation_json.contains("id"))
    {
      throw CREATE_TEMOTO_ERROR_STACK("Relation must contain an ID");
    }
    relation.suffix_ = relation_json.at("id");

    relations.push_back(relation);
  }

  return relations;
}

UmrfNode fromUmrfJson(const json& umrf_json)
try
{
  UmrfNode un;

  /*
   * Parse the required fields
   */
  // GET NAME
  if (!un.setName(umrf_json.at("name")))
  {
    throw CREATE_TEMOTO_ERROR_STACK("Illegal value in 'name' field.");
  }

  // GET TYPE
  if (!un.setEffect(umrf_json.at("effect")))
  {
    throw CREATE_TEMOTO_ERROR_STACK("Illegal value in 'effect' field.");
  }

  /*
   * Parse the optional fields
   */

  // GET DESCRIPTION
  if (umrf_json.contains("description"))
  try
  {
    un.setDescription(umrf_json.at("description"));
  }
  catch(const std::exception& e)
  {
    throw CREATE_TEMOTO_ERROR_STACK("Invalid 'description' in " + un.getName() + ": " + std::string(e.what()));
  }

  // GET ID
  if (umrf_json.contains("id"))
  try
  {
    un.setSuffix(umrf_json.at("id"));
  }
  catch(const std::exception& e)
  {
    throw CREATE_TEMOTO_ERROR_STACK("Invalid 'id' in " + un.getName() + ": " + std::string(e.what()));
  }

  // GET STATE
  if (umrf_json.contains("state"))
  try
  {
    un.setState(umrf_json.at("state"));
  }
  catch(const std::exception& e)
  {
    throw CREATE_TEMOTO_ERROR_STACK("Invalid 'state' in " + un.getName() + ": " + std::string(e.what()));
  }

  // GET PARENTS
  if (umrf_json.contains("parents"))
  try
  {
    std::vector<UmrfNode::Relation> parents = parseParentRelations(umrf_json["parents"]);
    un.setParents(parents);
  }
  catch(TemotoErrorStack e)
  {
    throw FORWARD_TEMOTO_ERROR_STACK_WMSG(e, "Invalid 'parents' in " + un.getName());
  }

  // GET CHILDREN
  if (umrf_json.contains("children"))
  try
  {
    std::vector<UmrfNode::Relation> children = parseChildRelations(umrf_json["children"]);
    un.setChildren(children);
  }
  catch(TemotoErrorStack e)
  {
    throw FORWARD_TEMOTO_ERROR_STACK_WMSG(e, "Invalid 'children' in " + un.getName());
  }

  // GET INPUT PARAMETERS
  if (umrf_json.contains("input_parameters"))
  try
  {
    un.setInputParameters(parseParameters("", umrf_json["input_parameters"]));
  }
  catch(TemotoErrorStack e)
  {
    throw FORWARD_TEMOTO_ERROR_STACK_WMSG(e, "Invalid 'input_parameters' in " + un.getName());
  }

  // GET OUTPUT PARAMETERS
  if (umrf_json.contains("output_parameters"))
  try
  {
    un.setOutputParameters(parseParameters("", umrf_json["output_parameters"]));
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
  std::string graph_name{ug_json["graph_name"]};
  std::vector<UmrfNode> umrf_actions;

  for (auto& umrf_json : ug_json["actions"])
  {
    umrf_actions.push_back(fromUmrfJson(umrf_json));
  }

  UmrfGraph umrf_graph(graph_name, umrf_actions);

  /*
   * Parse the optional fields
   */

  // GET DESCRIPTION
  if (ug_json.contains("graph_description"))
  try
  {
    umrf_graph.setDescription(ug_json.at("graph_description"));
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

int main()
{
  std::string ug_json_str = temoto_action_engine::readFromFile("example.json");
  UmrfGraph ug = fromUmrfGraphJsonStr(ug_json_str);

  std::cout << "graph_name: " << ug.getName() << std::endl;
  std::cout << "graph_description: " << ug.getDescription() << std::endl;
  std::cout << "actions: " << std::endl;

  for (const auto& action : ug.getUmrfNodes())
  {
    std::cout << " - name: " << action.getName() << std::endl;
    std::cout << "   id: " << action.getSuffix() << std::endl;
    std::cout << "   type: " << action.getEffect() << std::endl;

    if (!action.getDescription().empty())
    {
      std::cout << "   description: " << action.getDescription() << std::endl;
    }

    if (!action.getParents().empty())
    {
      std::cout << "   parents: " << std::endl;

      for (const auto& parent : action.getParents())
      {
        std::cout << "   - name: " << parent.getName() << std::endl;
        std::cout << "     id: " << parent.getSuffix() << std::endl;
        std::cout << "     required: " << parent.getRequired() << std::endl;
        std::cout << "     preempted_by: " << parent.getStopWhenReceived() << std::endl;
      }
    }

    if (!action.getChildren().empty())
    {
      std::cout << "   children: " << std::endl;

      for (const auto& child : action.getChildren())
      {
        std::cout << "   - name: " << child.getName() << std::endl;
        std::cout << "     id: " << child.getSuffix() << std::endl;
      }
    }

    if (!action.getInputParameters().empty())
    {
      std::cout << "   input_parameters: " << std::endl;

      for (const auto& parameter : action.getInputParameters())
      {
        std::cout << "   - name: " << parameter.getName() << std::endl;
        std::cout << "     type: " << parameter.getType() << std::endl;

        if (parameter.getDataSize() != 0)
        {
          const auto& data = parameter.getData();
          if (parameter.getType() == "string")
          {
            std::cout << "     value: " << boost::any_cast<std::string>(data) << std::endl;
          }
          else if (parameter.getType() == "number")
          {
            std::cout << "     value: " << boost::any_cast<double>(data) << std::endl;
          }
          else if (parameter.getType() == "bool")
          {
            std::cout << "     value: " << boost::any_cast<bool>(data) << std::endl;
          }
        }

        if (parameter.getAllowedData().size() != 0)
        {
          std::cout << "     allowed_values: ";
          for (const auto& allowed_data : parameter.getAllowedData())
          {
            if (parameter.getType() == "string")
            {
              std::cout << boost::any_cast<std::string>(allowed_data) << ", ";
            }
            else if (parameter.getType() == "number")
            {
              std::cout << boost::any_cast<double>(allowed_data) << ", ";
            }
            else if (parameter.getType() == "bool")
            {
              std::cout << boost::any_cast<bool>(allowed_data) << ", ";
            }
          }
        }
      }
    }
    std::cout << std::endl;
  }
}