#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

#include "temoto_action_engine/umrf_node.h"
#include "temoto_action_engine/umrf_graph.h"
#include "temoto_action_engine/umrf_graph_fs.h"

using json = nlohmann::json;

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
    if (parameters_json.contains("pvf_allowed_values"))
    try
    {
      for (const auto& allowed_value_json : parameters_json.at("pvf_allowed_values"))
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
    if (parameters_json.contains("pvf_value")) 
    try
    {
      pc.setData(toParameterData(pc.getType(), parameters_json.at("pvf_value")));
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
    if (!relation_json.contains("name"))
    {
      throw CREATE_TEMOTO_ERROR_STACK("Relation must contain a name");
    }
    relation.name_ = relation_json.at("name");

    // GET INSTANCE ID - not required if there are no duplicate instances
    try
    {
      // Default to '0'
      relation.instance_id_ = (relation_json.contains("instance_id")) ? uint16_t (relation_json.at("instance_id")) : 0;
    }
    catch(const std::exception& e)
    {
      throw CREATE_TEMOTO_ERROR_STACK("Invalid 'instance_id': " + std::string(e.what()));
    }

    // GET REQUIRED
    try
    {
      // Default to false
      relation.required_ = (relation_json.contains("required")) ? bool(relation_json.at("required")) : true;
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
    if (!relation_json.contains("name"))
    {
      throw CREATE_TEMOTO_ERROR_STACK("Relation must contain a name");
    }
    relation.name_ = relation_json.at("name");

    // GET INSTANCE ID - not required if there are no duplicate instances
    try
    {
      // Default to '0'
      relation.instance_id_ = (relation_json.contains("instance_id")) ? uint16_t (relation_json.at("instance_id")) : 0;
    }
    catch(const std::exception& e)
    {
      throw CREATE_TEMOTO_ERROR_STACK("Invalid 'instance_id': " + std::string(e.what()));
    }

    // GET CONDITION
    try
    {
      // Default to "always -> run"
      relation.condition_ = (relation_json.contains("condition")) ? relation_json.at("condition") : "always -> run";
    }
    catch(const std::exception& e)
    {
      throw CREATE_TEMOTO_ERROR_STACK("Invalid 'condition':" + std::string(e.what()));
    }

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
  if (!un.setType(umrf_json.at("type")))
  {
    throw CREATE_TEMOTO_ERROR_STACK("Illegal value in 'type' field.");
  }

  /*
   * Parse the optional fields
   */

  // GET ACTOR
  if (umrf_json.contains("actor"))
  try
  {
    un.setActor(umrf_json.at("actor"));
  }
  catch(const std::exception& e)
  {
    throw CREATE_TEMOTO_ERROR_STACK("Invalid 'actor' in " + un.getName() + ": " + std::string(e.what()));
  }

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
  if (umrf_json.contains("instance_id"))
  try
  {
    un.setInstanceId(umrf_json.at("instance_id"));
  }
  catch(const std::exception& e)
  {
    throw CREATE_TEMOTO_ERROR_STACK("Invalid 'instance_id' in " + un.getName() + ": " + std::string(e.what()));
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
    graph_entry = parseChildRelations(ug_json["graph_entry"]);
    graph_exit = parseParentRelations(ug_json["graph_exit"]);
  }

  // Create a new action that represents the entry 
  UmrfNode graph_entry_action;
  graph_entry_action.setName("graph_entry");
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
  graph_exit_action.setName("graph_exit");
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

std::string toUmrfGraphJsonStr(const UmrfGraph& ug)
{
  return "TODO";
}

void printParameters(const ActionParameters params)
{
  for (const auto& parameter : params)
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
      else if (parameter.getType() == "strings")
      {
        std::cout << "     value: ";
        for (const auto& data_item : boost::any_cast<std::vector<std::string>>(data))
        {
          std::cout << data_item << ", ";
        }
        std::cout << std::endl;
      }
      else if (parameter.getType() == "number")
      {
        std::cout << "     value: " << boost::any_cast<double>(data) << std::endl;
      }
      else if (parameter.getType() == "numbers")
      {
        std::cout << "     value: ";
        for (const auto& data_item : boost::any_cast<std::vector<double>>(data))
        {
          std::cout << data_item << ", ";
        }
        std::cout << std::endl;
      }
      else if (parameter.getType() == "bool")
      {
        std::cout << "     value: " << boost::any_cast<bool>(data) << std::endl;
      }
      else if (parameter.getType() == "bools")
      {
        std::cout << "     value: ";
        for (const auto& data_item : boost::any_cast<std::vector<bool>>(data))
        {
          std::cout << data_item << ", ";
        }
        std::cout << std::endl;
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
      std::cout << std::endl;
    }
  }
}

int main()
{
  std::string ug_json_str = temoto_action_engine::readFromFile("example_b.json");
  UmrfGraph ug = fromUmrfGraphJsonStr(ug_json_str);
  std::string ug_json_str_new = toUmrfGraphJsonStr(ug);

  std::cout << "graph_name: " << ug.getName() << std::endl;
  std::cout << "graph_description: " << ug.getDescription() << std::endl;


  std::cout << "actions: " << std::endl;

  for (const auto& action : ug.getUmrfNodes())
  {
    std::cout << " - name: " << action.getName() << std::endl;
    std::cout << "   instance_id: " << action.getInstanceId() << std::endl;
    std::cout << "   type: " << action.getType() << std::endl;

    if (!action.getActor().empty())
    {
      std::cout << "   actor: " << action.getActor() << std::endl;
    }

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
        std::cout << "     instance_id: " << parent.getInstanceId() << std::endl;
        std::cout << "     required: " << parent.getRequired() << std::endl;
      }
    }

    if (!action.getChildren().empty())
    {
      std::cout << "   children: " << std::endl;

      for (const auto& child : action.getChildren())
      {
        std::cout << "   - name: " << child.getName() << std::endl;
        std::cout << "     instance_id: " << child.getInstanceId() << std::endl;
        std::cout << "     condition: " << child.getCondition() << std::endl;
      }
    }

    if (!action.getInputParameters().empty())
    {
      std::cout << "   input_parameters: " << std::endl;
      printParameters(action.getInputParameters());
    }

    if (!action.getOutputParameters().empty())
    {
      std::cout << "   output_parameters: " << std::endl;
      printParameters(action.getOutputParameters());
    }

    std::cout << std::endl;
  }
}