#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

#include "temoto_action_engine/umrf_node.h"
#include "temoto_action_engine/umrf_graph.h"
#include "temoto_action_engine/umrf_graph_fs.h"

using json = nlohmann::json;

UmrfNode::Relation parseRelation(const json& relation_json)
{
  return UmrfNode::Relation();
}

std::vector<UmrfNode::Relation> parseRelations(const json& relations_json)
{
  if(!relations_json.is_array())
  {
    throw CREATE_TEMOTO_ERROR_STACK("The relations field must be an array");
  }

  std::vector<UmrfNode::Relation> relations;
  for (const auto& relation_json : relations_json)
  {
    relations.push_back(parseRelation(relation_json));
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

  // GET EFFECT
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
    std::vector<UmrfNode::Relation> parents = parseRelations(umrf_json["parents"]);
    un.setParents(parents);
  }
  catch(TemotoErrorStack e)
  {
    throw FORWARD_TEMOTO_ERROR_STACK_WMSG(e, "Invalid 'parents' in " + un.getName());
  }

  return UmrfNode();
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
  std::string graph_name;
  std::vector<UmrfNode> umrf_actions;

  graph_name = ug_json["graph_name"];

  for (auto& umrf_json : ug_json["actions"])
  {
    umrf_actions.push_back(fromUmrfJson(umrf_json));
  }

  /*
   * Parse the optional fields
   */

  return UmrfGraph(graph_name, umrf_actions);
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
}