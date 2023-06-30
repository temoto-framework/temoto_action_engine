
#include <gtest/gtest.h>
#include <math.h>
#include <nlohmann/json.hpp>
#include <temoto_action_engine/umrf_json_converter.h>
#include <temoto_action_engine/umrf_graph_fs.h>

using json = nlohmann::json;

TEST(UmrfJsonTest, Compare) 
{ 
  std::string ug_json_str = temoto_action_engine::readFromFile("example_b.json");
  UmrfGraph ug_a = umrf_json::fromUmrfGraphJsonStr(ug_json_str);
  std::string ug_json_str_new = umrf_json::toUmrfGraphJsonStr(ug_a);

  json ug_json_original = json::parse(ug_json_str);
  json ug_json_converted = json::parse(ug_json_str_new);

  ASSERT_EQ(ug_json_original, ug_json_converted);
}