
#include <gtest/gtest.h>
#include <math.h>
#include <temoto_action_engine/action_engine.h>

TEST(EngineTest, Lifecycle) 
{ 
  ActionEngine ae("ae_instance_1");
  // std::string ug_json_str = temoto_action_engine::readFromFile("example_b.json");
  // UmrfGraph ug_a = umrf_json::fromUmrfGraphJsonStr(ug_json_str);
  // std::string ug_json_str_new = umrf_json::toUmrfGraphJsonStr(ug_a);

  // json ug_json_original = json::parse(ug_json_str);
  // json ug_json_converted = json::parse(ug_json_str_new);

  // ASSERT_EQ(ug_json_original, ug_json_converted);
}