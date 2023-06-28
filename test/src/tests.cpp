
#include <gtest/gtest.h>
#include <math.h>
#include <temoto_action_engine/umrf_json_converter.h>
#include <temoto_action_engine/umrf_graph_fs.h>

TEST(UmrfJsonTest, Load) 
{ 
  std::string ug_json_str = temoto_action_engine::readFromFile("example_b.json");
  UmrfGraph ug_a = umrf_json::fromUmrfGraphJsonStr(ug_json_str);
  
  std::string ug_json_str_new = umrf_json::toUmrfGraphJsonStr(ug_a);
  UmrfGraph ug_b = umrf_json::fromUmrfGraphJsonStr(ug_json_str);

  ASSERT_EQ(ug_a.getName(), ug_b.getName());
}
 
// TEST(SquareRootTest, NegativeNos) {
//     ASSERT_EQ(-1.0, squareRoot(-15.0));
//     ASSERT_EQ(-1.0, squareRoot(-0.2));
// }