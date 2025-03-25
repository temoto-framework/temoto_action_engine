
#include <gtest/gtest.h>
#include <math.h>
#include <temoto_action_engine/action_engine.h>
#include <temoto_action_engine/umrf_graph_fs.h>

// TEST(EngineTest, Lifecycle)
// {
//   ActionEngine ae("ae_instance_1");
// }

// TEST(EngineTest, SimpleAction)
// {
//   std::string graph_name = "engine_test_1";
//   std::string expected_result = "on_true";

//   ActionEngine ae("ae_instance_1");
//   ae.addActionsPath(".");

//   ae.executeUmrfGraph(graph_name);
//   std::string result = ae.waitForGraph(graph_name);

//   ASSERT_EQ(result, expected_result);
// }

// TEST(EngineTest, SimpleSequence)
// {
//   std::string graph_name = "engine_test_2";
//   std::string expected_result = "on_true";

//   ActionEngine ae("ae_instance_1");
//   ae.addActionsPath(".");

//   ae.executeUmrfGraph(graph_name);
//   std::string result = ae.waitForGraph(graph_name);

//   ASSERT_EQ(result, expected_result);
// }

// TEST(EngineTest, SimpleConcurrency)
// {
//   std::string graph_name = "engine_test_3";
//   std::string expected_result = "on_true";

//   ActionEngine ae("ae_instance_1");
//   ae.addActionsPath(".");

//   ae.executeUmrfGraph(graph_name);
//   std::string result = ae.waitForGraph(graph_name);

//   ASSERT_EQ(result, expected_result);
// }

// TEST(EngineTest, SimpleConditionals)
// {
//   std::string graph_name = "engine_test_4";
//   std::string expected_result = "on_true";

//   ActionEngine ae("ae_instance_1");
//   ae.addActionsPath(".");

//   ae.executeUmrfGraph(graph_name);
//   std::string result = ae.waitForGraph(graph_name);

//   ASSERT_EQ(result, expected_result);
// }

// TEST(EngineTest, InvokeStop)
// {
//   std::string graph_name = "engine_test_5";
//   std::string expected_result = "on_stopped";

//   ActionEngine ae("ae_instance_1");
//   ae.addActionsPath(".");

//   ae.executeUmrfGraph(graph_name);
//   std::string result = ae.waitForGraph(graph_name);

//   ASSERT_EQ(result, expected_result);
// }

// TEST(EngineTest, SimpleHierarchy)
// {
//   std::string graph_name = "engine_test_6";
//   std::string expected_result = "on_true";

//   ActionEngine ae("ae_instance_1");
//   ae.addActionsPath(".");

//   ae.executeUmrfGraph(graph_name);
//   std::string result = ae.waitForGraph(graph_name);

//   ASSERT_EQ(result, expected_result);
// }

// TEST(EngineTest, SimpleReactiveBehavior)
// {
//   std::string graph_name = "engine_test_7";
//   std::string expected_result = "on_true";

//   ActionEngine ae("ae_instance_1");
//   ae.addActionsPath(".");

//   ae.executeUmrfGraph(graph_name);
//   std::string result = ae.waitForGraph(graph_name);

//   ASSERT_EQ(result, expected_result);
// }

// TEST(EngineTest, PauseResume)
// {
//   std::string graph_name = "engine_test_8";
//   std::string expected_result = "on_true";

//   ActionEngine ae("ae_instance_1");
//   ae.addActionsPath(".");

//   ae.executeUmrfGraph(graph_name);

//   // Pause the graph for two seconds
//   std::thread pause_resume_thread([&]
//   {
//     // Give the graph some time to initialize before pausing
//     std::this_thread::sleep_for(std::chrono::milliseconds(500));

//     ae.pauseUmrfGraph(graph_name);
//     std::this_thread::sleep_for(std::chrono::milliseconds(2000));

//     ae.resumeUmrfGraph(graph_name);
//   });

//   std::string result = ae.waitForGraph(graph_name);
//   pause_resume_thread.join();

//   ASSERT_EQ(result, expected_result);
// }

TEST(EngineTest, ModifyGraph)
{
  std::string graph_name = "engine_test_9";
  std::string expected_result = "on_true";

  std::string modified_graph_json_str{temoto_action_engine::readFromFile("engine_test_9_modified.graph.json")};
  UmrfGraph modified_graph{umrf_json::fromUmrfGraphJsonStr(modified_graph_json_str)};

  ActionEngine ae("ae_instance_1");
  ae.addActionsPath(".");

  ae.executeUmrfGraph(graph_name);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  ae.modifyGraph(modified_graph);

  std::string result = ae.waitForGraph(graph_name);

  ASSERT_EQ(result, expected_result);
}
