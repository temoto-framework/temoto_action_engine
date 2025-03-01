
#include <gtest/gtest.h>
#include <math.h>
#include <temoto_action_engine/action_engine.h>

TEST(EngineTest, Lifecycle)
{
  ActionEngine ae("ae_instance_1");
}

TEST(EngineTest, SimpleAction)
{
  std::string graph_name = "engine_test_1";
  std::string expected_result = "on_true";

  ActionEngine ae("ae_instance_1");
  ae.addActionsPath(".");

  ae.executeUmrfGraph(graph_name);
  std::string result = ae.waitForGraph(graph_name);

  ASSERT_EQ(result, expected_result);
}

TEST(EngineTest, SimpleSequence)
{
  std::string graph_name = "engine_test_2";
  std::string expected_result = "on_true";

  ActionEngine ae("ae_instance_1");
  ae.addActionsPath(".");

  ae.executeUmrfGraph(graph_name);
  std::string result = ae.waitForGraph(graph_name);

  ASSERT_EQ(result, expected_result);
}

TEST(EngineTest, SimpleConcurrency)
{
  std::string graph_name = "engine_test_3";
  std::string expected_result = "on_true";

  ActionEngine ae("ae_instance_1");
  ae.addActionsPath(".");

  ae.executeUmrfGraph(graph_name);
  std::string result = ae.waitForGraph(graph_name);

  ASSERT_EQ(result, expected_result);
}

TEST(EngineTest, SimpleConditionals)
{
  std::string graph_name = "engine_test_4";
  std::string expected_result = "on_true";

  ActionEngine ae("ae_instance_1");
  ae.addActionsPath(".");

  ae.executeUmrfGraph(graph_name);
  std::string result = ae.waitForGraph(graph_name);

  ASSERT_EQ(result, expected_result);
}

TEST(EngineTest, InvokeStop)
{
  std::string graph_name = "engine_test_5";
  std::string expected_result = "on_stopped";

  ActionEngine ae("ae_instance_1");
  ae.addActionsPath(".");

  ae.executeUmrfGraph(graph_name);
  std::string result = ae.waitForGraph(graph_name);

  ASSERT_EQ(result, expected_result);
}

TEST(EngineTest, SimpleHierarchy)
{
  std::string graph_name = "engine_test_6";
  std::string expected_result = "on_true";

  ActionEngine ae("ae_instance_1");
  ae.addActionsPath(".");

  ae.executeUmrfGraph(graph_name);
  std::string result = ae.waitForGraph(graph_name);

  ASSERT_EQ(result, expected_result);
}

TEST(EngineTest, SimpleReactiveBehavior)
{
  std::string graph_name = "engine_test_7";
  std::string expected_result = "on_true";

  ActionEngine ae("ae_instance_1");
  ae.addActionsPath(".");

  ae.executeUmrfGraph(graph_name);
  std::string result = ae.waitForGraph(graph_name);

  ASSERT_EQ(result, expected_result);
}
