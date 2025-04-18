#include <temoto_action_engine/action_engine.h>
#include <temoto_action_engine/umrf_graph_fs.h>

#include <gtest/gtest.h>
#include <math.h>
#include <nlohmann/json.hpp>

TEST(SemanticsTest, Lifecycle)
{
  ActionEngine ae("ae_instance_1");
}

TEST(SemanticsTest, SimpleAction)
{
  std::string graph_name = "semantics_test_1";
  std::string expected_result = "on_true";

  ActionEngine ae("ae_instance_1");
  ae.addActionsPath(".");

  ae.startGraph(graph_name);
  std::string result = ae.waitForGraph(graph_name);

  ASSERT_EQ(result, expected_result);
}

TEST(SemanticsTest, SimpleSequence)
{
  std::string graph_name = "semantics_test_2";
  std::string expected_result = "on_true";

  ActionEngine ae("ae_instance_1");
  ae.addActionsPath(".");

  ae.startGraph(graph_name);
  std::string result = ae.waitForGraph(graph_name);

  ASSERT_EQ(result, expected_result);
}

TEST(SemanticsTest, SimpleConcurrency)
{
  std::string graph_name = "semantics_test_3";
  std::string expected_result = "on_true";

  ActionEngine ae("ae_instance_1");
  ae.addActionsPath(".");

  ae.startGraph(graph_name);
  std::string result = ae.waitForGraph(graph_name);

  ASSERT_EQ(result, expected_result);
}

TEST(SemanticsTest, SimpleConditionals)
{
  std::string graph_name = "semantics_test_4";
  std::string expected_result = "on_true";

  ActionEngine ae("ae_instance_1");
  ae.addActionsPath(".");

  ae.startGraph(graph_name);
  std::string result = ae.waitForGraph(graph_name);

  ASSERT_EQ(result, expected_result);
}

TEST(SemanticsTest, InvokeStop)
{
  std::string graph_name = "semantics_test_5";
  std::string expected_result = "on_stopped";

  ActionEngine ae("ae_instance_1");
  ae.addActionsPath(".");

  ae.startGraph(graph_name);
  std::string result = ae.waitForGraph(graph_name);

  ASSERT_EQ(result, expected_result);
}

TEST(SemanticsTest, SimpleHierarchy)
{
  std::string graph_name = "semantics_test_6";
  std::string expected_result = "on_true";

  ActionEngine ae("ae_instance_1");
  ae.addActionsPath(".");

  ae.startGraph(graph_name);
  std::string result = ae.waitForGraph(graph_name);

  ASSERT_EQ(result, expected_result);
}

TEST(SemanticsTest, SimpleReactiveBehavior)
{
  std::string graph_name = "semantics_test_7";
  std::string expected_result = "on_true";

  ActionEngine ae("ae_instance_1");
  ae.addActionsPath(".");

  ae.startGraph(graph_name);
  std::string result = ae.waitForGraph(graph_name);

  ASSERT_EQ(result, expected_result);
}

TEST(SemanticsTest, Messages)
{
  std::string graph_name = "semantics_test_8";
  std::string expected_result = "on_true";

  ActionEngine ae("ae_instance_1");
  ae.addActionsPath(".");

  ae.startGraph(graph_name);
  std::string result = ae.waitForGraph(graph_name);

  std::string graph_str = ae.getGraphJsonsRunning().at(0);
  nlohmann::json graph_json = nlohmann::json::parse(graph_str);

  auto messages_str = graph_json.at("actions").at(0).at("input_parameters").at("messages").at("pvf_value").get<std::vector<std::string>>();
  auto messages_str_json = [&]
  {
    std::vector<std::string> v;
    for (const auto& m_j :  graph_json.at("actions").at(0).at("runtime").at("messages"))
    {
      v.push_back(m_j.at("message"));
    }
    return v;
  }();

  bool all_messages_correct = [&]
  {
    for (const auto m_s : messages_str)
    {
      std::cout << " * Checking message: '" << m_s << "'" << std::endl;
      if (std::find(messages_str_json.begin(), messages_str_json.end(), m_s) == messages_str_json.end())
      {
        return false;
      }
    }
    return true;
  }();

  if (all_messages_correct)
  {
    std::cout << "All messages are properly stored" << std::endl;
  }

  EXPECT_TRUE(all_messages_correct);
}

TEST(SemanticsTest, HierarchyWithStop)
{
  std::string graph_name = "semantics_test_9";
  std::string expected_result = "on_stopped";

  ActionEngine ae("ae_instance_1");
  ae.addActionsPath(".");

  ae.startGraph(graph_name);
  std::string result = ae.waitForGraph(graph_name);

  ASSERT_EQ(result, expected_result);
}
