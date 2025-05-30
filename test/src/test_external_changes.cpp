
#include <gtest/gtest.h>
#include <math.h>
#include <temoto_action_engine/action_engine.h>
#include <temoto_action_engine/umrf_graph_fs.h>

TEST(ExternalChangesTest, Stop)
{
  std::string graph_name = "external_changes_test_0";
  std::string expected_result = "on_stopped";

  ActionEngine ae("ae_instance_1");
  ae.addActionsPath(".");

  ae.startGraph(graph_name);

  // Pause the graph for two seconds
  std::thread pause_resume_thread([&]
  {
    // Give the graph some time to initialize before pausing
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    ae.stopGraph(graph_name);
  });

  std::string result = ae.waitForGraph(graph_name);
  pause_resume_thread.join();

  ASSERT_EQ(result, expected_result);
}

TEST(ExternalChangesTest, PauseResume)
{
  std::string graph_name = "external_changes_test_1";
  std::string expected_result = "on_true";

  ActionEngine ae("ae_instance_1");
  ae.addActionsPath(".");

  ae.startGraph(graph_name);

  // Pause the graph for two seconds
  std::thread pause_resume_thread([&]
  {
    // Give the graph some time to initialize before pausing
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    ae.pauseGraph(graph_name);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    ae.resumeGraph(graph_name);
  });

  std::string result = ae.waitForGraph(graph_name);
  pause_resume_thread.join();

  ASSERT_EQ(result, expected_result);
}

TEST(ExternalChangesTest, ModifyGraph)
{
  std::string graph_name = "external_changes_test_2";
  std::string expected_result = "on_true";

  std::string modified_graph_json_str{temoto_action_engine::readFromFile("external_changes_test_2_b.graph.json")};
  UmrfGraph modified_graph{umrf_json::fromUmrfGraphJsonStr(modified_graph_json_str)};

  ActionEngine ae("ae_instance_1");
  ae.addActionsPath(".");

  ae.startGraph(graph_name);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  ae.modifyGraph(modified_graph);

  std::string result = ae.waitForGraph(graph_name);

  ASSERT_EQ(result, expected_result);
}

TEST(ExternalChangesTest, ModifyGraphAndSet)
{
  std::string graph_name = "external_changes_test_3";
  std::string expected_result_before = "on_halted";
  std::string expected_result_after = "on_true";

  std::string modified_graph_json_str{temoto_action_engine::readFromFile("external_changes_test_3_b.graph.json")};
  UmrfGraph modified_graph{umrf_json::fromUmrfGraphJsonStr(modified_graph_json_str)};
  modified_graph.setName("external_changes_test_3");

  ActionEngine ae("ae_instance_1");
  ae.addActionsPath(".");
  std::string result;

  ae.startGraph(graph_name);
  result = ae.waitForGraph(graph_name);
  ASSERT_EQ(result, expected_result_before);

  ae.modifyGraph(modified_graph, "TaRelayString_0");
  result = ae.waitForGraph(graph_name);
  ASSERT_EQ(result, expected_result_after);
}

TEST(ExternalChangesTest, HaltOnTrue)
{
  std::string graph_name = "external_changes_test_3_c";
  std::string expected_result_before = "on_halted";

  ActionEngine ae("ae_instance_1");
  ae.addActionsPath(".");
  std::string result;

  ae.startGraph(graph_name);
  result = ae.waitForGraph(graph_name);
  ASSERT_EQ(result, expected_result_before);
}

TEST(ExternalChangesTest, ModifyGraphAndSetAfter)
{
  std::string graph_name = "external_changes_test_4";
  std::string expected_result_before = "on_halted";
  std::string expected_result_after = "on_true";

  std::string modified_graph_json_str{temoto_action_engine::readFromFile("external_changes_test_4_b.graph.json")};
  UmrfGraph modified_graph{umrf_json::fromUmrfGraphJsonStr(modified_graph_json_str)};
  modified_graph.setName("external_changes_test_4");

  ActionEngine ae("ae_instance_1");
  ae.addActionsPath(".");
  std::string result;

  ae.startGraph(graph_name);
  result = ae.waitForGraph(graph_name);
  ASSERT_EQ(result, expected_result_before);

  ae.modifyGraph(modified_graph, "TaRelayString_1");
  result = ae.waitForGraph(graph_name);
  ASSERT_EQ(result, expected_result_after);
}
