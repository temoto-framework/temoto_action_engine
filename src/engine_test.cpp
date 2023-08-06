

#include <temoto_action_engine/action_engine.h>


void runGraph(ActionEngine& engine, const std::string& graph_name, const std::string& expected_result)
{
  std::cout << "* TEST STARTED ---------------------------------- \n" << std::endl;

  std::cout << "Executing graph '" << graph_name << "'" << std::endl;
  engine.executeUmrfGraph(graph_name);

  std::cout << "Waiting for graph '" << graph_name << "' to finish ..." << std::endl;
  std::string result = engine.waitForGraph(graph_name);

  std::cout << graph_name << " finished with result '" << result << "'\n" << std::endl;
  (result == expected_result) ? std::cout << "SUCCESS\n" : std::cout << "FAIL\n";
  std::cout << "* TEST FINISHED ----------------------------------  \n" << std::endl;
}

int main()
{
  ActionEngine ae("ae_instance_1");

  std::string path = "../test/build";
  std::cout << "setting actions and graphs path to '" << path << "'" << std::endl;
  ae.addActionsPath("../test/build");

  runGraph(ae, "engine_test_1", "on_true");
  runGraph(ae, "engine_test_2", "on_true");
  runGraph(ae, "engine_test_3", "on_true");
  runGraph(ae, "engine_test_4", "on_true");
  runGraph(ae, "engine_test_5", "on_stopped");
  runGraph(ae, "engine_test_6", "on_true");

  return 0;
}