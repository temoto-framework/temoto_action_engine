

#include <temoto_action_engine/action_engine.h>

int main()
{
  ActionEngine ae("ae_instance_1");

  std::string path = "../test/build";
  std::cout << "setting actions and graphs path to '" << path << "'" << std::endl;
  ae.addActionsPath("../test/build");

  std::string graph_name = "engine_test_6";
  std::cout << "executing graph '" << graph_name << "'" << std::endl;
  ae.executeUmrfGraph(graph_name);

  std::cout << "waiting for graph '" << graph_name << "' to finish ..." << std::endl;
  std::string result = ae.waitForGraph(graph_name);
  std::cout << graph_name << " finished with result '" << result << "'" << std::endl;

  return 0;
}