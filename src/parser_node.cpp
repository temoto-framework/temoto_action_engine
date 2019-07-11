#include "ros/ros.h"
#include "temoto_action_engine/action_executor.h"
#include "temoto_action_engine/temoto_error.h"
#include "temoto_action_engine/umrf_json_converter.h"
#include "temoto_action_engine/messaging.h"
#include "temoto_action_engine/UmrfJsonGraph.h"
#include <sstream>
#include <fstream>

const std::string app_name = "AE_test";

int main(int argc, char** argv)
{
  if (!(argc == 2 || argc == 4))
  {
    std::cout << "Missing argument for json base path\n";
    return 1;
  }

  ros::init(argc, argv, "temoto_parser_node");
  ros::NodeHandle nh;
  ros::Publisher umrf_graph_pub = nh.advertise<temoto_action_engine::UmrfJsonGraph>("umrf_graph_topic", 1);

  std::string base_path(argv[1]);
  //std::vector<std::string> umrf_names = {"umrf_0.json", "umrf_1.json", "umrf_2.json", "umrf_3.json"};
  std::vector<std::string> umrf_names = {"umrf_google_parser.json"};

  /*
   * Read the UMRF jsons
   */ 
  temoto_action_engine::UmrfJsonGraph ujg_msg;
  ujg_msg.graph_name = "graph test 101";
  for (const auto& json_name : umrf_names)
  {
    std::string umrf_full_path = base_path + json_name;
    std::ifstream ifs(umrf_full_path);
    std::string umrf_json_str;
    umrf_json_str.assign(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());

    std::cout << "GOT:\n" << umrf_json_str << std::endl;

    ujg_msg.umrf_json_strings.push_back(umrf_json_str);
  }

  /*
   * Publish the UMRF JSON graph
   */
  umrf_graph_pub.publish(ujg_msg);
  ros::Duration(1).sleep();
  umrf_graph_pub.publish(ujg_msg);

  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
