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
  if (!(argc == 4 || argc == 6))
  {
    std::cout << "Missing argument for json base path\n";
    return 1;
  }

  ros::init(argc, argv, "temoto_parser_node");
  ros::NodeHandle nh;
  ros::Publisher umrf_graph_pub = nh.advertise<temoto_action_engine::UmrfJsonGraph>("umrf_graph_topic", 1);
  ros::AsyncSpinner spinner(0);
  spinner.start();

  // Get the commandline arguments
  std::string base_path(argv[1]);
  std::string umrf_list_name(argv[2]);
  std::string target(argv[3]);

  ROS_INFO_STREAM("Targeting the message to '" << target << "'");

  std::ifstream umrf_list_fs(base_path + "/" + umrf_list_name);
  std::vector<std::string> umrf_names;

  // Extract the umrf names
  ROS_INFO_STREAM("Reading the UMRF file names from '" << umrf_list_name << "'");
  if (umrf_list_fs.is_open())
  {
    std::string umrf_name;
    while (getline(umrf_list_fs, umrf_name))
    {
      std::cout << " * umrf name: " << umrf_name << std::endl;
      umrf_names.push_back(umrf_name);
    }
    umrf_list_fs.close();
  }

  /*
   * Read the UMRF jsons
   */ 
  temoto_action_engine::UmrfJsonGraph ujg_msg;
  ujg_msg.graph_name = "graph_test_" + umrf_list_name;
  ujg_msg.targets.push_back(target);
  for (const auto& json_name : umrf_names)
  {
    std::string umrf_full_path = base_path + "/" + json_name;
    std::ifstream ifs(umrf_full_path);
    std::string umrf_json_str;
    umrf_json_str.assign(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());

    std::cout << "GOT:\n" << umrf_json_str << std::endl;

    ujg_msg.umrf_json_strings.push_back(umrf_json_str);
  }

  /*
   * Wait until there is somebody to publish the message to
   */
  ROS_INFO_STREAM("Waiting for subscribers ...");
  while (umrf_graph_pub.getNumSubscribers() <= 0 && ros::ok())
  {
    ros::Duration(0.1).sleep();
  }

  /*
   * Publish the UMRF JSON graph
   */
  ROS_INFO_STREAM("Publishing the UMRF Graph message ...");
  if (ros::ok())
  {
    umrf_graph_pub.publish(ujg_msg);
  }

  ROS_INFO_STREAM("UMRF Graph message published, shutting down.");
  return 0;
}
