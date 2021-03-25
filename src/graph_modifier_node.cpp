#include "ros/ros.h"
#include "temoto_action_engine/umrf_json_converter.h"
#include "temoto_action_engine/UmrfGraph.h"
#include "temoto_action_engine/UmrfGraphDiff.h"
#include <sstream>
#include <fstream>

int main(int argc, char** argv)
{
  if (!(argc == 5))
  {
    std::cout << "Missing arguments\n";
    return 1;
  }

  ros::init(argc, argv, "temoto_graph_modifier_node");
  ros::NodeHandle nh;
  ros::Publisher umrf_graph_pub = nh.advertise<temoto_action_engine::UmrfGraph>("umrf_graph_topic", 1);
  ros::AsyncSpinner spinner(0);
  spinner.start();

  // Get the commandline arguments
  std::string umrf_graph_name(argv[1]);
  std::string target(argv[2]);
  std::string operation(argv[3]);
  std::string umrf_json_file_path(argv[4]);

  ROS_INFO_STREAM("Targeting the message to '" << target << "'");

  /*
   * Read the UMRF Graph JSON
   */ 
  std::ifstream umrf_json_fs(umrf_json_file_path);
  std::string umrf_json_str;
  umrf_json_str.assign(std::istreambuf_iterator<char>(umrf_json_fs), std::istreambuf_iterator<char>());
  std::cout << "GOT:\n" << umrf_json_str << std::endl;
  Umrf umrf = umrf_json_converter::fromUmrfJsonStr(umrf_json_str);

  /*
   * Create UMRF Graph ROS message
   */ 
  temoto_action_engine::UmrfGraphDiff ugd;
  ugd.operation = operation;
  ugd.umrf_json = umrf_json_str;

  temoto_action_engine::UmrfGraph ujg_msg;
  ujg_msg.graph_name = umrf_graph_name;
  ujg_msg.targets.push_back(target);
  ujg_msg.umrf_graph_diffs.push_back(ugd);

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
    // Sleep for some time, so that "all" possible pub/sub connections are made
    ros::Duration(3).sleep();
    umrf_graph_pub.publish(ujg_msg);
  }

  ROS_INFO_STREAM("UMRF Graph message published, shutting down.");
  return 0;
}
