#include "ros/ros.h"
#include "temoto_action_engine/temoto_error.h"
#include "temoto_action_engine/umrf_json_converter.h"
#include "temoto_action_engine/umrf_graph_fs.h"
#include "temoto_action_engine/BroadcastStartUmrfGraph.h"

const std::string app_name = "AE_test";

int main(int argc, char** argv)
{
  if (!(argc == 3 || argc == 5))
  {
    std::cout << "Missing umrf graph path argument \n";
    return 1;
  }

  ros::init(argc, argv, "temoto_parser_node");
  ros::NodeHandle nh;
  ros::Publisher umrf_graph_pub = nh.advertise<temoto_action_engine::BroadcastStartUmrfGraph>("/broadcast_start_umrf_graph", 1);
  ros::AsyncSpinner spinner(0);
  spinner.start();

  // Get the commandline arguments
  std::string umrf_graph_json_file(argv[1]);
  std::string target(argv[2]);

  ROS_INFO_STREAM("Targeting the message to '" << target << "'");

  /*
   * Read the UMRF Graph JSON
   */
  std::string umrf_graph_json_str = temoto_action_engine::readFromFile(umrf_graph_json_file);
  UmrfGraph umrf_graph = umrf_json_converter::fromUmrfGraphJsonStr(umrf_graph_json_str);
  std::cout << "GOT:\n" << umrf_graph_json_str << std::endl;

  /*
   * Create UMRF Graph ROS message
   */ 
  temoto_action_engine::BroadcastStartUmrfGraph ujg_msg;
  ujg_msg.umrf_graph_name = umrf_graph.getName();
  ujg_msg.name_match_required = 1;
  ujg_msg.targets.push_back(target);
  ujg_msg.umrf_graph_json = umrf_graph_json_str;

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
