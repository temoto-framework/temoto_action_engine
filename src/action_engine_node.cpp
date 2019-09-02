#include "ros/ros.h"
#include "temoto_action_engine/action_executor.h"
#include "temoto_action_engine/action_indexer.h"
#include "temoto_action_engine/action_match_finder.h"
#include "temoto_action_engine/temoto_error.h" 
#include "temoto_action_engine/umrf_json_converter.h"
#include "temoto_action_engine/messaging.h"
#include "temoto_action_engine/UmrfJsonGraph.h"

class TemotoActionEngineNode
{
public:
  TemotoActionEngineNode()
  {}

  ~TemotoActionEngineNode()
  {
    ae_.stopAndCleanUp();
  }
  
  /**
   * @brief Reads in the commandline arguments for action base path and initiates the subscriber
   * 
   * @param argc 
   * @param argv 
   */
  bool initializeActionEngine(int argc, char** argv)
  {
    // Extract the arguments
    if (!(argc == 2 || argc == 4))
    {
      std::cout << "Missing argument for action base path\n";
      return false;
    }

    std::string base_path(argv[1]);

    umrf_graph_sub_ = nh_.subscribe("umrf_graph_topic", 1, &TemotoActionEngineNode::umrfGraphCallback, this);
    ai_.addActionPath(base_path);
    ai_.indexActions();
    TEMOTO_PRINT("Action Engine is good to go");
    return true;
  }
private:

  /**
   * @brief Callback for executing UMRF graphs
   * 
   * @param msg 
   */
  void umrfGraphCallback(const temoto_action_engine::UmrfJsonGraph& msg)
  {
    /*
     * Read the UMRF jsons and parse them to Umrf
     */
    try
    {
      std::vector<Umrf> umrf_vec;
      for (const auto& umrf_json_str : msg.umrf_json_strings)
      {
        Umrf umrf = umrf_json_converter::fromUmrfJsonStr(umrf_json_str);
        TEMOTO_PRINT("Parsed " + umrf.getFullName());
        std::cout << umrf;

        // Find a matching action for this UMRF
        if (!amf_.findMatchingAction(umrf, ai_.getUmrfs()))
        {
          throw CREATE_TEMOTO_ERROR("Could not find a matching action for UMRF named " + umrf.getName());
        }

        umrf_vec.emplace_back(umrf);
      }
      TEMOTO_PRINT("All UMRFs parsed successfully");

      ae_.addUmrfGraph(msg.graph_name, umrf_vec);
      TEMOTO_PRINT("UMRF graph initialized");

      ae_.executeUmrfGraph(msg.graph_name);
      TEMOTO_PRINT("UMRF graph executed successfully");
    }
    catch(const std::exception& e)
    {
      TEMOTO_PRINT(std::string(e.what()));
    }
  }

  ActionExecutor ae_;
  ActionIndexer ai_;
  ActionMatchFinder amf_;

  ros::NodeHandle nh_;
  ros::Subscriber umrf_graph_sub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "temoto_action_engine_node");
  TemotoActionEngineNode action_engine_node;
  if (!action_engine_node.initializeActionEngine(argc, argv))
  {
    return 1;
  }

  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
