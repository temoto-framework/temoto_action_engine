#include "ros/ros.h"
#include "ros/package.h"
#include "temoto_action_engine/action_engine.h"
#include "temoto_action_engine/temoto_error.h" 
#include "temoto_action_engine/umrf_json_converter.h"
#include "temoto_action_engine/messaging.h"
#include "temoto_action_engine/UmrfJsonGraph.h"
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>

class TemotoActionEngineNode
{
public:
  TemotoActionEngineNode()
  {}
  
  /**
   * @brief Reads in the commandline arguments for action base path and initiates the subscriber
   * 
   * @param argc 
   * @param argv 
   */
  bool initialize()
  {
    // Set up the UMRF graph subscriber to a globally namespaced topic
    umrf_graph_sub_ = nh_.subscribe("/umrf_graph_topic", 1, &TemotoActionEngineNode::umrfGraphCallback, this);

    // Set the default action paths
    int successful_paths = 0;
    for (const auto& ap : action_paths_)
    {
      try
      {
        ae_.addActionsPath(ap);
        successful_paths++;
      }
      catch(const std::exception& e)
      {
        TEMOTO_PRINT(e.what());
      }
    }

    if (successful_paths == 0)
    {
      TEMOTO_PRINT("None of the indicated directories contained TeMoto actions, exiting.");
      return false;
    }

    // Start the action engine
    ae_.start();

    // Execute the default action
    if (!default_umrf_.getName().empty())
    {
      try
      {
        ae_.executeUmrfGraph("default graph", std::vector<Umrf> {default_umrf_});
      }
      catch(const std::exception& e)
      {
        TEMOTO_PRINT(std::string(e.what()));
        return false;
      }
    }
    return true;
  }

  /**
   * @brief Parses the command line parameters
   * 
   * @param argc 
   * @param argv 
   * @return int 
   */
  int parseCmdArguments(int argc, char** argv)
  {
    namespace po = boost::program_options;
    po::variables_map vm;
    po::options_description desc("Allowed options");
    desc.add_options()
      ("help,h", "Show help message")
      ("w", po::value<std::string>(), "Optional. Additional wake words. Indicates to which wake words the action engine will respond to.")
      ("mw", po::value<std::string>(), "Required. Main wake word.")
      ("a", po::value<std::string>(), "Required. Path to action packages path file.")
      ("d", po::value<std::string>(), "Optional. Path to default UMRF that will be executed when the action engine starts up.");

    /* 
     * Process the arguments
     */ 
    try
    {
      po::store(po::parse_command_line(argc, argv, desc), vm);
      po::notify(vm);

      /*
       * Print help message
       */
      if (vm.count("help"))
      {
        std::cout << desc << std::endl;
        return 0;
      }

      /*
       * Get the wake words
       */ 
      if (vm.count("w"))
      {
        std::string wake_words_str = vm["w"].as<std::string>();
        boost::replace_all(wake_words_str, " ", "");
        boost::split(wake_words_, wake_words_str, boost::is_any_of(","));
      }

      /*
       * Get the action packages path file and get the paths    
       */
      if (vm.count("a"))
      {
        std::string action_uri_file_path = vm["a"].as<std::string>();

        /*
         * Open the action sources yaml file and get the paths to action libs
         * TODO: check for typos and other existance problems
         */
        YAML::Node config = YAML::LoadFile(action_uri_file_path);
        TEMOTO_PRINT("Indexing TeMoto actions from:");
        for (YAML::const_iterator it = config.begin(); it != config.end(); ++it)
        {
          std::string package_name = (*it)["package_name"].as<std::string>();
          std::string relative_path = (*it)["relative_path"].as<std::string>();
          std::string full_path = ros::package::getPath(package_name) + "/" + relative_path;
          action_paths_.push_back(full_path);
          std::cout << " * " << full_path << std::endl;
        }
        std::cout << std::endl;
      }
      else
      {
        TEMOTO_PRINT("Missing action packages path file");
        std::cout << desc << std::endl;
        return 1;
      }

      /*
       * Get the main wake word
       */ 
      if (vm.count("mw"))
      {
        std::string main_wake_word = vm["mw"].as<std::string>();
        wake_words_.push_back(main_wake_word);

        TEMOTO_PRINT("Wake words that this Action Engine Node responds to:");
        for (const auto& ww : wake_words_)
        {
          std::cout << " * " << ww << std::endl;
        }
        std::cout << std::endl;
      }
      else
      {
        std::cout << "Missing the main wake word" << std::endl;
        std::cout << desc << std::endl;
        return 1;
      }

      /*
       * Get the default umrf
       */ 
      if (vm.count("d"))
      {
        std::string default_umrf_path = vm["d"].as<std::string>();

        // Put the contents of umrf json file to a string 
        std::ifstream ifs(default_umrf_path);
        std::string umrf_json_str;
        umrf_json_str.assign(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());

        if (!umrf_json_str.empty())
        {
          // Convert the json string to UMRF data structure
          default_umrf_ = umrf_json_converter::fromUmrfJsonStr(umrf_json_str);
          TEMOTO_PRINT("Parsed default UMRF '" + default_umrf_.getFullName() + "'.");
          std::cout << default_umrf_;
        }
      }
      return 0;
    }
    catch (const std::exception& e)
    {
      std::cerr << e.what() << std::endl;
      std::cout << desc << std::endl;
      return 1;
    }
  }
private:

  /**
   * @brief Callback for executing UMRF graphs
   * 
   * @param msg 
   */
  void umrfGraphCallback(const temoto_action_engine::UmrfJsonGraph& msg)
  {
    TEMOTO_PRINT("Received a UMRF graph message ...");

    /*
     * Check for the wake word
     */
    bool wake_word_found = false;
    for (const auto& target : msg.targets)
    {
      for (const auto& ww : wake_words_)
      {
        if (ww == target)
        {
          wake_word_found = true;
          break;
        }  
      }
      if (wake_word_found)
      {
        break;
      }
    }

    // If the wake word was not found then return
    if (!wake_word_found)
    {
      TEMOTO_PRINT("The UMRF graph was not targeted at this Action Engine.");
      return;
    }

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
        umrf_vec.emplace_back(umrf);
      }
      ae_.executeUmrfGraph(msg.graph_name, umrf_vec);
    }
    catch(const std::exception& e)
    {
      TEMOTO_PRINT(std::string(e.what()));
    }
  }

  ActionEngine ae_;
  ros::NodeHandle nh_;
  ros::Subscriber umrf_graph_sub_;

  // Action Engine setup parameters
  Umrf default_umrf_;
  std::vector<std::string> action_paths_;
  std::vector<std::string> wake_words_; 
};

int main(int argc, char** argv)
{
  /*
   * Initialize ROS
   */
  std::cout << std::endl;
  std::cout << "* * * * * * * * * * * * * * * * * * * * " << std::endl;
  std::cout << "*        - ACTION ENGINE NODE -         " << std::endl;
  std::cout << "* * * * * * * * * * * * * * * * * * * * " << std::endl;

  ros::init(argc, argv, "temoto_action_engine_node");

  // Instantiate the Action Engine node object
  TemotoActionEngineNode action_engine_node;

  // Process command line arguments
  int return_code = action_engine_node.parseCmdArguments(argc, argv);
  if (return_code != 0)
  {
    return return_code;
  }

  // Initialize the action engine
  if (!action_engine_node.initialize())
  {
    return 1;
  }

  /*
   * Set up the spinner
   */
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
